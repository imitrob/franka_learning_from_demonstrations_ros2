import numpy as np
import quaternion
import warnings
warnings.filterwarnings("ignore", message=".*The 'nopython' keyword.*")
from geometry_msgs.msg import PoseStamped, Pose
import math

def orientation_2_quaternion(orientation):
    return quaternion.quaternion(orientation.w, orientation.x, orientation.y, orientation.z)

def position_2_array(position):
    return np.array([position.x, position.y, position.z])

def pose_2_transformation(pose: Pose):
    quaternion_orientation = orientation_2_quaternion(pose.orientation)
    translation = position_2_array(pose.position)
    rotation_matrix = quaternion.as_rotation_matrix(quaternion_orientation)
    transformation_matrix = np.identity(4)
    transformation_matrix[0:3, 0:3] = rotation_matrix
    transformation_matrix[0:3, 3] = translation
    return transformation_matrix

def pos_quat_2_pose_st(pos_array, quat):
    pose_st = PoseStamped()
    pose_st.pose.position.x = pos_array[0]
    pose_st.pose.position.y = pos_array[1]
    pose_st.pose.position.z = pos_array[2]
    pose_st.pose.orientation.x = quat.x
    pose_st.pose.orientation.y = quat.y
    pose_st.pose.orientation.z = quat.z
    pose_st.pose.orientation.w = quat.w
    return pose_st

def transformation_2_pose(transformation_matrix):
    pos_array = transformation_matrix[0:3, 3]
    rotation_matrix = transformation_matrix[0:3, 0:3]
    quat = quaternion.from_rotation_matrix(rotation_matrix)
    pose_st = pos_quat_2_pose_st(pos_array, quat)
    return pose_st

def pose_st_2_transformation(pose_st: PoseStamped):
    transformation_matrix = pose_2_transformation(pose_st.pose)
    return transformation_matrix

def transform_pose(pose: PoseStamped, transformation_matrix):
    pose_as_matrix = pose_st_2_transformation(pose)
    transformed_pose_matrix = transformation_matrix @ pose_as_matrix
    transformed_pose = transformation_2_pose(transformed_pose_matrix)
    return transformed_pose

def transform_pos_ori(pos: np.array, ori, transform):
    ori_quat = list_2_quaternion(ori)
    ori_rot_matrix = quaternion.as_rotation_matrix(ori_quat)
    transformed_ori_rot_matrix = transform[:3,:3] @ ori_rot_matrix
    pos = np.hstack((pos, 1))
    transformed_pos = transform @ pos
    transformed_ori_quat = quaternion.from_rotation_matrix(transformed_ori_rot_matrix)
    transformed_ori_array = np.array([transformed_ori_quat.w, transformed_ori_quat.x, transformed_ori_quat.y, transformed_ori_quat.z])
    return transformed_pos[:3], transformed_ori_array

def list_2_quaternion(quaternion_list: list):
    return quaternion.quaternion(quaternion_list[0], quaternion_list[1], quaternion_list[2], quaternion_list[3])

def transform_between_poses(pose2: PoseStamped, pose1: PoseStamped):
    pose1_matrix = pose_st_2_transformation(pose1)
    pose2_matrix = pose_st_2_transformation(pose2)
    transform=pose2_matrix @ np.linalg.inv(pose1_matrix)
    return transform

def interpolate_poses(pose_start: PoseStamped, pose_goal: PoseStamped, interp_dist_linear, interp_dist_polar):
    position_goal = np.array([pose_goal.pose.position.x, pose_goal.pose.position.y, pose_goal.pose.position.z])
    position_start = np.array([pose_start.pose.position.x, pose_start.pose.position.y, pose_start.pose.position.z])
    dist_lin = np.sqrt(np.sum(np.subtract(position_start, position_goal)**2))
    # dist = np.sqrt(np.sum(np.subtract(start, goal_array)**2, axis=0))
    
    step_num_lin = int(np.ceil(dist_lin / interp_dist_linear))
    try:
        quaternion_start = list_2_quaternion([pose_start.pose.orientation.w, pose_start.pose.orientation.x, pose_start.pose.orientation.y, pose_start.pose.orientation.z])
        quaternion_goal = list_2_quaternion([pose_goal.pose.orientation.w, pose_goal.pose.orientation.x, pose_goal.pose.orientation.y, pose_goal.pose.orientation.z])

        quaternion_start_norm = np.sqrt(quaternion_start.x**2 + quaternion_start.y**2 + quaternion_start.z**2 + quaternion_start.w**2)
        quaternion_goal_norm = np.sqrt(quaternion_goal.x**2 + quaternion_goal.y**2 + quaternion_goal.z**2 + quaternion_goal.w**2)

        inner_prod= quaternion_start.x * quaternion_goal.x + quaternion_start.y * quaternion_goal.y + quaternion_start.z * quaternion_goal.z + quaternion_start.w * quaternion_goal.w
        if inner_prod < 0: quaternion_start = -quaternion_start
        inner_prod= quaternion_start.x * quaternion_goal.x + quaternion_start.y * quaternion_goal.y + quaternion_start.z * quaternion_goal.z + quaternion_start.w * quaternion_goal.w
        inner_prod= inner_prod / (quaternion_start_norm * quaternion_goal_norm)
        theta= np.arccos(np.abs(inner_prod))
    
        step_num_polar = int(np.ceil(theta / interp_dist_polar))
    except Exception as e:
        print("Zero division error in polar interpolation")
        print(f"Exception {e}")
        step_num_polar = 2
    
    step_num=np.max([2,np.max([step_num_polar,step_num_lin]) + 1])
    
    x = np.linspace(position_start[0], position_goal[0], step_num)
    y = np.linspace(position_start[1], position_goal[1], step_num)
    z = np.linspace(position_start[2], position_goal[2], step_num)

    poses = []
    for i in range(step_num):
        pos=np.array([x[i], y[i], z[i]])
        quat=np.slerp_vectorized(quaternion_start, quaternion_goal, i/(step_num-1))
        pose_st = pos_quat_2_pose_st(pos, quat)
        poses.append(pose_st)

    return poses

def invert_tf(T):
    """Invert a 4×4 homogeneous transform T more efficiently."""
    R = T[:3, :3]        # rotation part
    t = T[:3,  3]        # translation part
    R_inv = R.T          # inverse of a rotation is its transpose
    t_inv = -R_inv @ t   # new translation
    T_inv = np.eye(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3,  3] = t_inv
    return T_inv


def q_norm(wxyz):
    q = np.array(wxyz, dtype=float)
    n = np.linalg.norm(q)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or n == 0:
        raise ValueError("Quaternion must be finite and nonzero")
    q /= n
    if q[0] < 0: q = -q  # shortest-path hemisphere
    return q

def q_angle(q0, q1):
    dot = float(np.clip(np.dot(q0, q1), -1.0, 1.0))
    return 2.0 * math.acos(abs(dot))

def q_slerp(q0, q1, t):
    dot = float(np.clip(np.dot(q0, q1), -1.0, 1.0))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        q = q0 + t*(q1 - q0)
        return q / np.linalg.norm(q)
    th0 = math.acos(dot)
    st0 = math.sin(th0)
    th = th0 * t
    s0 = math.sin(th0 - th) / st0
    s1 = math.sin(th) / st0
    return s0*q0 + s1*q1

def build_quat_seq(q0, q1, N):
    ts = np.linspace(0.0, 1.0, N)
    return np.stack([q_slerp(q0, q1, t) for t in ts], axis=0)


def min_angle_condition(q1, q2):
    '''
    q1 : xyzw
    q2 : xyzw
    '''
    a, b = np.asarray(q1, dtype=float), np.asarray(q2, dtype=float)
    for q in (a, b):
        if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q) == 0:
            raise ValueError("Quaternion must be finite and nonzero")
    return q_angle(a / np.linalg.norm(a), b / np.linalg.norm(b))


def step_slerp(q1, q2, epsilon):
    '''
    q1 : xyzw
    q2 : xyzw
    returns q : xyzw
    '''
    # xyzw -> wxyz (used in quaternion)
    q1_ = quaternion.quaternion(*[q1[3], q1[0], q1[1], q1[2]])
    q2_ = quaternion.quaternion(*[q2[3], q2[0], q2[1], q2[2]]) 

    q_rel = q2_ * q1_.conjugate()
    if q_rel.w < 0:
        q_rel = -q_rel

    angle = 2.0 * np.arccos(np.clip(q_rel.w, -1.0, 1.0))

    t = epsilon / angle
    out_q = np.slerp_vectorized(q1_, q2_, t)
    # wxyz -> xyzw (back)
    return [out_q[1], out_q[2], out_q[3], out_q[0]]