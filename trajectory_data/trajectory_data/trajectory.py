
from fastdtw import fastdtw
import numpy as np

import trajectory_data

import spatialmath as sm
from spatialmath.base import q2r
import matplotlib.pyplot as plt

class Skill():
    def __init__(self, 
                 traj= np.empty([3,0]), 
                 ori_wxyz = np.empty([4,0]), 
                 gripper = np.empty([1,0]), 
                 img = [], 
                 img_feedback_flag = np.empty([1,0]), 
                 spiral_flag = np.empty([1,0]), 
                 filename = "",
                 ):
        self.traj_T = traj
        self.ori_wxyz_T = ori_wxyz
        self.gripper_T = gripper
        self.img = img
        self.img_feedback_flag_T = img_feedback_flag
        self.spiral_flag_T = spiral_flag
        self.filename = filename

    def __eq__(self, other):
        if  (len(self.traj_T[0]) == len(other.traj_T[0])) and \
            (self.traj_T == other.traj_T).all() and \
            (self.ori_wxyz_T == other.ori_wxyz_T).all() and \
            (self.gripper_T == other.gripper_T).all() and \
            (self.img == other.img).all() and \
            (self.img_feedback_flag_T == other.img_feedback_flag_T).all() and \
            (self.spiral_flag_T == other.spiral_flag_T).all():
            return True
        else:
            return False

    def __str__(self):
        s = ""
        s += f"Skill file: {self.filename}\n"
        s += f"traj_T {self.traj_T.shape}\n"
        s += f"ori_wxyz_T {self.ori_wxyz_T.shape}\n"
        s += f"gripper_T {self.gripper_T.shape}\n"
        s += f"img {self.img.shape}\n"
        s += f"img_feedback_flag_T {self.img_feedback_flag_T.shape}\n"
        s += f"spiral_flag_T {self.spiral_flag_T.shape}\n"
        
        return s

    @classmethod
    def from_file(cls, file: str):
        """ Note: Not transformed path. """
        data = np.load(trajectory_data.package_path + '/trajectories/' + str(file) + '.npz')
        self = cls()
        self.traj_T = data['traj']
        self.ori_wxyz_T = data['ori']
        self.gripper_T = data['grip']
        self.img = data['img']
        self.img_feedback_flag_T = data['img_feedback_flag']
        self.spiral_flag_T = data['spiral_flag']
        self.filename=str(file)
        return self

    @property
    def img_feedback_flag(self):
        return self.img_feedback_flag_T.flatten()

    @property
    def spiral_flag(self):
        return self.spiral_flag_T.flatten()

    @property
    def traj(self):
        return self.traj_T.T
    
    @property
    def ori_wxyz(self):
        return self.ori_wxyz_T.T
    
    @property
    def gripper(self):
        return self.gripper_T.flatten()

    @property
    def poses(self): # returns as (7,x)
        return np.vstack((self.traj_T, self.ori_wxyz_T))

    def morth_trajectories(self, other, p: float):
        """Makes interpolated trajectory between two trajectories, where p=0 returns first trajectory and p=1 returns second trajectory

        Args:
            T1 (Dict of Numpy Arrays): 1. Trajectory
            T2 (Dict of Numpy Arrays): 2. Trajectory
            p (float): Morph parameter (0-1)

        Returns:
            (Dict of Numpy Arrays): Morphed trajectory
        """
        
        _, conn = fastdtw(self.poses.T, other.poses.T) # (x,7), (x,7)
        morphed_traj = Skill()
        
        for i,j in conn:
            blended_quaternion = slerp(self.ori_wxyz[i], other.ori_wxyz[j], p)

            morphed_traj.traj_T = np.c_[morphed_traj.traj_T,      (1 - p) * self.traj[i]     + p * other.traj[j]  ]
            morphed_traj.ori_wxyz_T = np.c_[morphed_traj.ori_wxyz_T,  blended_quaternion   ]
            morphed_traj.gripper_T = np.c_[morphed_traj.gripper_T,   (1 - p) * self.gripper[i]  + p * other.gripper[j]  ]
            morphed_traj.img_feedback_flag_T = np.c_[morphed_traj.img_feedback_flag_T, self.img_feedback_flag[i] or other.img_feedback_flag[j] ]
            morphed_traj.spiral_flag_T = np.c_[morphed_traj.spiral_flag_T, self.spiral_flag[i] or other.spiral_flag[j] ]
            morphed_traj.img.append(self.img[i])

        morphed_traj.img = np.array(morphed_traj.img)
        morphed_traj.filename = self.filename + "_morphed_with_" + other.filename
        return morphed_traj

    def poses_SE3(self):
        poses = []
        for t,q in zip(self.traj, self.ori_wxyz):
            # print(q2r(q), t)
            poses.append(sm.SE3.Rt(q2r(q), t))
        return poses

    def plot(self, color="blue", rot_plot_interval=100):
        i = 0
        poses = self.poses_SE3()
        p = poses[0]
        for se3 in poses:
            plt.plot([p.t[0], se3.t[0]], [p.t[1], se3.t[1]], zs=[p.t[2], se3.t[2]], color=color)
            p = se3
            if i % rot_plot_interval == 0:
                frame = f"{i}"
                se3.plot(style='rviz', width=2, length=(0.1, 0.1, 0.1), frame=frame, color=color, textcolor=color)
            i += 1

        plt.axis("equal")
        plt.tight_layout()
        

    def animate(self, color="blue"):
        se3_list = sm.SE3(self.poses_SE3())
        se3_list.animate(style='rviz', width=2, color=color, textcolor=color)
        plt.show()


def main():
    skill1 = Skill().from_file("open_drawer")
    skill2 = Skill().from_file("pick_lid")
    print(skill1.poses.shape)
    print(skill2.poses.shape)
    
    print(skill1 == skill2)

    print(skill1)
    print(skill2)

    skill_morphed = skill1.morth_trajectories(skill2, p = 0.5)

    print(skill_morphed)

    print(skill1 == skill_morphed)

    plt.rcParams["figure.figsize"] = (19,19)
    sm.base.plot_cuboid(sides=(1.4, 0.8, 0.9), centre=(0.4, 0, -0.45), color="blue")
    sm.base.plot_cone(radius=0.1, height=0.2, color="blue")
    skill1.plot()
    skill2.plot(color="red")
    skill_morphed.plot(color="green")
    plt.show()

    
    # skill1.animate()


def normalize(q):
    """Normalize a quaternion to unit length."""
    return q / np.linalg.norm(q)

def slerp(q1, q2, p, epsilon=1e-8):
    """
    Perform Spherical Linear Interpolation (SLERP) between two quaternions.
    
    Args:
        q1: First quaternion (4-element array-like).
        q2: Second quaternion (4-element array-like).
        p: Interpolation weight (0 <= p <= 1).
        epsilon: Threshold to prevent division by small numbers.
    
    Returns:
        Interpolated quaternion (numpy array).
    """
    q1 = normalize(np.array(q1, dtype=np.float64))
    q2 = normalize(np.array(q2, dtype=np.float64))
    
    # Compute the dot product to determine the angle between quaternions
    dot = np.dot(q1, q2)
    
    # Ensure shortest path (correct for negative dot product)
    if dot < 0.0:
        q2 = -q2
        dot = -dot
    
    # If the quaternions are almost the same, use linear interpolation
    if dot > 1.0 - epsilon:
        result = q1 + p * (q2 - q1)
        return normalize(result)
    
    # Compute the angle between the quaternions
    theta_0 = np.arccos(dot)
    theta = theta_0 * p
    
    # Compute the interpolated quaternion components
    q3 = normalize(q2 - q1 * dot)  # Orthogonal component
    
    return q1 * np.cos(theta) + q3 * np.sin(theta)

# Example usage:
q1 = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion
q2 = np.array([0.0, 1.0, 0.0, 0.0])  # 180-degree rotation around X-axis
p = 0.5  # Blend halfway

blended_quaternion = slerp(q1, q2, p)
print("Blended quaternion:", blended_quaternion)

if __name__ == "__main__":
    main()
