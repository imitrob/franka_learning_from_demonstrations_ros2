#!/usr/bin/env python3
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from panda_control import SpinningRosNode
from panda_control.home_pose import HOME_POSE
from panda_control.pose_transform_functions import orientation_2_quaternion, pose_st_2_transformation, position_2_array, pos_quat_2_pose_st, transformation_2_pose, transform_pose, list_2_quaternion, transform_pos_ori, list_2_quaternion, pos_quat_2_pose_st

import tf_transformations
from lfd_msgs.srv import ComputeLocalization, GetScene
import numpy as np
import time
from copy import deepcopy
from tf_transformations import euler_from_quaternion
from queue import Queue
import rclpy
from rclpy.duration import Duration

from skills_manager.ros_param_manager import get_remote_parameters
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped

from object_localization.tf_utils import CustomTransformListener
# Reused rather than restated: "base" is the name scene consumers expect, while
# the TF chain is broadcast under panda_link0.
from object_localization.localizer_service import ROBOT_BASE_TF_FRAME, SCENE_FRAME_ID

CAMERA_COLOR_TOPIC = '/camera/color/image_raw'

# SIFT matching degrades as the camera leaves the canonical home pose, so this
# doubles as the position at which a scene can be believed.
SCENE_HOME_POSITION = np.asarray(HOME_POSE.position)
SCENE_NAME = "active_localizer_scene"

import threading
import scene_msgs.msg as scene_ros

class ActiveLocalizerNode(CustomTransformListener, SpinningRosNode):
    def __init__(self) -> None:
        super(ActiveLocalizerNode, self).__init__()

        self._imgs = Queue(maxsize=1)
        self._img = None
        self._rate = self.create_rate(5)
        self.image_sub = self.create_subscription(Image, CAMERA_COLOR_TOPIC, self.image_callback, 5)


        self.compute_box_tf = self.create_client(ComputeLocalization, 'compute_localization', callback_group=self.callback_group)
        while not self.compute_box_tf.wait_for_service(timeout_sec=1.0):
            print('service ("compute_localization") not available, waiting again...')

        self._go_to = True
        self._window = Queue(maxsize=10)
        self._service = self.create_service(Trigger, 'active_localizer', self.handle_request, qos_profile=QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT), callback_group=self.callback_group)

        self._start_publishing_scene_service = self.create_service(Trigger, 'start_publishing_scene', self.start_publishing_scene, qos_profile=QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT), callback_group=self.callback_group)
        self._stop_publishing_scene_service = self.create_service(Trigger, 'stop_publishing_scene', self.stop_publishing_scene, qos_profile=QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT), callback_group=self.callback_group)
        self.compute_scene_positions_client = self.create_client(GetScene, 'compute_object_positions', callback_group=self.callback_group)

        self.position_accuracy = 0.003
        self.orientation_accuracy=0.5 *(np.pi/180)
        self.timeout_counter_max = 10

        self.goal_pose_pub = self.create_publisher(PoseStamped, "/panda/goal_pose", 5)
        self.create_subscription(PoseStamped, "/panda/curr_pose", self.curr_pose_callback, 5)

        # self._prev_img = None
        self.img_last_rec = 0.0

        self.curr_pos = None
        self.curr_ori_wxyz = None

        self.scene_pub = self.create_publisher(scene_ros.Scene, "/scene", 5)
        # Off until asked, where "asked" now also means somebody subscribed.
        # start_publishing_scene / stop_publishing_scene remain the explicit
        # switch that lfd.py and SceneGetterViaObjectLocalizer throw; treating a
        # subscriber as the same request means opening the gestures dashboard is
        # enough to watch the scene, and closing it stops the SIFT work rather
        # than leaving it competing with the servo loop for nobody's benefit.
        self.publishing_scene = False
        self.declare_parameter("scene_rate", 1.0)
        self.declare_parameter("scene_home_tolerance", 0.05)

        # Last poses actually recognised, and the image stamp they were seen at.
        # Republished while the view cannot be trusted so that a consumer can
        # grey them out, instead of objects blinking out of existence every time
        # the arm leaves home.
        self._scene_objects = []
        self._scene_observed_at = None

        spinning_thread = threading.Thread(target=self.publish_scene_thread, args=(), daemon=True)
        spinning_thread.start()

    def _untrustworthy_view(self):
        """Why the current frame cannot be believed, or None when it can.

        The home check is the one that matters. Every template is captured with
        the arm at home, so SIFT stops matching as the view departs from it, and
        a miss then means "cannot see" rather than "not there". At home the
        distinction runs the other way: a template that stops matching really has
        been picked up or moved, and must vanish from the scene rather than
        linger at a pose it no longer occupies.
        """
        if self._img is None:
            return "no image yet"
        # Same freshness rule handle_request uses: a stale frame yields a stale
        # scene, and the server cannot tell the difference.
        if (time.time() - self.img_last_rec) > 1.0:
            return "image is not fresh"
        # The arm transform is what get_scene actually needs, and its absence is
        # the difference between "the table is empty" and "I have no idea where the
        # camera is". Checked here rather than left to get_scene because the
        # service can only answer with an empty response either way: without this
        # a missing transform silently overwrites the remembered scene with an
        # empty one and the dashboard shows nothing, with no reason given.
        translation, _ = self.lookup_relative_transform(ROBOT_BASE_TF_FRAME, "panda_hand")
        if translation is None:
            return (f"no {ROBOT_BASE_TF_FRAME} -> panda_hand transform; the camera "
                    f"rides on the end effector, so there is nowhere to put the "
                    f"objects (is the persistent lfd_server running?)")

        if self.curr_pos is None:
            # No pose feed to judge distance from home with. The transform above
            # is the hard requirement, so let it proceed rather than refusing on a
            # missing topic -- though without /panda/curr_pose the home check
            # below cannot run, and an off-home miss will look like a removal.
            return None
        distance = float(np.linalg.norm(np.array(self.curr_pos, dtype=float) - SCENE_HOME_POSITION))
        tolerance = float(self.get_parameter("scene_home_tolerance").value)
        if distance > tolerance:
            return f"arm is {distance:.3f} m from home (tolerance {tolerance:.3f} m)"
        return None

    def _recognise_objects(self, SceneObject):
        """Ask the localizer what it can see, as SceneObjects. None on failure."""
        scene_response = self.compute_scene_positions_client.call(
            GetScene.Request(img=self._img))
        if scene_response is None:
            return None

        # The server resolves poses in the robot base frame at the image's own
        # stamp, so there is nothing left to transform here. In particular do NOT
        # route these through self.transform(): that is the servoing helper, and
        # it clamps z to the home EE height and flattens orientation.
        return [
            SceneObject.from_dict(name, {
                "position": [
                    posestamped.pose.position.x,
                    posestamped.pose.position.y,
                    posestamped.pose.position.z,
                ],
                "orientation": [
                    posestamped.pose.orientation.x,
                    posestamped.pose.orientation.y,
                    posestamped.pose.orientation.z,
                    posestamped.pose.orientation.w,
                ],
                "params": "",
            })
            for name, posestamped in zip(scene_response.names, scene_response.pose)
        ]

    def publish_scene_thread(self):
        from scene_getter.scene_lib.scene import Scene
        from scene_getter.scene_lib.scene_object import SceneObject

        # Remembered so a sustained hold is logged once, not once a cycle.
        reported = "startup"
        # Kept outside the loop so a failure to read the parameter cannot turn
        # this into a hot loop -- the previous period still applies.
        period = 1.0

        while rclpy.ok():
            time.sleep(period)

            # Everything below is inside try/except because this runs in a bare
            # thread: an unhandled exception would kill it silently and publishing
            # would simply stop with nothing in the log. That includes the
            # subscriber count, which raises once the node is destroyed.
            try:
                period = 1.0 / max(float(self.get_parameter("scene_rate").value), 0.01)

                # Somebody listening on /scene *is* the request to publish:
                # opening the gestures dashboard starts the scene and closing it
                # stops the SIFT work. The explicit flag still forces it on for
                # lfd.py, which wants poses before an action without subscribing.
                if not (self.publishing_scene or self.scene_pub.get_subscription_count() > 0):
                    continue

                holding = self._untrustworthy_view()
                if holding is None:
                    recognised = self._recognise_objects(SceneObject)
                    if recognised is None:
                        holding = "compute_object_positions returned nothing"
                    else:
                        # Replace wholesale rather than merge: at home, a template
                        # that no longer matches is genuinely gone.
                        self._scene_objects = recognised
                        self._scene_observed_at = self._img.header.stamp

                if holding != reported:
                    if holding is None:
                        self.get_logger().info("[scene] observing")
                    else:
                        self.get_logger().warning(
                            f"[scene] {holding}: holding the last seen poses, stamped "
                            f"when they were seen so consumers can grey them out")
                    reported = holding

                if self._scene_observed_at is None:
                    # Nothing has ever been recognised, so there is no scene to
                    # describe and no honest stamp to put on one. Note this is
                    # not the same as an *empty* scene: once something has been
                    # looked at, "I see nothing" is a real answer and has to go
                    # out, or an object that was picked up would linger on the
                    # dashboard for the rest of the session.
                    continue

                scene = Scene(name=SCENE_NAME, objects=self._scene_objects)
                message = scene.to_ros()
                # Stamped when the poses were *observed*, not when republished, so
                # age tells a consumer whether to trust them. This is the only
                # staleness signal on the wire: SceneObject.params is free-form
                # description text that get_params() feeds downstream, so it must
                # not be overloaded with a status flag.
                message.header.stamp = self._scene_observed_at
                message.header.frame_id = SCENE_FRAME_ID
                self.scene_pub.publish(message)
            except Exception as error:  # noqa: BLE001 -- keep the thread alive
                self.get_logger().error(f"[scene] publish failed: {error}")

    def curr_pose_callback(self, msg):
        self.curr_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.curr_ori_wxyz = [msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z]

    def image_callback(self, img):
        # self._prev_img = deepcopy(self._img)
        self.img_last_rec = time.time()
        self._img = img
        
    def start_publishing_scene(self, req, res):
        self.publishing_scene = True
        res.success = True  # Trigger.Response defaults to False; callers check it
        return res

    def stop_publishing_scene(self, req, res):
        self.publishing_scene = False
        res.success = True
        return res

    def handle_request(self, req, res):
        print("Active localization started", flush=True)
        self._rate.sleep()
        self.timeout_counter = 0
        while rclpy.ok():
            if self._img is None: # or self._prev_img is None or list(self._img.data) == list(self._prev_img.data):
                self.get_logger().warning("No Image")
                self._rate.sleep()
                continue
            if (time.time() - self.img_last_rec) > 1.0:
                self.get_logger().warning("Img is not fresh")
                self._rate.sleep()
                continue
            position = self.curr_pos
            ori = list_2_quaternion(self.curr_ori_wxyz)
            home_pose = pos_quat_2_pose_st(position, ori)
            
            try:
                resp = self.compute_box_tf.call(request=ComputeLocalization.Request(img=self._img))
                if not resp.success:
                    # A single miss can be a blurred frame mid-approach, so spend
                    # the same budget the convergence loop gets before giving up.
                    self.timeout_counter += 1
                    if self.timeout_counter >= self.timeout_counter_max:
                        res.success = False
                        res.message = "object not detected"
                        self.get_logger().error(
                            "Localization aborted: object not detected")
                        return res
                    self._rate.sleep()
                    continue
                box_tf = resp.pose
                ori = [
                    resp.pose.pose.orientation.x,
                    resp.pose.pose.orientation.y,
                    resp.pose.pose.orientation.z,
                    resp.pose.pose.orientation.w
                ]
                xy_yaw = [
                    resp.pose.pose.position.x, 
                    resp.pose.pose.position.y,
                    euler_from_quaternion(ori)[2]
                ]
            except Exception as e:
                print(e, flush=True)
                continue

            self._transformed_pose = self.transform(box_tf, home_pose)
 
            assert self._transformed_pose.pose.position.z > 0.1
            self.goal_pose_pub.publish(self._transformed_pose)
            self._rate.sleep()
            pos_error = np.linalg.norm(xy_yaw[:2])
            yaw_error = abs(xy_yaw[2])
            time.sleep(pos_error * 2) # Note: waits for the move to end, but not guaranteed
            time.sleep(0.1) # Note: waits for the move to end, but not guaranteed
            print("", flush=True)
            print("Localization step : ", self.timeout_counter, flush=True)
            print(f"position error {pos_error}, yaw error {yaw_error}", flush=True)
            print("", flush=True)
            if (pos_error < self.position_accuracy and yaw_error < self.orientation_accuracy) or self.timeout_counter >= self.timeout_counter_max:
                print(f"Localization finished! final error: {pos_error + yaw_error}", flush=True)
                res.success = True
                res.message = f"final error: {pos_error + yaw_error}"
                return res
            self.timeout_counter = self.timeout_counter + 1

    def get_transform_camera(self):
        while True:
            try:
                translation1, rotation1 = self.lookup_relative_transform("panda_link0", "panda_hand")
                translation2, rotation2 = self.lookup_relative_transform("panda_hand", "camera_color_optical_frame")
                
                rp_tr1 = [translation1.x, translation1.y, translation1.z]
                rp_rt1 = [rotation1.x, rotation1.y, rotation1.z, rotation1.w]
                rp_tr2 = [translation2.x, translation2.y, translation2.z]
                rp_rt2 = [rotation2.x, rotation2.y, rotation2.z, rotation2.w]
                
                transform = np.dot(
                    tf_transformations.translation_matrix(rp_tr1),
                    tf_transformations.quaternion_matrix(rp_rt1),
                )
                transform = np.dot(
                    transform,
                    tf_transformations.translation_matrix(rp_tr2),
                )
                transform = np.dot(
                    transform,
                    tf_transformations.quaternion_matrix(rp_rt2),
                )
                return transform
                
            except Exception as e:
                time.sleep(0.3)
                print(f"Transform lookup failed: {e}. Retrying...", flush=True)

        
    
    def transform(self, transformation_pose, pose, check_z_axis=True):
        transform_base_2_cam = self.get_transform_camera()

        # if transform box is not in camera frame, remove the base_2_cam transforms
        transform_box = pose_st_2_transformation(transformation_pose)
        transform = transform_base_2_cam @ transform_box @ np.linalg.inv(transform_base_2_cam)

        pose = transform_pose(pose, transform)
        pose_quat = orientation_2_quaternion(pose.pose.orientation)

        # Maintain orientation and only apply 'yaw' (rotation around EE z-axis)
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0
        new_magnitude = np.sqrt(pose_quat.x * pose_quat.x + pose_quat.y * pose_quat.y)
        pose_quat.x = pose_quat.x / new_magnitude
        pose_quat.y = pose_quat.y / new_magnitude
        pose.pose.orientation.x = pose_quat.x
        pose.pose.orientation.y = pose_quat.y

        home_EE_height = get_remote_parameters(self, ["position_z"], server="localizer_node")[0]
        if check_z_axis:
            assert home_EE_height > 0.1, f"Your template z-axis coords is below safety limit: {home_EE_height} < 0.1"
        pose.pose.position.z=home_EE_height  # Maintain same height
        return pose

def main():
    rclpy.init()
    rosnode = ActiveLocalizerNode()
    while rclpy.ok():
        time.sleep(1.0)

if __name__ == '__main__':
    main()
