#!/bin/python3

import cv2
import rclpy
import os
import yaml
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from panda_control import SpinPandaNode
import object_localization
import time
CAMERA_COLOR_TOPIC = "/camera/color/image_raw"
CAMERA_DEPTH_TOPIC = "/camera/depth/image_rect_raw"

class Template():
    def __init__(self):
        self.panda=SpinPandaNode()
        self.params = dict()

    def crop_image(self, event, x, y, flags, param):
        global mouseX, mouseY, cropping
        if event == cv2.EVENT_LBUTTONDOWN:
            mouseX, mouseY = x, y
            cropping = True
        elif event == cv2.EVENT_LBUTTONUP:
            mouseX2, mouseY2 = x, y
            cropping = False
            cv2.imshow("image", self.image)
            y_low = min(mouseY, mouseY2)
            y_high = max(mouseY, mouseY2)
            x_low = min(mouseX, mouseX2)
            x_high = max(mouseX, mouseX2)
            self.params['crop'] = [x_low, x_high, y_low, y_high]
            print(f'{x_low}, {x_high}, {y_low}, {y_high}')

            cropped_image = self.image[y_low: y_high, x_low:x_high]
            cv2.imshow("cropped", cropped_image)
            cv2.imwrite(f"{self.save_dir}/template.png", cropped_image)
    
    def img_rec_clb(self, msg):
        self.img_message_received = msg
    def img_rec_clb_depth(self, msg):
        self.img_message_received_depth = msg


    def record(self, name='template_test'):
        self.save_dir = object_localization.package_path +f"/cfg/{name}"
        self.params['template_path'] = f"/cfg/{name}"+"/full_image.png"
        try:
            os.mkdir(self.save_dir)
        except FileExistsError:
            pass
        depth=None
        bridge = CvBridge()

        self.img_message_received = None
        self.panda.create_subscription(Image, CAMERA_COLOR_TOPIC, self.img_rec_clb, 5)
        time.sleep(5.0) # wait while camera brightness stabilizes
        while self.img_message_received is None:
            rclpy.spin_once(self.panda, timeout_sec=10.0)
            print("waiting for message", flush=True)
        msg = self.img_message_received
        # msg = rospy.wait_for_message(, timeout=10)
        self.image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite(f"{self.save_dir}/full_image.png", self.image)
    
        bridge = CvBridge()

        self.img_message_received_depth = None
        self.panda.create_subscription(Image, CAMERA_DEPTH_TOPIC, self.img_rec_clb_depth, 5)
        while self.img_message_received_depth is None:
            rclpy.spin_once(self.panda, timeout_sec=10.0)
            print("waiting for message", flush=True)
        msg_depth = self.img_message_received_depth
        # msg_depth = rospy.wait_for_message('camera/depth/image_rect_raw', Image, timeout=10)
        depth = bridge.imgmsg_to_cv2(msg_depth, desired_encoding="passthrough")
        cv2.imwrite(f"{self.save_dir}/depth.png", depth)
        
        print("Click and drag to select template")
        print("Press 'q' to quit")
        # Create window and set mouse callback function
        cv2.namedWindow("image")
        cv2.setMouseCallback("image", self.crop_image)

        # Loop until user presses 'q'
        cropping = False
        while True:
            cv2.imshow(f"image", self.image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
        depth_row=depth[self.params['crop'][2]:self.params['crop'][3], self.params['crop'][0]:self.params['crop'][1]].reshape(-1)
        self.params['depth'] = float(np.median(depth_row))

        print(self.panda.curr_pos[0])
        self.params['position']={'x': float(self.panda.curr_pos[0]), 'y': float(self.panda.curr_pos[1]), 'z': float(self.panda.curr_pos[2])}
        self.params['orientation']={'w': float(self.panda.curr_ori_wxyz[0]) ,'x': float(self.panda.curr_ori_wxyz[1]) , 'y': float(self.panda.curr_ori_wxyz[2]), 'z': float(self.panda.curr_ori_wxyz[3])}
        with open(f"{self.save_dir}/params.yaml", 'w') as file:
            yaml.dump(self.params, file)


