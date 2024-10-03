#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import os
import argparse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

class CameraHandler():
    def __init__(self, save_rate, save_dir, manual_save_dir):
        self.bridge = CvBridge()
        self.cv_image = np.zeros((640, 480))
        rospy.init_node('CAMnod', anonymous=True)
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.callback)
        self.last_saved_time = time.time()
        self.save_rate = save_rate
        self.save_dir = save_dir
        self.manual_save_dir = manual_save_dir
        self.ensure_dir_exists(save_dir)
        self.ensure_dir_exists(manual_save_dir)
        rate = rospy.Rate(save_rate)
        rospy.spin()
        rate.sleep()
    

    def ensure_dir_exists(self, directory):
        if not os.path.exists(directory):
            os.makedirs(directory)

    def save_frame(self, directory):
        filename = f"{directory}/frame_{int(time.time())}.png"
        cv2.imwrite(filename, self.cv_image)
        print(f"Frame saved at {filename}")

    def callback(self, data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.imshow("Frame preview", self.cv_image)
        key = cv2.waitKey(1)

        # Save frame at regular intervals
        self.save_frame(self.save_dir)

        # Manual save when 's' is pressed
        if key == ord('s'):
            self.save_frame(self.manual_save_dir)
            
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Camera frame saver')
    parser.add_argument('--rate', type=int, default=5, help='Frame save rate in seconds')
    parser.add_argument('--dir', type=str, default='saved_frames', help='Directory for automatic saves')
    parser.add_argument('--mdir', type=str, default='manual_frames', help='Directory for manual saves')
    args = parser.parse_args()

    try:
        nod = CameraHandler(args.rate, args.dir, args.mdir)
    except rospy.ROSInterruptException:
        pass