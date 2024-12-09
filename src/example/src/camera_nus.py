#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class CameraHandler():
    def __init__(self):
        """
        Creates a bridge for converting the images from Gazebo image to OpenCV format.
        """
        self.bridge = CvBridge()
        
        # Dictionary to hold the latest images from each camera
        self.images = {
            "front": None,
            "front_left": None,
            "front_right": None,
            "back": None,
            "back_left": None,
            "back_right": None
        }
        
        # Initialize ROS node
        rospy.init_node('CAMnod', anonymous=True)
        
        # Subscribe to each camera topic
        self.subscribers = {
            "front": rospy.Subscriber("/cam_front/raw", Image, self.callback, "front"),
            "front_left": rospy.Subscriber("/cam_front_left/raw", Image, self.callback, "front_left"),
            "front_right": rospy.Subscriber("/cam_front_right/raw", Image, self.callback, "front_right"),
            "back": rospy.Subscriber("/cam_back/raw", Image, self.callback, "back"),
            "back_left": rospy.Subscriber("/cam_back_left/raw", Image, self.callback, "back_left"),
            "back_right": rospy.Subscriber("/cam_back_right/raw", Image, self.callback, "back_right")
        }
        
        rospy.spin()

    def callback(self, data, cam_name):
        """
        Callback function for each camera topic. Updates the image for the given camera.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.images[cam_name] = cv_image
            
            # Display the images in two rows
            self.display_images()
        
        except CvBridgeError as e:
            rospy.logerr(f"Error converting image for {cam_name}: {e}")
    
    def display_images(self):
        """
        Display the six camera images in a 2x3 grid using OpenCV.
        """
        # Check if we have all images using any() function
        if any(img is None for img in self.images.values()):
            return
        
        # Resize all images to be the same size for concatenation
        resized_images = {key: cv2.resize(img, (400, 300)) for key, img in self.images.items()}
        
        # Concatenate the images into a 2x3 grid
        top_row = np.hstack((resized_images["front_left"], resized_images["front"], resized_images["front_right"]))
        bottom_row = np.hstack((resized_images["back_left"], resized_images["back"], resized_images["back_right"]))
        grid = np.vstack((top_row, bottom_row))
        
        # Display the grid
        cv2.imshow("Camera Grid", grid)
        key = cv2.waitKey(1)
    
if __name__ == '__main__':
    try:
        CameraHandler()
    except rospy.ROSInterruptException:
        pass
