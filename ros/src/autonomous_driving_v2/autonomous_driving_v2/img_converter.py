import rclpy
import cv2
import rclpy
import cv2
import matplotlib.pylab as plt
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import Subscriber
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float64


class ImgConverter:
    # "bridge" between OpenCV and ROS Image
    bridge = CvBridge()

    @staticmethod
    def get_cv(msg_in):
        # ROS image => OpenCV image (grey scaled)
        return ImgConverter.bridge.imgmsg_to_cv2(msg_in, desired_encoding='8UC3')

    @staticmethod
    def get_bw(cv_image):
        gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    
        threshold = 45  # at what grey level  a pixel should become white
        white = 255
        # return is a tuple => first value ignored
        (thresh, bw_img) = cv2.threshold(gray_img, threshold, white, cv2.THRESH_BINARY)
        return bw_img

    @staticmethod
    def get_ol(cv_image, middle_lane_left, middle_lane_right):
        gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        all_lines_mask = cv2.inRange(gray_img, 70, 255, cv2.THRESH_BINARY)
        all_lines_mask = cv2.dilate(all_lines_mask, np.ones((5,5),np.uint8), iterations=1)
        all_lines_mask = cv2.bitwise_not(all_lines_mask)
        outer_lines_mask = cv2.rectangle(all_lines_mask, (middle_lane_left, 0), (middle_lane_right, 210), (255,255,255), -1)

        threshold = 45  # at what grey level  a pixel should become white
        white = 255
        # return is a tuple => first value ignored
        (thresh, bw_img) = cv2.threshold(gray_img, threshold, white, cv2.THRESH_BINARY)

        outer_lines_img = cv2.bitwise_not(outer_lines_mask, bw_img)

        # plt.imshow(outer_lines_img)
        # plt.show()

        return outer_lines_img

    @staticmethod
    def get_crop(cv_image):
        # crop sky
        new_height = 275  # by testing with plt.show()
        # [rows, columns]
        return cv_image[new_height:cv_image.shape[0], 0:cv_image.shape[1]]

    @staticmethod
    def get_ros_img(cv_image):
        return ImgConverter.bridge.cv2_to_imgmsg(cv_image, encoding='rgb8')