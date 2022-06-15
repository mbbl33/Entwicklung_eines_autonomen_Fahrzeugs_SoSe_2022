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
    def get_crop(cv_image):
        # crop sky
        new_height = 275  # by testing with plt.show()
        # [rows, columns]
        return cv_image[new_height:cv_image.shape[0], 0:cv_image.shape[1]]

    @staticmethod
    def get_ros_img(cv_image):
        return ImgConverter.bridge.cv2_to_imgmsg(cv_image, encoding='rgb8')
