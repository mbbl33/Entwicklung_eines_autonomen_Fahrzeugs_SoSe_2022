import rclpy
import cv2
import rclpy
import cv2
import matplotlib.pylab as plt
import numpy as np
import math

from .img_converter import ImgConverter
from .pid import PID_Controller
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import Subscriber
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float64, Int64MultiArray
from std_msgs.msg import Bool


class LaneBasedSteer(Node):

    def __init__(self):
        super().__init__('lane_based_steer')
        self.debug = True
        self.blocked = False
        self.car_pos = 320  # static car position
        self.top_line = 125  # pixel height for upper lane scan
        self.bottom_line = 195  # pixel height for lower lane scan
        self.indexes_top = (144, 377)  # (left, right) indexes for found lanes (upper lane scan)
        self.indexes_bottom = (144, 377)  # (left, right) indexes for found lanes (lower lane scan)
        self.target_point_top = 0  # driveway point (upper lane scan)
        self.target_point_bottom = 0  # driveway point (lower lane scan)

        self.pid_front = PID_Controller(1.5, 0.001, 0.5, 45, -45, (1 / 5))
        self.pid_back = PID_Controller(1.5, 0.001, 0.5, 45, -45, (1 / 5))

        self.ol_max_points = (100, 425)  # for lane change - left and right max points for lane scan
        self.ol_middle_lane_points = (200, 300)  # for lane change - left and right points for middle lane removal
        self.driveway_factor = 0.75

        # /camera/image_raw [sensor_msgs/msg/Image]
        self.sub_camera = self.create_subscription(Image, '/camera/image_raw', self.steer, 1)

        # for overtaker and parking
        self.sub_block_lbs = self.create_subscription(Bool, 'block_lane_based_steer', self.update_blocked, 1)
     
        # raw image with lines on it for debug
        self.pub_lane_img = self.create_publisher(Image, 'lane_image', 1)

        # pub RoI for debug
        self.pub_roi_left = self.create_publisher(Image, 'left_roi', 1)
        self.pub_roi_right = self.create_publisher(Image, 'right_roi', 1)

        # simple steering
        self.pub_steering = self.create_publisher(Float64, '/steering', 1)

        self.sub_lane_data = self.create_subscription(Int64MultiArray, 'lane_data', self.update_lane_data, 1)
        self.sub_driveway_factor = self.create_subscription(Float64, 'driveway_factor', self.update_driveway_factor, 1)

        self.pub_parking_box_detect = self.create_publisher(Bool, 'early_box_detection', 1)

    def update_blocked(self, msg_in):
        self.blocked = msg_in.data

    def update_lane_data(self, msg_in):
        self.ol_max_points = (msg_in.data[0], msg_in.data[1])
        self.ol_middle_lane_points = (msg_in.data[2], msg_in.data[3])
        self.indexes_top = (msg_in.data[4], msg_in.data[5])
        self.indexes_bottom = (msg_in.data[4], msg_in.data[5])

    def update_driveway_factor(self, msg_in):
        self.driveway_factor = msg_in.data

    def update_parking_early_detection(self, cv_img):
        msg_out = Bool()
        msg_out.data = bool(cv_img[10][500] != 0)
        self.pub_parking_box_detect.publish(msg_out)

    def find_lanes(self, img_ol, height, left_max, right_max):
        # finding the left line
        index_l = 295
        while img_ol[height,index_l] == 0 and index_l > left_max:
            index_l -= 2
        # finding the right line
        index_r = 345
        while img_ol[height,index_r] == 0 and index_r < right_max:
            index_r += 2

        # filter out segments without lines
        if height == self.top_line:
            dif_l = abs(index_l - self.indexes_top[0])
            dif_r = abs(index_r - self.indexes_top[1])
            if index_l <= left_max or dif_l > 30:
                index_l = self.indexes_top[0]
            if index_r >= right_max or dif_r > 30:
                index_r = self.indexes_top[1]
        else:
            dif_l = abs(index_l - self.indexes_bottom[0])
            dif_r = abs(index_r - self.indexes_bottom[1])
            if index_l <= left_max or dif_l > 30:
                index_l = self.indexes_bottom[0]
            if index_r >= right_max or dif_r > 30:
                index_r = self.indexes_bottom[1]

        # filter out segments with horizontal lines
        if height == self.top_line:
            if img_ol[height,index_l - 23] != 0:
                index_l = self.indexes_top[0]
            elif img_ol[height,index_r + 23] != 0:
                index_r = self.indexes_top[1]
        else:
            if img_ol[height,index_l - 23] != 0:
                index_l = self.indexes_bottom[0]
            elif img_ol[height,index_r + 23] != 0:
                index_r = self.indexes_bottom[1]

        return (index_l, index_r)

    def calc_driveway_points(self):
        self.target_point_top = int((self.indexes_top[1] - self.indexes_top[0]) * self.driveway_factor + self.indexes_top[0])
        self.target_point_bottom = int((self.indexes_bottom[1] - self.indexes_bottom[0]) * self.driveway_factor + self.indexes_bottom[0])

    def analyze(self, msg_in):

        img_raw = ImgConverter.get_cv(msg_in)
        img_croped = ImgConverter.get_crop(img_raw)
        img_ol = ImgConverter.get_ol(img_croped, *(self.ol_middle_lane_points))

        self.indexes_top = self.find_lanes(img_ol, self.top_line, *(self.ol_max_points))
        self.indexes_bottom = self.find_lanes(img_ol, self.bottom_line, *(self.ol_max_points))

        self.update_parking_early_detection(img_ol)

        self.calc_driveway_points()

        # plt.imshow(img_croped)
        # plt.show()

        if (self.debug):
            # img_cv_all_lines = self.debug_draw_lines(img_croped, vectors_r, vectors_l, mid_of_lines, drive_way)
            img_cv_all_lines = self.debug_draw_circles(img_croped, self.top_line, self.bottom_line)
            msg_out = ImgConverter.get_ros_img(img_cv_all_lines)
            self.pub_lane_img.publish(msg_out)

    def steer(self, msg_in):
        self.analyze(msg_in)
        if self.blocked:
            return
        front_correct = self.pid_front.calc_pid(self.car_pos, self.target_point_top, self.indexes_top[1], self.indexes_top[0])
        back_correct = self.pid_back.calc_pid(self.car_pos, self.target_point_bottom, self.indexes_bottom[1], self.indexes_bottom[0])
        out = (front_correct + back_correct) / 2
        steer = Float64()
        steer.data = float(out)
        self.pub_steering.publish(steer)

    ######### for Debug

    def debug_draw_circles(self, img, top, bottom):

        out_img = cv2.circle(img, (self.indexes_top[0], top), 5, (255,0,0), -1)
        out_img = cv2.circle(out_img, (self.indexes_top[1], top), 5, (255,0,0), -1)
        out_img = cv2.circle(img, (self.indexes_bottom[0], bottom), 5, (255,0,0), -1)
        out_img = cv2.circle(out_img, (self.indexes_bottom[1], bottom), 5, (255,0,0), -1)

        out_img = cv2.circle(out_img, (self.target_point_top, top), 5, (0,255,0), -1)
        out_img = cv2.circle(out_img, (self.target_point_bottom, bottom), 5, (0,255,0), -1)

        out_img = cv2.line(out_img, (self.target_point_bottom, bottom), (self.target_point_top, top), (0,0,255), 3)

        out_img = cv2.line(out_img, (self.ol_max_points[0], 0), (self.ol_max_points[0], 205), (255,255,0), 3)
        out_img = cv2.line(out_img, (self.ol_max_points[1], 0), (self.ol_max_points[1], 205), (255,255,0), 3)

        return out_img

def main(args=None):
    rclpy.init(args=args)
    lane_detection = LaneBasedSteer()
    rclpy.spin(lane_detection)
    lane_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()