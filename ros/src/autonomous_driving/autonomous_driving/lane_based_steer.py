import rclpy
import cv2
import rclpy
import cv2
import matplotlib.pylab as plt
import numpy as np

from .img_converter import ImgConverter
from .region_of_interest import RegionOfInterest
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
        self.driveway_factor = 1.25
        self.x_left_lane = 140
        self.x_right_lane = 380
        self.blocked = False
        max_roi = 100
        min_roi = 40
        self.left_RoI = self.set_roi(self.x_left_lane, self.x_left_lane, max_roi, min_roi)
        self.right_RoI = self.set_roi(self.x_right_lane, self.x_right_lane, max_roi, min_roi)

        # /camera/image_raw [sensor_msgs/msg/Image]
        self.sub_camera = self.create_subscription(Image, '/camera/image_raw', self.steer, 1)

        # for overtaker and parking
        self.sub_block_lbs = self.create_subscription(Bool, 'block_lane_based_steer', self.update_blocked, 1)

        # for overtaker
        self.sub_adjust_roi = self.create_subscription(Int64MultiArray, 'adjust_region_of_interest', self.update_roi, 1)
        self.sub_adjust_driveway = self.create_subscription(Float64, 'adjust_driveway_factor', self.update_driveway_factor,
                                                        1)

        # raw image with lines on it for debug
        self.pub_lane_img = self.create_publisher(Image, 'lane_image', 1)

        # pub RoI for debug
        self.pub_roi_left = self.create_publisher(Image, 'left_roi', 1)
        self.pub_roi_right = self.create_publisher(Image, 'right_roi', 1)

        # simple steering
        self.pub_steering = self.create_publisher(
            Float64, '/steering', 1)

    def set_roi(self, x_lower, x_upper, max_width, min_width):
        return RegionOfInterest(x_lower, 0, x_upper, 0, max_width, min_width, 5)

    def update_blocked(self, msg_in):
        self.blocked = msg_in.data

    def update_roi(self, msg_in):
        self.left_RoI = self.set_roi(msg_in.data[0], msg_in.data[0], msg_in.data[2], msg_in.data[3])
        self.right_RoI = self.set_roi(msg_in.data[1], msg_in.data[1], msg_in.data[2], msg_in.data[3])

    def update_driveway_factor(self, factor_msg):
        self.driveway_factor = factor_msg.data

    def get_edges(self, cv_img):
        img_bw = ImgConverter.get_bw(cv_img)
        edges = cv2.Canny(img_bw, 10, 40)
        return edges

    def get_lines(self, edge_img, roi):
        roi_img = roi.get_roi(edge_img)

        while True:
            out = cv2.HoughLinesP(roi_img,
                                  1,
                                  np.pi / 180,
                                  15,
                                  np.array([]),
                                  minLineLength=15,
                                  maxLineGap=50)
            if out is None and roi.is_not_max():
                roi.increase_roi()
            else:
                if out is not None:
                    roi.reset_roi()
                break
        return out

    def analyze(self, msg_in):

        img_raw = ImgConverter.get_cv(msg_in)
        img_croped = ImgConverter.get_crop(img_raw)
        img_edge = self.get_edges(img_croped)
        vectors_l = self.get_lines(img_edge, self.left_RoI)
        vectors_r = self.get_lines(img_edge, self.right_RoI)
        mid_of_lines = self.get_mid_x(vectors_l, vectors_r)
        drive_way = mid_of_lines * self.driveway_factor
        # plt.imshow(img_croped)
        # plt.show()

        if (self.debug):
            img_cv_all_lines = self.debug_draw_lines(img_croped, vectors_r, vectors_l, mid_of_lines, drive_way)
            msg_out = ImgConverter.get_ros_img(img_cv_all_lines)
            self.pub_lane_img.publish(msg_out)
            self.debug_rois(img_raw)

        return (img_croped, drive_way)

    def steer(self, msg_in):
        analyzed = self.analyze(msg_in)
        if self.blocked:
            return
        img = analyzed[0]
        drive_way = analyzed[1]
        steer = Float64()
        mid = img.shape[1] / 2
        dif = drive_way - mid
        dif = min(dif, 45)
        out = max(dif, -45)
        steer.data = float(out)
        self.pub_steering.publish(steer)

    def get_mid_x(self, l_vectors, r_vectors):
        self.x_left_lane = self.calc_average_x(l_vectors, self.x_left_lane)
        self.x_right_lane = self.calc_average_x(r_vectors, self.x_right_lane)

        return (self.x_left_lane + self.x_right_lane) / 2

    def calc_average_x(self, vectors, old_value):
        average_x = 0
        if vectors is not None:
            for vector in vectors:
                for x1, y1, x2, y2 in vector:
                    average_x += x1 + x2
            out = average_x / (len(vectors) * 2)
        else:
            out = old_value
        return out

    ######### for Debug

    # pub rois on img for debug
    def debug_rois(self, img):
        outL = self.left_RoI.get_roi(img)
        outR = self.right_RoI.get_roi(img)
        msgOut = ImgConverter.get_ros_img(outL)
        self.pub_roi_left.publish(msgOut)
        msgOut = ImgConverter.get_ros_img(outR)
        self.pub_roi_right.publish(msgOut)

    # pup lines on image
    def debug_draw_lines(self, img, vectors_r, vectors_l, mid_of_lines, drive_way):
        img_cv_right = self.draw_lane_img(img, vectors_r, (0, 255, 0))
        img_cv_mid_of_lanes = self.draw_veritcal_line(img_cv_right, mid_of_lines, (255, 0, 255))
        img_cv_mid_of_img = self.draw_veritcal_line(img_cv_mid_of_lanes, img.shape[1] // 2, (255, 255, 255))
        img_cv_driveway = self.draw_veritcal_line(img_cv_mid_of_img, drive_way, (0, 0, 255))
        return self.draw_lane_img(img_cv_driveway, vectors_l, (255, 0, 0))

    def draw_lane_img(self, cv_img, line_vectors, color):
        img = np.copy(cv_img)
        line_img = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
        if line_vectors is not None:
            for vector in line_vectors:
                for x1, y1, x2, y2 in vector:
                    cv2.line(line_img, (x1, y1), (x2, y2), color, 3)
        # place lines on original image
        return cv2.addWeighted(img, 0.8, line_img, 1, 1)

    def draw_veritcal_line(self, cv_img, x, color):
        height = cv_img.shape[0]
        img = np.copy(cv_img)
        line_img = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
        x = int(x)
        cv2.line(line_img, (x, height), (x, 0), color, 3)
        return cv2.addWeighted(img, 0.8, line_img, 1, 1)


def main(args=None):
    rclpy.init(args=args)
    lane_detection = LaneBasedSteer()
    rclpy.spin(lane_detection)
    lane_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
