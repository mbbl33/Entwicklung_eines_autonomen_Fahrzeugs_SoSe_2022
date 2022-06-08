import rclpy
import cv2
import rclpy
import cv2
import matplotlib.pylab as plt
import numpy as np

from .region_of_interest import Region_of_Interest
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import Subscriber
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge



class LaneDetection(Node):

    def __init__(self):
        super().__init__('image_converter')

        # "bridge" between OpenCV and ROS Image
        self.bridge = CvBridge()

        self.left_RoI = Region_of_Interest(150, 275, 50, 300, 75, 5)
        self.right_RoI = Region_of_Interest(490, 360, 50, 300, 75, 5)

        # /camera/image_raw [sensor_msgs/msg/Image]
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.pub_lane_img, 1)

        # raw image with line on it
        self.pub_lane_img = self.create_publisher(Image, 'lane_image', 1)

        self.pub_roi_left = self.create_publisher(Image, 'left_roi', 1)
        self.pub_roi_rigth = self.create_publisher(Image, 'right_roi', 1)

    # image conveter
    def get_CV(self, msg_in):
        # ROS image => OpenCV image (grey scaled)
        return self.bridge.imgmsg_to_cv2(msg_in, desired_encoding='8UC3')

    def get_bw(self, cv_image):
        gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        threshold = 45  # at what grey level  a pixel should become white
        white = 255
        # return is a tuple => first value ignored
        (thresh, bw_img) = cv2.threshold(gray_img, threshold, white, cv2.THRESH_BINARY)
        return bw_img

    def get_crop(self, cv_image):
        # crop sky
        new_height = 275  # by testing with plt.show()
        # [rows, columns]
        return cv_image[new_height:cv_image.shape[0], 0:cv_image.shape[1]]

    # lane detection
    def get_edges(self, cv_img):
        blur = cv2.GaussianBlur(cv_img, (5, 5), 0)
        edges = cv2.Canny(blur, 20, 80)
        return edges

    def get_lines(self, edge_img, roi, ):
        roi_img = roi.get_RoI(edge_img)

        while True:
            out = cv2.HoughLinesP(roi_img, 1, np.pi / 180, 15, np.array([]), minLineLength=30,
                                  maxLineGap=20)  # cv2.HoughLinesP(roi_img, 2, np.pi / 180, 60, np.array([]), minLineLength=10, maxLineGap=40)
            print("Out ", out)
            if out is None and roi.current_w < roi.max_width:
                roi.increase_roi()
                print("aktuelle roi %d von %d ", roi.current_w, roi.lower_x)
            else:
                break
        # roi.reset_roi()
        return out

    # pup lines on raw image
    def draw_line_img(self, cv_img, line_vectors, color):
        img = np.copy(cv_img)
        line_img = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
        if line_vectors is not None:
            for vector in line_vectors:
                for x1, y1, x2, y2 in vector:
                    cv2.line(line_img, (x1, y1), (x2, y2), color, 3)
        # place lines on original image
        return cv2.addWeighted(img, 0.8, line_img, 1, 1)

    def pub_lane_img(self, msg_in):

        img_raw = self.get_CV(msg_in)
        img_croped = self.get_crop(img_raw)

        img_bw = self.get_bw(img_croped)

        img_edge = self.get_edges(img_bw)

        # img_roi_l = self.left_RoI.get_RoI(img_edge)
        # img_roi_r = self.right_RoI.get_RoI(img_edge)
        vectors_l = self.get_lines(img_edge, self.left_RoI)
        vectors_r = self.get_lines(img_edge, self.right_RoI)
        mid_of_lines = self.get_mid_x(vectors_l,vectors_r)
        print("Left\t", vectors_l)
        print("Right\t", vectors_r)
        print("X \t", mid_of_lines)

        plt.imshow(img_edge)
        plt.show()
        img_cv_right = self.get_line_img(img_croped, vectors_r, (0, 255, 0))
        img_cv_mid = self.draw_mid_of_lines(img_cv_right, mid_of_lines)
        img_cv_all_lines = self.get_line_img(img_cv_mid, vectors_l, (255, 0, 0))

        # img_cv_mid = self.get_mid(img_cv_all_lines, vectors_l, vectors_r)
        msgOut = self.bridge.cv2_to_imgmsg(img_cv_all_lines, encoding='rgb8')
        self.pub_lane_img.publish(msgOut)

        # pub rois on raw for debug
        outL = self.left_RoI.get_RoI(img_croped)
        outR = self.right_RoI.get_RoI(img_croped)
        msgOut = self.bridge.cv2_to_imgmsg(outL, encoding='rgb8')
        self.pub_roi_left.publish(msgOut)
        msgOut = self.bridge.cv2_to_imgmsg(outR, encoding='rgb8')
        self.pub_roi_rigth.publish(msgOut)
        self.left_RoI.reset_roi()
        self.right_RoI.reset_roi()

    def get_mid_x(self, l_vectors, r_vectors):
        if l_vectors is not None:
            for vector in l_vectors:
                for x1, y1, x2, y2 in vector:
                    self.lowestXL = x1 if y1 < y2 else x2

        if r_vectors is not None:
            for vector in r_vectors:
                for x1, y1, x2, y2 in vector:
                    self.lowestXR = x1 if y1 < y2 else x2

        return (self.lowestXL + self.lowestXR) / 2

    def draw_mid_of_lines(self, cv_img, x):
        height = cv_img.shape[0]
        img = np.copy(cv_img)
        line_img = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
        x = int(x)

        cv2.line(line_img, (x, height), (x, 0), (255, 0, 255), 3)

        # place lines on original image
        return cv2.addWeighted(img, 0.8, line_img, 1, 1)




def main(args=None):
    rclpy.init(args=args)
    lane_detection = LaneDetection()
    rclpy.spin(lane_detection)
    lane_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
