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


class Lane_Detection(Node):

    def __init__(self):
        super().__init__('image_converter')

        # "bridge" between OpenCV and ROS Image
        self.bridge = CvBridge()

        self.left_RoI = Region_of_Interest(150, 300, 0, 300, 100, 20)
        self.right_RoI = Region_of_Interest(490, 340, 0, 300, 100, 20)
        # /camera/image_raw [sensor_msgs/msg/Image]
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.pub_lane_img, 1)

        # raw image with line on it
        self.publisher = self.create_publisher(Image, 'lane_image', 1)

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

    def get_downscale(self, cv_image):
        scale = 2
        rows, cols = map(int, cv_image.shape)
        return cv2.pyrDown(cv_image, dstsize=(cols // scale, rows // scale))

    def get_crop(self, cv_image):
        # cut sky
        new_height = 275  # by testing with plt.show()
        # [rows, columns]
        return cv_image[new_height:cv_image.shape[0], 0:cv_image.shape[1]]

    # lane detection
    def get_lines(self, edge_img):
        return cv2.HoughLinesP(edge_img, 2, np.pi / 180, 60, np.array([]), minLineLength=10,
                               maxLineGap=40)

    def get_edges(self, cv_img):
        blur = cv2.GaussianBlur(cv_img, (5, 5), 0)
        edges = cv2.Canny(blur, 20, 80)
        return edges

    # pup lines on raw image
    def get_line_img(self, cv_img, line_vectors, color):
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


        img_roi_l = self.left_RoI.get_RoI(img_edge)
        img_roi_r = self.right_RoI.get_RoI(img_edge)

        #plt.imshow(img_roi_l)
        #plt.show()
        vectors_l = self.get_lines(img_roi_l)
        vectors_r = self.get_lines(img_roi_r)
        print("Left\t", vectors_l)
        print("Left\t", vectors_r)

        img_cv_right = self.get_line_img(img_croped, vectors_r, (0, 255, 0))
        img_cv_both = self.get_line_img(img_cv_right, vectors_l, (255, 0, 0))

        # img_cv_mid = self.get_mid(img_cv_both, vectors_l, vectors_r)
        msgOut = self.bridge.cv2_to_imgmsg(img_cv_both, encoding='rgb8')
        self.publisher.publish(msgOut)

    def get_mid(self, cv_img, l_vectors, r_vectors):
        height = cv_img.shape[0]
        width = cv_img.shape[1]
        img = np.copy(cv_img)
        line_img = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)

        if l_vectors is not None:
            for vector in l_vectors:
                for x1, y1, x2, y2 in vector:
                    lowestXL = x1 if y1 < y2 else x2

        if r_vectors is not None:
            for vector in r_vectors:
                for x1, y1, x2, y2 in vector:
                    lowestXR = x1 if y1 < y2 else x2

        midX = (lowestXL + lowestXR) / 2

        cv2.line(line_img, (midX, height), (midX, 0), (255, 0, 255), 3)

        # place lines on original image
        return cv2.addWeighted(img, 0.8, line_img, 1, 1)


class Region_of_Interest():
    def __init__(self, lower_x, upper_x, height, max_width, min_width, step_size):
        self.lower_x = lower_x
        self.upper_x = upper_x
        self.roi_height = height
        self.min_width = min_width
        self.max_width = max_width
        self.step_size = step_size
        self.l_current_w = self.min_width
        self.u_current_w = self.min_width // 3

    def get_RoI(self, cv_img):
        img_height = cv_img.shape[0]
        img_width = cv_img.shape[1]
        # center = int(width / 2)

        # lower left
        x1 = self.lower_x - self.l_current_w // 2
        y1 = img_height

        # upper left
        x2 = self.upper_x - self.u_current_w // 2
        y2 = self.roi_height

        # upper right
        x3 = self.upper_x + self.u_current_w // 2
        y3 = self.roi_height

        # lower right
        x4 = self.lower_x + self.l_current_w // 2
        y4 = img_height

        # cut along these points
        pts = np.array([[x1, y1], [x2, y2], [x3, y3], [x4, y4]])
        print(pts)
        # white mask (like in lightroom/ photoshop)
        mask = np.zeros(cv_img.shape[:2], np.uint8)
        cv2.drawContours(mask, [pts], -1, (255, 255, 255), -1, cv2.LINE_AA)

        return cv2.bitwise_and(cv_img, cv_img, mask=mask)


def main(args=None):
    rclpy.init(args=args)
    lane_detection = Lane_Detection()
    rclpy.spin(lane_detection)
    lane_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
