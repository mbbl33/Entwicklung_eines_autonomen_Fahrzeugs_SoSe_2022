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

class Image_Converter(Node):

    def __init__(self):
        super().__init__('image_converter')

        # "bruecke" zwischen OpenCV und ROS Image
        self.bridge = CvBridge()

        # /camera/image_raw [sensor_msgs/msg/Image]
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.pub_lane_img, 1)

        self.publisher = self.create_publisher(Image, 'lane_image', 1)

    def get_CV(self, msg_in):
        # ROS image => OpenCV image grau skaliert
        return self.bridge.imgmsg_to_cv2(msg_in, desired_encoding='mono8')

    def get_bw(self, cv_image):
        # ab welcher graustufe soll ein pixel weiss werden
        threshold = 45
        white = 255
        # rueckgabe tupel => erster wert ignoriert
        (thresh, bw_img) = cv2.threshold(cv_image, threshold, white, cv2.THRESH_BINARY)
        return bw_img

    def get_downscale(self, cv_image):
        scale = 2
        rows, cols = map(int, cv_image.shape)
        # //teilen und ergebnis wird abgerundet
        return cv2.pyrDown(cv_image,
                           dstsize=(cols // scale, rows // scale))

    def get_crop(self, cv_image):
        # schneidet himmel ab
        # durch testen durch plt.show()
        new_height = 275
        # [rows, columns]
        return cv_image[new_height:cv_image.shape[0], 0:cv_image.shape[1]]

    def pub_bw_img(self, msg_in):
        img_cv = self.get_CV(msg_in)
        img_cv = self.get_crop(img_cv)
        # img_cv = self.get_downscale(img_cv)
        img_cv = self.get_bw(img_cv)
        msgOut = self.bridge.cv2_to_imgmsg(img_cv, encoding="mono8")
        self.publisher.publish(msgOut)

    # lane detection
    def pub_lane_img(self, msg_in):

        raw_img_cv = self.get_CV(msg_in)
        img_croped = self.get_crop(raw_img_cv)
        img_cv = cv2.cvtColor(img_croped, cv2.COLOR_BGR2RGB)

        img_cv = self.get_bw(img_cv)

        img_cv = self.get_edges(img_cv)
        img_cv = self.get_RoI(img_cv)
        #plt.imshow(img_cv)
        #plt.ylabel('Crop for RoI')
        #plt.show()
        vectors = self.get_lines(img_cv)
        print(vectors)
        img_cv = self.get_line_img(img_croped, vectors)
        msgOut = self.bridge.cv2_to_imgmsg(img_cv, encoding="rgb8")
        self.publisher.publish(msgOut)

        #plt.imshow(img_cv)
        # plt.show()

    def get_line_img(self, cv_img, line_vectors):
        img = np.copy(cv_img)
        line_img = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
        for vector in line_vectors:
            for x1, y1, x2, y2 in vector:
                cv2.line(line_img, (x1, y1), (x2, y2), (0, 255, 0), 3)
        # linien auf orginal bild legen
        return line_img # cv2.addWeighted(img, 0.8, line_img, 1, 0.0)

    def get_lines(self, edge_img):
        # vom opencv tutorial / https://www.youtube.com/watch?v=G0cHyaP9HaQ&t=454s
        return cv2.HoughLinesP(edge_img, 6, np.pi / 180, 150, None) #np.array([]), 40, 25

    def get_edges(self, cv_img):

        edges = cv2.Canny(cv_img, 50, 200)  # werte von wo bis wo Kanten erkannt werden
        return edges

    def get_RoI(self, cv_img):
        # anregung durch https://stackoverflow.com/questions/48301186/cropping-concave-polygon-from-image-using-opencv-python
        height = cv_img.shape[0]
        width = cv_img.shape[1]
        center = int(width / 2)

        # punkte an denen ausgeschnitten wird
        pts = np.array([[0, height], [center, 0], [width, height]])

        # maskieren (wie in lightroom/photoshop)
        pts = pts - pts.min(axis=0)
        mask = np.zeros(cv_img.shape[:2], np.uint8)
        cv2.drawContours(mask, [pts], -1, (255, 255, 255), -1, cv2.LINE_AA)

        return cv2.bitwise_and(cv_img, cv_img, mask=mask)

def main(args=None):
    rclpy.init(args=args)
    image_Converter = Image_Converter()
    rclpy.spin(image_Converter)
    image_Converter.destroy_node()
    rclpy.shutdown


if __name__ == '__main__':
    main()
