import cv2
import numpy as np


class Region_of_Interest():
    def __init__(self, lower_x, lower_y, upper_x, height, max_width, min_width, step_size):
        self.lower_x = lower_x
        self.lower_y = lower_y
        self.upper_x = upper_x
        self.roi_height = height
        self.min_width = min_width
        self.max_width = max_width
        self.step_size = step_size
        self.current_w = self.min_width

    def get_RoI(self, cv_img):
        img_height = cv_img.shape[0]

        # lower left
        x1 = self.lower_x - self.current_w // 2
        y1 = img_height - self.lower_y

        # upper left
        x2 = self.upper_x - (self.current_w // 3) // 2
        y2 = self.roi_height

        # upper right
        x3 = self.upper_x + (self.current_w // 3) // 2
        y3 = self.roi_height

        # lower right
        x4 = self.lower_x + self.current_w // 2
        y4 = img_height - self.lower_y

        # cut along these points
        pts = np.array([[x1, y1], [x2, y2], [x3, y3], [x4, y4]])

        # white mask (like in lightroom/ photoshop)
        mask = np.zeros(cv_img.shape[:2], np.uint8)
        cv2.drawContours(mask, [pts], -1, (255, 255, 255), -1, cv2.LINE_AA)

        return cv2.bitwise_and(cv_img, cv_img, mask=mask)

    def increase_roi(self):
        if (self.current_w < self.max_width):
            self.current_w += self.step_size

    def reset_roi(self):
        self.current_w = self.min_width