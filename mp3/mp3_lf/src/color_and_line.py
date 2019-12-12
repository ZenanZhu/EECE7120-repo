#!/usr/bin/env python
import sys
import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage

class mp3_color_line():
    def __init__(self):
        # subscribe from duckiebot CompressedImage
        self.sub_image = rospy.Subscriber("~compressed", CompressedImage, self.lane_filter, queue_size=1)
        self.bridge = CvBridge()
        # publish yellow and white hough transform images
        self.pub_yellow = rospy.Publisher("~yellow_hough_image", Image, queue_size=1)
        self.pub_white = rospy.Publisher("~white_hough_image", Image, queue_size=1)

    def get_lines(self, original_image, filtered_image):
        # do our hough transform on the white image
        # resolution: 1 pixel radius, 1 degree rotational
        r_res = 1
        theta_res = np.pi/180
        # threshold: number of intersections to define a line
        thresh = 6
        # min_length: minimum number of points to form a line
        min_length = 4
        # max_gap: maximum gap between two points to be considered a line
        max_gap = 4
        lines = cv2.HoughLinesP(filtered_image, r_res, theta_res, thresh, np.empty(1), min_length, max_gap)

        output = np.copy(original_image)
        if lines is not None:
            # grab the first line
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (0,0,255), 3, cv2.LINE_AA)
        return output

    def lane_filter(self,image):
        # Image to numpy array
        np_arr = np.fromstring(image.data, np.uint8)
        # Decode to cv2 image and store
        image_cv = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # The incoming image is BGR format, convert it to HSV
        hsv = cv2.cvtColor(image_cv, cv2.COLOR_BGR2HSV)

        # Filter for only white pixels. Experiment with values as needed
        white_filter = cv2.inRange(hsv, (0,0,150), (255,55,255))
        # Filter for only yellow pixels. Experiment with values as needed
        yellow_filter = cv2.inRange(hsv, (27,110,110), (55,255,255))

        # Create a kernel to dilate the image.
        # Experiment with the numbers in parentheses (optional)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))

        # Dilate both the white and yellow images.
        # No need to experiment here.
        white_dilate = cv2.dilate(white_filter, kernel)
        yellow_dilate = cv2.dilate(yellow_filter, kernel)

        # Perform edge detection on the original image.
        # Experiment with the first two numbers. Aperture size experimentation optional
        edges = cv2.Canny(image_cv, 0, 300, apertureSize=3)

        # Use the edges to refine the lines in both white and yellow images
        # No need to experiment here
        white_edges = cv2.bitwise_and(white_dilate, edges)
        yellow_edges = cv2.bitwise_and(yellow_dilate, edges)

        white_output = self.get_lines(image_cv, white_edges)
        yellow_output = self.get_lines(image_cv, yellow_edges)

        try:
            self.pub_white.publish(self.bridge.cv2_to_imgmsg(white_output, "bgr8"))
            self.pub_yellow.publish(self.bridge.cv2_to_imgmsg(yellow_output, "bgr8"))
        except CvBridgeError as e:
            print(e)


if __name__ == "__main__":

    rospy.init_node("color_and_line", anonymous=False)  # adapted to sonjas default file

    color_and_line = mp3_color_line()
rospy.spin()
