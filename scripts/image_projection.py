#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge



class ImageProjection():
    def __init__(self):
        self.top_x =  72
        self.top_y = 4
        self.bottom_x = 115
        self.bottom_y = 120


        # subscribes raw image 
        self.sub_image_original = rospy.Subscriber('/camera/image', Image, self.cbImageProjection, queue_size=1)


        # publishes ground-project image in raw type 
        self.pub_image_projected = rospy.Publisher('/camera/image_projected', Image, queue_size=1)

        self.cvBridge = CvBridge()


    def cbImageProjection(self, msg_img):

        # converts raw image to opencv image
        cv_image_original = self.cvBridge.imgmsg_to_cv2(msg_img, "bgr8")

        # setting homography variables
        top_x = self.top_x
        top_y = self.top_y
        bottom_x = self.bottom_x
        bottom_y = self.bottom_y

        # adding Gaussian blur to the image of original
        cv_image_original = cv2.GaussianBlur(cv_image_original, (5, 5), 0)

        ## homography transform process
        # selecting 4 points from the original image
        pts_src = np.array([[160 - top_x, 180 - top_y], [160 + top_x, 180 - top_y], [160 + bottom_x, 120 + bottom_y], [160 - bottom_x, 120 + bottom_y]])

        # selecting 4 points from image that will be transformed
        pts_dst = np.array([[200, 0], [800, 0], [800, 600], [200, 600]])

        # finding homography matrix
        h, status = cv2.findHomography(pts_src, pts_dst)

        # homography process
        cv_image_homography = cv2.warpPerspective(cv_image_original, h, (1000, 600))

        # fill the empty space with black triangles on left and right side of bottom
        triangle1 = np.array([[0, 599], [0, 340], [200, 599]], np.int32)
        triangle2 = np.array([[999, 599], [999, 340], [799, 599]], np.int32)
        black = (0, 0, 0)
        white = (255, 255, 255)
        cv_image_homography = cv2.fillPoly(cv_image_homography, [triangle1, triangle2], black)


        # publishes ground-project image in raw type
        self.pub_image_projected.publish(self.cvBridge.cv2_to_imgmsg(cv_image_homography, "bgr8"))

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('image_projection')
    node = ImageProjection()
    node.main()
