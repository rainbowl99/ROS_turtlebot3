#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import os
import sys
import logging.config
import yaml


class ClutteredNavigation:
    def __init__(self):
        log_config_path = os.path.expanduser('~/catkin_ws/src/projet/log/log_config.yaml')
        with open(log_config_path, 'r') as stream:
            log_config = yaml.safe_load(stream)
            logging.config.dictConfig(log_config)
        rospy.init_node('cluttered_navigation', anonymous=True)

        self.is_cluttere_open = 0

        self.bridge = CvBridge()

        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.camera_sub = rospy.Subscriber('/camera/image', Image, self.camera_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.is_cluttere_start = rospy.Subscriber('/redline_counter', Float64, self.order_callback, queue_size = 1)
        self.lane_following_continue = rospy.Publisher('/lane_following_continue', Float64, queue_size = 1)

        self.angular_speed_threshold = 0.1
        self.linear_speed_threshold = 0.1


        self.laser_data = None
        self.image_data = None
        self.odom_data = None
        self.midpoint = None

        self.target_found = False
        self.state = 'searching'

        
        self.target_orientation = None

        self.boundary_detected = False

        self.rate = rospy.Rate(10)

        self.is_exiting = False
        self.Kp = 0.01  # Proportional gain
        self.Ki = 0.000001 # Integral gain
        self.Kd = 0.0001  # Derivative gain
        self.setpoint = 0  # Setpoint
        self.integral = 0  # Integral term
        self.previous_error = 0  # Previous error for derivative term

        self.prev_odom_position = None
        self.total_distance = 0.0

    def order_callback(self, response):
        if int(response.data) == 5:
            self.is_cluttere_open = 1
            self.lane_following_continue.publish(0)
            



    def laser_callback(self, data):
        self.laser_data = data

    def camera_callback(self, data):
        self.image_data = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def odom_callback(self, data):
        self.odom_data = data

    def detect_boundary(self, image):
        # Convert image to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define blue color range
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([130, 255, 255])

        # Threshold the image to get only blue colors
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Add a mask to cover the top half of the image
        mask[:int(mask.shape[0]*0.8), :] = 0

        # Perform morphological operations to remove noise and find contours
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        boundary_detected = False
        obstacle_detected = False
        

        # Set a threshold value to distinguish between boundary and obstacle
        area_threshold_obstacle_min = 2000
        area_threshold_obstacle_max = 100000
        area_threshold_boundary_min= 220
        area_threshold_boundary_max= 1500


        for contour in contours:
            area = cv2.contourArea(contour)
            rospy.loginfo(f"\narea: {area}")
            if area_threshold_boundary_min<area<area_threshold_boundary_max:
                boundary_detected = True
            elif area_threshold_obstacle_min<area<area_threshold_obstacle_max:
                obstacle_detected = True

        self.boundary_detected = boundary_detected
        self.obstacle_detected = obstacle_detected
        #rospy.loginfo(f"\rboundary_detected: {self.boundary_detected},obstacle_detected: {self.obstacle_detected}")



    def find_target(self, image):
        # Convert image to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define green color range
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([90, 255, 255])

        # Threshold the image to get only green colors
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Perform morphological operations to remove noise and find contours
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if self.image_data is not None:
            self.target_distance_threshold = 0.5 * self.image_data.shape[1]
        # Find the two largest contours (green posts) and their bounding boxes
        if len(contours) >= 2:
            largest_contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2]
            bounding_boxes = [cv2.boundingRect(cnt) for cnt in largest_contours]

            # Check if the bounding boxes are close to each other
            box_centers = [(box[0] + box[2] / 2, box[1] + box[3] / 2) for box in bounding_boxes]
            distance = math.sqrt((box_centers[0][0] - box_centers[1][0])**2 + (box_centers[0][1] - box_centers[1][1])**2)
            if distance < self.target_distance_threshold:
                # Calculate the midpoint between the centers of the bounding boxes
                mid_x = (bounding_boxes[0][0] + bounding_boxes[0][2] / 2 + bounding_boxes[1][0] + bounding_boxes[1][2] / 2) / 2
                mid_y = (bounding_boxes[0][1] + bounding_boxes[0][3] / 2 + bounding_boxes[1][1] + bounding_boxes[1][3] / 2) / 2
                self.midpoint = (int(mid_x), int(mid_y))

                # Set self.target_found to True
                target_found_this_iteration= True
            else:
                target_found_this_iteration = False
            # Optionally, draw contours and midpoint on the original image for visualization
            cv2.drawContours(image, largest_contours, -1, (0, 255, 0), 3)
            cv2.circle(image, self.midpoint, 5, (0, 0, 255), -1)
            cv2.imshow("Target Detection", image)
            cv2.waitKey(1)
        else:
            target_found_this_iteration = False

        if self.target_found:
                    self.target_orientation = self.odom_data.pose.pose.orientation
        return target_found_this_iteration
    
    def count_green_pixels(self, image):
        if image is None:
            return 0

        # Convert image to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define green color range
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([90, 255, 255])

        # Threshold the image to get only green colors
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Count the non-zero pixels in the mask (green pixels)
        green_pixels = np.count_nonzero(mask)

        return green_pixels
    
    def blue_contour_distance(self, image):
        # Convert image to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define blue color range
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])

        # Threshold the image to get only blue colors
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Perform morphological operations to remove noise and find contours
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) >= 2:
            largest_blue_contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2]
            blue_bounding_boxes = [cv2.boundingRect(cnt) for cnt in largest_blue_contours]
            blue_box_centers = [(box[0] + box[2] / 2, box[1] + box[3] / 2) for box in blue_bounding_boxes]
            blue_distance = math.sqrt((blue_box_centers[0][0] - blue_box_centers[1][0])**2 + (blue_box_centers[0][1] - blue_box_centers[1][1])**2)
        else:
            blue_distance = 0

        return blue_distance

    
    def navigate(self):
        if self.image_data is None:
            return
        target_found_this_iteration = False
        if not target_found_this_iteration and self.image_data is not None:
            target_found_this_iteration = self.find_target(self.image_data)

        if self.image_data is not None:
            self.detect_boundary(self.image_data)

        green_pixel_threshold = 4500  # Set your green pixel threshold here
        green_pixels = self.count_green_pixels(self.image_data)
        blue_distance = self.blue_contour_distance(self.image_data)

        #rospy.loginfo(f"\ngreen_pixels: {green_pixels}")
        if green_pixels > green_pixel_threshold:
            self.is_exiting = True

        if self.laser_data is not None and self.odom_data is not None:
            move_cmd = Twist()

            if self.state == 'exiting':
                move_cmd.linear.x = 0.3
                move_cmd.angular.z = 0.0

            else:
                # Obstacle avoidance
                min_distance = min(self.laser_data.ranges[-20:] + self.laser_data.ranges[0:20])
                #rospy.loginfo(f"\rmin_distance: {min_distance}")
                if min_distance < 0.2 or self.boundary_detected and blue_distance < self.image_data.shape[1] / 2:
                    # Stop the robot
                    move_cmd.linear.x = 0.0

                    # Determine the direction to turn
                    left_obstacle_distance = np.mean(self.laser_data.ranges[70:110])
                    right_obstacle_distance = np.mean(self.laser_data.ranges[250:290])
                    rospy.loginfo(f"\rleft_obstacle_distance: {left_obstacle_distance}")
                    rospy.loginfo(f"\rright_obstacle_distance: {right_obstacle_distance}")
                    # if left_obstacle_distance > right_obstacle_distance:
                    #     move_cmd.angular.z = 0.5
                    # elif right_obstacle_distance > left_obstacle_distance:
                    #     move_cmd.angular.z = -0.5
                    # else:
                    #     move_cmd.angular.z = 0.5
                    move_cmd.angular.z = 0.5
                        
                else:
                    if self.state == 'searching':
                        # Move forward and rotate while searching for the target
                        move_cmd.linear.x = 0.1
                        move_cmd.angular.z = -0.3

                    elif self.state == 'navigating':
                        if self.target_found and self.midpoint is not None:
                            
                            # Calculate the error
                            error = self.midpoint[0] - self.image_data.shape[1] / 2

                            # Update the integral term
                            self.integral += error

                            # Calculate the derivative term
                            derivative = error - self.previous_error

                            # Use the PID controller to calculate the angular velocity
                            move_cmd.angular.z = - (self.Kp * error + self.Ki * self.integral + self.Kd * derivative)

                            # Move forward
                            move_cmd.linear.x = 0.2

                            # Update the previous error for the next iteration
                            self.previous_error = error

            rospy.loginfo("\rState: {: <10}. Linear velocity: {: <10}, Angular velocity: {: <10}".format(self.state, move_cmd.linear.x, move_cmd.angular.z))
            self.cmd_vel_pub.publish(move_cmd)
            
            if self.odom_data is not None:
                current_position = np.array([self.odom_data.pose.pose.position.x, self.odom_data.pose.pose.position.y])
                if self.prev_odom_position is not None:
                    distance = np.linalg.norm(current_position - self.prev_odom_position)
                    self.total_distance += distance
                    rospy.loginfo("Total distance moved: {:.2f} meters".format(self.total_distance))

                self.prev_odom_position = current_position


            # Update the robot state based on whether the target is found
            if self.is_exiting:
                self.state = 'exiting'
            else:
                if target_found_this_iteration:
                    self.state = 'navigating'
                    self.target_found = True
                else:
                    self.state = 'searching'
                    self.target_found = False


            



    def run(self):
        while not rospy.is_shutdown():
            if self.is_cluttere_open == 1:
                self.navigate()
                self.rate.sleep()
        rospy.spin()
            

if __name__ == '__main__':
    cluttered_navigation = ClutteredNavigation()
    cluttered_navigation.run()
