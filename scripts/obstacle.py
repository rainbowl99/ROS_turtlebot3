#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import rospy
import os 
from enum import Enum
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from projet.srv import mission_change, mission_changeResponse
from math import pi

class DetectContruction():
    def __init__(self):
        # When the third red line is detected, run the current node and when the fourth red line is detected, stop running the current node.
        self.is_obstacle_detected = False

        self.is_obstacle_finished = False
        # counter for obstacle
        self.counter = 0

        # self.server_obstacle = rospy.Service('pause_velocity', Float64, self.lane_following_continue)
        # subscribes state 
        self.scan_info = rospy.Subscriber('/scan', LaserScan , self.cbScanObstacle, queue_size=1) # On souscrit au topic scan pour recuperer les données lidar 
        self.is_obstacle_start = rospy.Subscriber('/redline_counter', Float64, self.cbAvoidObstalce, queue_size = 1)
        # publishes state
        self.obstacle_return = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.lane_following_continue = rospy.Publisher('/lane_following_continue', Float64, queue_size = 1)

    def cbAvoidObstalce(self, order):
        Vitesse = Twist()
        Vitesse0 = Twist()
        Vitesse0.linear.x = 0.0
        Vitesse0.angular.z = 0.0
        if int(order.data) == 2 and self.is_obstacle_finished == False:
            

            if self.is_obstacle_detected == True:
                self.obstacle_return.publish(Vitesse0)
                self.counter += 1
                
                if self.counter == 1:
                    self.lane_following_continue.publish(0)

                    rospy.sleep(3)

                    rospy.loginfo(self.is_obstacle_detected)
                    # first obstacle -- left
                    timeNow = rospy.Time.now()
                    while (rospy.Time.now() - timeNow).to_sec() < 1.2:
                        Vitesse.linear.x = 0.0
                        Vitesse.angular.z = pi/2
                        self.obstacle_return.publish(Vitesse)
                    self.obstacle_return.publish(Vitesse0)
                    rospy.sleep(3)
    

                    # first obstacle -- straight
                    timeNow = rospy.Time.now()
                    while (rospy.Time.now() - timeNow).to_sec() < 5:
                        Vitesse.linear.x = 0.05
                        Vitesse.angular.z = 0.0
                        self.obstacle_return.publish(Vitesse)
                    self.obstacle_return.publish(Vitesse0)
                    rospy.sleep(3)

                    
                    # first obstacle -- right
                    timeNow = rospy.Time.now()
                    while (rospy.Time.now() - timeNow).to_sec() < 1.0:
                        Vitesse.linear.x = 0.0
                        Vitesse.angular.z = -pi/2
                        self.obstacle_return.publish(Vitesse)
                    self.obstacle_return.publish(Vitesse0)
                    self.lane_following_continue.publish(1)
                    rospy.sleep(3)
                    


                elif self.counter == 2:   
                    self.lane_following_continue.publish(0)
                    rospy.sleep(3)

                    # second obstacle -- right
                    timeNow = rospy.Time.now()
                    while (rospy.Time.now() - timeNow).to_sec() < 1.2:
                        Vitesse.linear.x = 0.0
                        Vitesse.angular.z = -pi/2
                        self.obstacle_return.publish(Vitesse)
                    self.obstacle_return.publish(Vitesse0)
                    rospy.sleep(3)
                    
    

                    # second obstacle -- straight
                    timeNow = rospy.Time.now()
                    while (rospy.Time.now() - timeNow).to_sec() < 5:
                        Vitesse.linear.x = 0.05
                        Vitesse.angular.z = 0.0
                        self.obstacle_return.publish(Vitesse)
                    self.obstacle_return.publish(Vitesse0)
                    rospy.sleep(3)

                    # second obstacle -- left
                    timeNow = rospy.Time.now()
                    while (rospy.Time.now() - timeNow).to_sec() < 1.05:
                        Vitesse.linear.x = 0.0
                        Vitesse.angular.z = pi/2
                        self.obstacle_return.publish(Vitesse)
                    self.obstacle_return.publish(Vitesse0)


                    rospy.sleep(3)

                    # second obstacle -- straight
                    timeNow = rospy.Time.now()
                    while (rospy.Time.now() - timeNow).to_sec() < 1:
                        Vitesse.linear.x = 0.05
                        Vitesse.angular.z = 0.0
                        self.obstacle_return.publish(Vitesse)
                    self.obstacle_return.publish(Vitesse0)
                    
                    self.is_obstacle_finished = True
                    self.lane_following_continue.publish(1)
                    
            
            self.is_obstacle_detected == False
        


    def cbScanObstacle(self, scan):
        angle_scan = 25
        scan_start = 0 - angle_scan
        scan_end = 0 + angle_scan
        threshold_distance = 0.2

        for i in range(scan_start, scan_end):
            if scan.ranges[i] < threshold_distance and scan.ranges[i] > 0.01:
                self.is_obstacle_detected = True
            else:
                self.is_obstacle_detected = False
    
    # def lane_following_continue(self,req):
    #     if self.is_obstacle_finished == True:
    #         return mission_changeResponse(1)
    #     elif self.is_obstacle_finished == False:
    #         return mission_changeResponse(0)
        
    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('obstacle')
    node = DetectContruction()
    node.main()
