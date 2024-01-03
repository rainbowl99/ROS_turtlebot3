#!/usr/bin/env python3
# -*- coding: utf-8 -*-

 
import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from projet.srv import mission_change, mission_changeResponse

class ControlLane():
    def __init__(self):

        # self.obstacle_open = rospy.ServiceProxy('obstalce_start',mission_change)
        # self.lane_open = rospy.ServiceProxy('pause_velocity',mission_changeResponse)
        self.sub_lane_open = rospy.Subscriber('/lane_following_continue', Float64, self.lane_open_callback, queue_size = 1)
        self.sub_lane = rospy.Subscriber('/detect/lane', Float64, self.cbFollowLane, queue_size = 1)
        self.sub_max_vel = rospy.Subscriber('/control/max_vel', Float64, self.cbGetMaxVel, queue_size = 1)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        
        self.lane_open = 1

        self.lastError = 0
        self.MAX_VEL = 0.1

        rospy.on_shutdown(self.fnShutDown)

    def cbGetMaxVel(self, max_vel_msg):
        self.MAX_VEL = max_vel_msg.data
    
    def lane_open_callback(self,response):
        if int(response.data) == 1:
            self.lane_open = 1
        else:
            self.lane_open = 0

    def cbFollowLane(self, desired_center):
        center = desired_center.data

        error = center - 500

        Kp = 0.005
        Kd = 0.007

        angular_z = Kp * error + Kd * (error - self.lastError)
        self.lastError = error
        
        twist = Twist()
        
        twist.linear.x = min(self.MAX_VEL * ((1 - abs(error) / 500) ** 2.2), 0.05)
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
        if self.lane_open == 1:
            self.pub_cmd_vel.publish(twist)

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist) 

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('driving_control')
    node = ControlLane()
    node.main()
