#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from math import *

class ObstacleAvoidance:

    def __init__(self):
        rospy.init_node('Couloir', anonymous=True)
        
        rospy.Subscriber('/scan', LaserScan, self.couloir)
        self.command_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.lane_following_continue = rospy.Publisher('/lane_following_continue', Float64, queue_size = 1)

        # self.sub_couloir_open = rospy.Subscriber('/couloir_open', Float64, self.couloir_open_callback, queue_size = 1)
        self.couloir_open = 0
                                                                                                            
        self.sub_couloir_open = rospy.Subscriber('/redline_counter', Float64, self.couloir_open_callback, queue_size = 1)

        self.lane_following_continue = rospy.Publisher('/lane_following_continue', Float64, queue_size = 1)
        
    def couloir(self, scan):
        vitesse = Twist()
        if self.couloir_open == 1:
            self.lane_following_continue.publish(0)

            for i in range(-70, 70):
                if scan.ranges[i] < 0.185:
                    if i < 0:
                        vitesse.linear.x = 0.0
                        vitesse.angular.z = (pi / 8)

                    elif i > 0:
                        vitesse.linear.x = 0.0
                        vitesse.angular.z = -(pi / 8)


                elif all(scan.ranges[i] >= 0.185 for i in range(-70, 70)):
                    vitesse.linear.x = 0.06
                    vitesse.angular.z = 0
                self.command_publisher.publish(vitesse)
        if self.couloir_open == 2:
            self.lane_following_continue.publish(1)
            self.couloir_open = 3



    
    def couloir_open_callback(self,response):
        if int(response.data) == 3:
            self.couloir_open = 1
        elif int(response.data) == 4 and self.couloir_open != 3:
            self.couloir_open = 2


    
    def emergency(self,scan):
        vitesse = Twist()
        for i in range(90, -90, -1):
            if scan.ranges[i] < 0.02:
                if i < 0:
                    vitesse.linear.x = 0.0
                    vitesse.angular.z = (pi / 3)

                elif i > 0:
                    vitesse.linear.x = 0.0
                    vitesse.angular.z = -(pi / 3)

            self.command_publisher.publish(vitesse)

            


if __name__ == '__main__':
    try:
        couloir_work = ObstacleAvoidance()
        rospy.spin()
    except rospy.ROSInterruptException:
       pass

