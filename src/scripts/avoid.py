#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoider:
    def __init__(self):
        rospy.init_node('obstacle_avoider', anonymous=True)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.twist = Twist()
        self.rate = rospy.Rate(10)

    def laser_callback(self, scan):
        front_ranges = scan.ranges[len(scan.ranges)//3: 2*len(scan.ranges)//3]  # 正前方60度
        left_ranges = scan.ranges[0:len(scan.ranges)//6]                        # 左侧30度
        right_ranges = scan.ranges[-len(scan.ranges)//6:]                      # 右侧30度

        front_min = min(front_ranges)
        left_min = min(left_ranges)
        right_min = min(right_ranges)

        safe_distance = 0.1

        if front_min < safe_distance:
            # 前方有障碍，转向空的一侧
            rospy.loginfo("Obstacle ahead! Turning...")
            self.twist.linear.x = 0.0
            if left_min > right_min:
                self.twist.angular.z = 0.5  # 左转
            else:
                self.twist.angular.z = -0.5  # 右转
        else:
            # 前方安全，前进
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0.0

        self.cmd_pub.publish(self.twist)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        ObstacleAvoider().run()
    except rospy.ROSInterruptException:
        pass

