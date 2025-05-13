#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def move_straight():
    # 初始化节点
    rospy.init_node('straight_mover')

    # 创建Publisher，发布到/cmd_vel
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # 创建Twist消息
    twist = Twist()
    twist.linear.x = 0.2   # 向前移动0.2 m/s
    twist.angular.z = 0.0  # 不转弯，保持直线

    rate = rospy.Rate(10)  # 10Hz
    duration = 20           # 移动时间（秒）

    rospy.loginfo("Robot moving straight...")

    t0 = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        t1 = rospy.Time.now().to_sec()
        if t1 - t0 > duration:
            break
        pub.publish(twist)
        rate.sleep()

    # 停止机器人
    stop_twist = Twist()
    pub.publish(stop_twist)
    rospy.loginfo("Robot stopped.")

if __name__ == '__main__':
    try:
        move_straight()
    except rospy.ROSInterruptException:
        pass

