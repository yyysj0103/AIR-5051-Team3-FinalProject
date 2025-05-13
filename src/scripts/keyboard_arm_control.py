#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from spark_carry_object.msg import position
import sys
import select
import termios
import tty

def get_key():
    '''读取键盘按键（非阻塞）'''
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)  # 0.1秒超时
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def keyboard_control():
    rospy.init_node('keyboard_lift_arm', anonymous=True)
    pub = rospy.Publisher('position_write_topic', position, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    # 初始位置
    pos = position()
    pos.x = 200
    pos.y = 0
    pos.z = 50

    move_step = 10  # 每次移动多少单位

    rospy.loginfo("""
    键盘控制机械臂！
    W/S: 前后 (X轴增加/减少)
    A/D: 左右 (Y轴增加/减少)
    Q/E: 上下 (Z轴增加/减少)
    X: 退出
    """)

    while not rospy.is_shutdown():
        key = get_key()
        if key == 'w':
            pos.x += move_step
        elif key == 's':
            pos.x -= move_step
        elif key == 'a':
            pos.y += move_step
        elif key == 'd':
            pos.y -= move_step
        elif key == 'q':
            pos.z += move_step
        elif key == 'e':
            pos.z -= move_step
        elif key == 'x':
            rospy.loginfo("退出控制")
            break
            
        pos.x = max(-200, min(400, pos.x))
        pos.y = max(-200, min(200, pos.y))
        pos.z = max(-200, min(200, pos.z))

        pub.publish(pos)
        rospy.loginfo(f"目标位置: x={pos.x}, y={pos.y}, z={pos.z}")
        rate.sleep()

if __name__ == '__main__':
    try:
        keyboard_control()
    except rospy.ROSInterruptException:
        pass

