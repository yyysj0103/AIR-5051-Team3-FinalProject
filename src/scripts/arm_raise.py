#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from spark_carry_object.msg import position  # 你项目里面定义的 position 消息

def lift_arm():
    rospy.init_node('simple_lift_arm', anonymous=True)

    pub = rospy.Publisher('position_write_topic', position, queue_size=10)
    
    rospy.sleep(1)  # 等待一下 publisher 建立连接
    
    # 创建 position 消息
    pos = position()
    
    # 设置初始位置 (比如当前位置附近，假设基准位置是 x=200, y=0, z=0)
    pos.x = 200
    pos.y = 0
    pos.z = 0
    pub.publish(pos)
    rospy.loginfo(f"Move to base position: x={pos.x}, y={pos.y}, z={pos.z}")

    rospy.sleep(2)  # 等2秒，机械臂到达初始位置

    # 抬起Z轴，往上升，比如升到 100
    pos.z = 100
    pub.publish(pos)
    rospy.loginfo(f"Lift arm: x={pos.x}, y={pos.y}, z={pos.z}")

    rospy.sleep(2)  # 等待动作完成

    rospy.loginfo("Finished lifting arm.")

if __name__ == '__main__':
    try:
        lift_arm()
    except rospy.ROSInterruptException:
        pass

