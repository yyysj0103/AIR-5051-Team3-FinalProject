#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from spark_carry_object.msg import position
import cv2
from ultralytics import YOLO
import time
import threading
import os

class SecurityRobot:
    def __init__(self):
        rospy.init_node('security_robot_node')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.arm_pub = rospy.Publisher('/position_write_topic', position, queue_size=10)
        self.person_pub = rospy.Publisher('/detected_person', String, queue_size=1)
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

        self.bridge = CvBridge()
        self.model = YOLO('/home/spark/spark_noetic/src/spark_security_guard/models/yolov8n.pt')

        self.last_detection_time = 0
        self.following = False
        self.person_center_x = None
        self.frame_width = 640  # default value, will update from image

        self.patrol_twist = Twist()
        self.patrol_twist.linear.x = 0.1

        rospy.loginfo("Security robot initialized.")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.frame_width = frame.shape[1]
        except Exception as e:
            rospy.logerr(f"cv_bridge error: {e}")
            return

        results = self.model(frame)[0]
        person_detected = False

        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            if self.model.names[cls_id] == 'person' and conf > 0.5:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx = (x1 + x2) // 2
                self.person_center_x = cx

                # 发布检测信息
                self.person_pub.publish(f"{cx}")
                person_detected = True
                break

        if person_detected:
            self.last_detection_time = time.time()
            threading.Thread(target=self.play_warning_sound).start()
            self.following = True
        elif time.time() - self.last_detection_time > 2:
            self.following = False
            
        cv2.namedWindow("Camera View", cv2.WINDOW_NORMAL)
        cv2.imshow("Camera View", frame)
        cv2.waitKey(1)

    def control_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.following and self.person_center_x is not None:
                error = self.person_center_x - self.frame_width // 2
                twist = Twist()
                twist.linear.x = 0.2
                twist.angular.z = -error / 500.0  # 简单比例控制
                self.cmd_pub.publish(twist)

                if error > 100:
                    self.swing_arm("right")
                elif error < -100:
                    self.swing_arm("left")

            else:
                # 巡逻
                self.cmd_pub.publish(self.patrol_twist)
            rate.sleep()

    def swing_arm(self, direction):
        pos = position()
        pos.x = 200
        pos.z = 100
        if direction == "left":
            pos.y = -100
        else:
            pos.y = 100
        self.arm_pub.publish(pos)
        rospy.loginfo(f"Swing arm {direction}: {pos}")
        
    def play_warning_sound(self):
        for _ in range(3):
            os.system('play -nq -t alsa synth 0.2 sine 880')  # 0.2秒 880Hz
            time.sleep(0.3)

if __name__ == '__main__':
    try:
        robot = SecurityRobot()
        robot.control_loop()
    except rospy.ROSInterruptException:
        pass

