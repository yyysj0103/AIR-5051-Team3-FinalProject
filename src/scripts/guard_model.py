#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import String
from cv_bridge import CvBridge
from spark_carry_object.msg import position
import cv2
from ultralytics import YOLO
import time
import threading
import os
import tkinter as tk
from PIL import Image as PILImage, ImageTk
import math
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
class SecurityRobotGUI:
    def __init__(self):
        self.root = tk.Tk()
        rospy.init_node('security_robot_node')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.arm_pub = rospy.Publisher('/position_write_topic', position, queue_size=10)
        self.person_pub = rospy.Publisher('/detected_person', String, queue_size=1)
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', RosImage, self.image_callback)
        self.vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.velocity_callback)

        self.bridge = CvBridge()
        self.model = YOLO('/home/spark/spark_noetic/src/spark_security_guard/models/yolov8n.pt')

        self.last_detection_time = 0
        self.following = False
        self.person_center_x = None
        self.person_box_height = 0
        self.frame_width = 640
        self.mode = "idle"  # idle, auto, manual, guard

        self.patrol_twist = Twist()
        self.patrol_twist.linear.x = 0.1

        self.manual_twist = Twist()
        self.manual_twist.linear.x = 0.0
        self.manual_twist.angular.z = 0.0

        self.pos = position()
        self.pos.x = 200
        self.pos.y = 0
        self.pos.z = 50

        self.origin_pose = self.pos
        self.current_velocity = Twist()
        self.warning_label_visible = False

        self.build_gui()
        threading.Thread(target=self.control_loop, daemon=True).start()
        self.root.bind("<KeyPress>", self.key_press)
        self.root.bind("<KeyRelease>", self.key_release)
        self.root.mainloop()

    def build_gui(self):
        self.root.title("安保机器人控制界面")
        self.root.geometry("1000x600")
        self.root.configure(bg="#f0f0f0")

        main_frame = tk.Frame(self.root, bg="#f0f0f0")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # 左侧信息与控制面板
        left_frame = tk.Frame(main_frame, bg="#f0f0f0", width=300)
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)

        self.status_label = tk.Label(left_frame, text="状态: 巡逻中", font=("Courier", 12), bg="#000", fg="lime", width=40, anchor="w")
        self.status_label.pack(anchor="w", pady=2)

        self.param_label = tk.Label(left_frame, text="机械臂位置: x=200 y=0 z=50", font=("Courier", 12), bg="#000", fg="cyan", width=40, anchor="w")
        self.param_label.pack(anchor="w", pady=2)

        button_frame = tk.LabelFrame(left_frame, text="模式选择", bg="#f0f0f0")
        button_frame.pack(pady=10, fill=tk.X)

        tk.Button(button_frame, text="自动模式", command=lambda: self.set_mode("auto")).grid(row=0, column=0, padx=5, pady=5)
        tk.Button(button_frame, text="手动模式", command=lambda: self.set_mode("manual")).grid(row=0, column=1, padx=5, pady=5)
        tk.Button(button_frame, text="防护模式", command=lambda: self.set_mode("guard")).grid(row=1, column=0, padx=5, pady=5)
        tk.Button(button_frame, text="静止模式", command=lambda: self.set_mode("idle")).grid(row=1, column=1, padx=5, pady=5)

        control_frame = tk.LabelFrame(left_frame, text="手动攻击", bg="#f0f0f0")
        control_frame.pack(pady=10)

        tk.Button(control_frame, text="↑", width=5, command=lambda: self.move_manual('Up')).grid(row=0, column=1)
        tk.Button(control_frame, text="←", width=5, command=lambda: self.move_manual('Left')).grid(row=1, column=0)
        tk.Button(control_frame, text="→", width=5, command=lambda: self.move_manual('Right')).grid(row=1, column=2)
        tk.Button(control_frame, text="↓", width=5, command=lambda: self.move_manual('Down')).grid(row=1, column=1)
        tk.Button(control_frame, text="升", width=5, command=lambda: self.move_manual('q')).grid(row=2, column=0)
        tk.Button(control_frame, text="降", width=5, command=lambda: self.move_manual('e')).grid(row=2, column=2)

        # 右侧图像与警告
        right_frame = tk.Frame(main_frame, bg="#f0f0f0")
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.warning_label = tk.Label(right_frame, text="警告：发现入侵者！", font=("Arial", 24, "bold"), fg="red", bg="#f0f0f0")
        self.warning_label.pack(pady=5)
        self.warning_label.pack_forget()

        self.image_label = tk.Label(right_frame)
        self.image_label.pack()


    def set_mode(self, mode):
        self.mode = mode
        rospy.loginfo(f"切换到模式: {mode}")
        self.status_label.config(text=f"状态: {mode}")
        self.cmd_pub.publish(Twist())  # 停止移动
        self.following = False

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
                person_detected = True
                break

        if person_detected:
            self.last_detection_time = time.time()
            self.following = True
        elif time.time() - self.last_detection_time > 2:
            self.following = False

    def key_press(self, event):
        if self.mode == "manual":
            if event.keysym == 'w':
                self.manual_twist.linear.x = 0.2
            elif event.keysym == 's':
                self.manual_twist.linear.x = -0.2
            elif event.keysym == 'a':
                self.manual_twist.angular.z = 0.5
            elif event.keysym == 'd':
                self.manual_twist.angular.z = -0.5
            self.cmd_pub.publish(self.manual_twist)

    def key_release(self, event):
        if self.mode == "manual" and event.keysym in ['w', 's', 'a', 'd']:
            self.manual_twist = Twist()
            self.cmd_pub.publish(self.manual_twist)

    def get_current_pose(self):
        try:
            odom_msg = rospy.wait_for_message("/odom", Odometry, timeout=1.0)
            position = odom_msg.pose.pose.position
            orientation_q = odom_msg.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            _, _, yaw = euler_from_quaternion(orientation_list)
            return (position.x, position.y, yaw)
        except Exception as e:
            rospy.logwarn(f"Failed to get current pose: {e}")
            return (0.0, 0.0, 0.0)

    def compute_twist_to_pose(self, target_pose):
        current_x, current_y, current_yaw = self.get_current_pose()
        target_x, target_y, target_yaw = target_pose

        dx = target_x - current_x
        dy = target_y - current_y
        distance = (dx ** 2 + dy ** 2) ** 0.5

        angle_to_target = math.atan2(dy, dx)
        angular_error = angle_to_target - current_yaw

        # Normalize angle to [-pi, pi]
        while angular_error > math.pi:
            angular_error -= 2 * math.pi
        while angular_error < -math.pi:
            angular_error += 2 * math.pi

        twist = Twist()
        twist.linear.x = min(0.2, distance)
        twist.angular.z = max(-0.5, min(0.5, angular_error))
        return twist

    def control_loop(self):
        rate = rospy.Rate(10)
        previous_distance = None
        guard_origin_recorded = False
        while not rospy.is_shutdown():
            if self.mode == "auto":
                if self.following and self.person_center_x is not None:
                    error = self.person_center_x - self.frame_width // 2
                    twist = Twist()
                    twist.linear.x = 0.2
                    twist.angular.z = -error / 500.0
                    self.cmd_pub.publish(twist)
                    if error > 100:
                        self.swing_arm("right")
                    elif error < -100:
                        self.swing_arm("left")
                else:
                    self.cmd_pub.publish(self.patrol_twist)
            elif self.mode == "manual":
                self.cmd_pub.publish(self.manual_twist)
                self.arm_pub.publish(self.pos)
            elif self.mode == "guard":
                if not guard_origin_recorded:
                    self.guard_origin_pose = self.get_current_pose()
                    guard_origin_recorded = True

                if self.following and self.person_center_x is not None:
                    current_distance = self.person_box_height
                    if previous_distance is None or current_distance <= previous_distance:
                        error = self.person_center_x - self.frame_width // 2
                        twist = Twist()
                        twist.linear.x = 0.2
                        twist.angular.z = -error / 500.0
                        self.cmd_pub.publish(twist)
                        self.swing_arm("block")
                    else:
                        twist = self.compute_twist_to_pose(self.guard_origin_pose)
                        self.cmd_pub.publish(twist)
                    previous_distance = current_distance
                else:
                    self.cmd_pub.publish(Twist())
                    self.arm_pub.publish(self.origin_pose)
                    previous_distance = None
            rate.sleep()


    def swing_arm(self, direction):
        pos = position()
        pos.x = 200
        pos.z = 100
        pos.y = 100 if direction == "right" else -100 if direction == "left" else 0
        self.arm_pub.publish(pos)

    def velocity_callback(self, msg):
        self.current_velocity = msg

if __name__ == '__main__':
    try:
        SecurityRobotGUI()
    except rospy.ROSInterruptException:
        pass

