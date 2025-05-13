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
        self.guard_mode = False
        self.person_center_x = None
        self.person_bbox_height = 0
        self.frame_width = 640
        self.auto_mode = True

        self.patrol_twist = Twist()
        self.patrol_twist.linear.x = 0.1

        self.manual_twist = Twist()
        self.manual_twist.linear.x = 0.0
        self.manual_twist.angular.z = 0.0

        self.pos = position()
        self.pos.x = 200
        self.pos.y = 0
        self.pos.z = 50

        self.current_velocity = Twist()
        self.warning_label_visible = False

        self.build_gui()
        threading.Thread(target=self.control_loop, daemon=True).start()
        threading.Thread(target=self.external_display_loop, daemon=True).start()
        self.root.bind("<KeyPress>", self.key_press)
        self.root.bind("<KeyRelease>", self.key_release)
        self.root.mainloop()

    def build_gui(self):
        self.root.title("安保机器人控制界面")
        self.root.geometry("1000x600")
        self.root.configure(bg="#f0f0f0")

        main_frame = tk.Frame(self.root, bg="#f0f0f0")
        main_frame.pack(fill=tk.BOTH, expand=True)

        left_frame = tk.Frame(main_frame, bg="#f0f0f0")
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=5)

        right_frame = tk.Frame(main_frame, bg="#f0f0f0")
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=5)

        self.status_label = tk.Label(left_frame, text="状态: 巡逻中", font=("Courier", 12), bg="#000", fg="lime", width=40, anchor="w")
        self.status_label.pack(anchor="w")

        self.param_label = tk.Label(left_frame, text="机械臂位置: x=200 y=0 z=50", font=("Courier", 12), bg="#000", fg="cyan", width=40, anchor="w")
        self.param_label.pack(anchor="w")

        self.velocity_label = tk.Label(left_frame, text="速度: 0.00 m/s, 0.00 rad/s", font=("Courier", 12), bg="#000", fg="yellow", width=40, anchor="w")
        self.velocity_label.pack(anchor="w")

        self.mode_button = tk.Button(left_frame, text="切换模式：自动", command=self.toggle_mode)
        self.mode_button.pack(anchor="w", pady=10)

        self.guard_button = tk.Button(left_frame, text="进入防护模式", command=self.toggle_guard_mode)
        self.guard_button.pack(anchor="w", pady=5)

        self.warning_label = tk.Label(right_frame, text="警告：发现入侵者！", font=("Arial", 24, "bold"), fg="red", bg="#f0f0f0")
        self.warning_label.place_forget()

        self.image_label = tk.Label(right_frame)
        self.image_label.pack(pady=5)

        control_frame = tk.LabelFrame(left_frame, text="手动控制", bg="#f0f0f0")
        control_frame.pack(pady=10)

        tk.Button(control_frame, text="↑", width=5, command=lambda: self.move_manual('Up')).grid(row=0, column=1)
        tk.Button(control_frame, text="←", width=5, command=lambda: self.move_manual('Left')).grid(row=1, column=0)
        tk.Button(control_frame, text="→", width=5, command=lambda: self.move_manual('Right')).grid(row=1, column=2)
        tk.Button(control_frame, text="↓", width=5, command=lambda: self.move_manual('Down')).grid(row=1, column=1)
        tk.Button(control_frame, text="升", width=5, command=lambda: self.move_manual('q')).grid(row=2, column=0)
        tk.Button(control_frame, text="降", width=5, command=lambda: self.move_manual('e')).grid(row=2, column=2)

    def toggle_guard_mode(self):
        self.guard_mode = not self.guard_mode
        if self.guard_mode:
            self.guard_button.config(text="退出防护模式")
            self.status_label.config(text="状态: 防护模式中")
        else:
            self.guard_button.config(text="进入防护模式")
            self.status_label.config(text="状态: 巡逻中")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.frame_width = frame.shape[1]
        except Exception as e:
            rospy.logerr(f"cv_bridge error: {e}")
            return

        results = self.model(frame)[0]
        person_detected = False
        max_height = 0

        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            if self.model.names[cls_id] == 'person' and conf > 0.5:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx = (x1 + x2) // 2
                self.person_center_x = cx
                max_height = max(max_height, y2 - y1)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                self.person_pub.publish(f"{cx}")
                person_detected = True
                break

        if person_detected:
            self.person_bbox_height = max_height
            self.last_detection_time = time.time()
            threading.Thread(target=self.play_warning_sound, daemon=True).start()
            self.following = True
            if not self.guard_mode:
                self.status_label.config(text="状态: 发现入侵者！")
                self.show_warning_label()
        elif time.time() - self.last_detection_time > 2:
            self.following = False
            if not self.guard_mode:
                self.status_label.config(text="状态: 巡逻中")
            self.warning_label.place_forget()

        frame_gui = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_gui = cv2.resize(frame_gui, (400, 300))
        img = PILImage.fromarray(frame_gui)
        imgtk = ImageTk.PhotoImage(image=img)
        self.image_label.imgtk = imgtk
        self.image_label.config(image=imgtk)

        # cv2.imshow("Security Camera Feed", frame)
        # cv2.waitKey(1)

    def control_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.guard_mode:
                if self.person_bbox_height > 150:  # 接近的判断
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_pub.publish(twist)
                    self.swing_arm("block")
                    self.status_label.config(text="状态: 阻止入侵")
                else:
                    self.cmd_pub.publish(self.patrol_twist)
                    self.status_label.config(text="状态: 巡逻中")
            elif self.auto_mode:
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
            rate.sleep()

    def swing_arm(self, direction):
        pos = position()
        pos.x = 200
        pos.z = 100
        if direction == "left":
            pos.y = -100
        elif direction == "right":
            pos.y = 100
        elif direction == "block":
            pos.y = 0
        self.arm_pub.publish(pos)

    def play_warning_sound(self):
        for _ in range(3):
            os.system('play -nq -t alsa synth 0.2 sine 880')
            time.sleep(0.3)

    def velocity_callback(self, msg):
        self.current_velocity = msg
        self.velocity_label.config(text=f"速度: {msg.linear.x:.2f} m/s, {msg.angular.z:.2f} rad/s")

    def show_warning_label(self):
        def blink():
            while time.time() - self.last_detection_time < 2:
                self.warning_label.place(relx=0.5, rely=0.05, anchor="n")
                self.warning_label.update()
                time.sleep(0.3)
                self.warning_label.place_forget()
                time.sleep(0.3)
        threading.Thread(target=blink, daemon=True).start()

    def external_display_loop(self):
        while not rospy.is_shutdown():
            key = cv2.waitKey(30) & 0xFF
            if key == 27:
                break
        cv2.destroyAllWindows()

    def toggle_mode(self):
        self.auto_mode = not self.auto_mode
        mode_text = "自动" if self.auto_mode else "手动"
        self.mode_button.config(text=f"切换模式：{mode_text}")

    def key_press(self, event):
        if not self.auto_mode:
            if event.keysym == 'w':
                self.manual_twist.linear.x = 0.2
            elif event.keysym == 's':
                self.manual_twist.linear.x = -0.2
            elif event.keysym == 'a':
                self.manual_twist.angular.z = 0.5
            elif event.keysym == 'd':
                self.manual_twist.angular.z = -0.5
            elif event.keysym == 'Up':
                self.pos.z += 10
            elif event.keysym == 'Down':
                self.pos.z -= 10
            elif event.keysym == 'Left':
                self.pos.y += 10
            elif event.keysym == 'Right':
                self.pos.y -= 10
            self.cmd_pub.publish(self.manual_twist)
            self.arm_pub.publish(self.pos)
            self.update_param_label()

    def key_release(self, event):
        if not self.auto_mode:
            if event.keysym in ['w', 's', 'a', 'd']:
                self.manual_twist = Twist()
                self.cmd_pub.publish(self.manual_twist)

    def move_manual(self, key):
        step = 10
        if key == 'q':
            self.pos.z += step
        elif key == 'e':
            self.pos.z -= step
        elif key == 'Up':
            self.pos.z += step
        elif key == 'Down':
            self.pos.z -= step
        elif key == 'Left':
            self.pos.y += step
        elif key == 'Right':
            self.pos.y -= step

        self.arm_pub.publish(self.pos)
        self.update_param_label()

    def update_param_label(self):
        self.param_label.config(text=f"机械臂位置: x={self.pos.x} y={self.pos.y} z={self.pos.z}")

if __name__ == '__main__':
    try:
        SecurityRobotGUI()
    except rospy.ROSInterruptException:
        pass
