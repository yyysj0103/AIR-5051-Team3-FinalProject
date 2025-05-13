#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import threading
import tkinter as tk
from tkinter import messagebox
import os
import time
from tkinter import ttk
class HumanDetector:
    def __init__(self):
        rospy.init_node('yolo_human_detector')
        self.bridge = CvBridge()

        self.model = YOLO('/home/spark/spark_noetic/src/spark_security_guard/models/yolov8n.pt')

        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.person_pub = rospy.Publisher("/detected_person", String, queue_size=1)

        self.alert_active = False     # 当前是否有警告
        self.last_detection_time = time.time()
        self.alert_window = None
        
    def show_custom_warning():
    # 创建临时窗口
    	root = tk.Tk()
    	root.withdraw()  # 隐藏主窗口
    
    # 创建自定义弹窗
    	top = tk.Toplevel(root)
    	top.title("警告")
    
    # 设置字体（字体名, 大小, 样式）
    	custom_font = ("Microsoft YaHei", 14, "bold")  
    
    # 添加内容
    	label = ttk.Label(top, text="自定义警告内容！", font=custom_font, foreground="red")
    	label.pack(padx=20, pady=10)
    
    # 添加确认按钮
    	button = ttk.Button(top, text="知道了", command=top.destroy)
    	button.pack(pady=5)

    # 关键修改：添加10秒自动关闭
    	top.after(10000, top.destroy)  # 10000毫秒 = 10秒
    
    # 弹窗置顶
    	top.grab_set()
    	root.wait_window(top)  # 等待弹窗关闭
    	root.destroy()


    def play_warning_sound(self):
        # 播放蜂鸣声（Linux自带命令）
        for _ in range(3):
            os.system('play -nq -t alsa synth 0.2 sine 880')  # 0.2秒 880Hz
            time.sleep(0.3)

    def open_warning_window(self):
        if self.alert_window is None:
            self.root = tk.Tk()
            self.root.title("警告")
            self.root.geometry("300x150")
            label = tk.Label(self.root, text="检测到可疑人员！", font=("Arial", 20), fg="red")
            label.pack(expand=True)
            threading.Thread(target=self.play_warning_sound).start()
            self.alert_window = self.root
            threading.Thread(target=self.root.mainloop).start()

    def close_warning_window(self):
        if self.alert_window:
            try:
                self.root.destroy()
            except:
                pass
            self.alert_window = None

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr("cv_bridge error: {}".format(e))
            return

        results = self.model(frame)[0]
        person_detected = False

        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            if self.model.names[cls_id] == 'person' and conf > 0.5:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

                self.person_pub.publish(f"{cx},{cy}")

                cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
                cv2.putText(frame, f"Person {conf:.2f}", (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

                person_detected = True

        # 根据检测结果弹出或关闭警告窗口
        if person_detected:
            self.last_detection_time = time.time()
            if not self.alert_active:
                self.alert_active = True
                self.open_warning_window()
                #self.show_custom_warning()
        else:
            if self.alert_active and (time.time() - self.last_detection_time > 2):  # 超过2秒没人
                self.alert_active = False
                self.close_warning_window()

        cv2.namedWindow("YOLO Human Detection", cv2.WINDOW_NORMAL)
        cv2.imshow("YOLO Human Detection", frame)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        HumanDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

