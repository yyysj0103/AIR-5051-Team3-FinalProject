#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class HumanDetector:
    def __init__(self):
        rospy.init_node('yolo_human_detector')
        self.bridge = CvBridge()

        # 加载 YOLOv8 模型（你可以替换为 yolov5）
        self.model = YOLO('/home/spark/spark_noetic/src/spark_security_guard/models/yolov8n.pt')

        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.person_pub = rospy.Publisher("/detected_person", String, queue_size=1)  # 示例

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr("cv_bridge error: {}".format(e))
            return

        results = self.model(frame)[0]
        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            if self.model.names[cls_id] == 'person' and conf > 0.5:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                # 发布检测到的人（你可以发布PoseStamped等）
                self.person_pub.publish(f"{cx},{cy}")
                cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
                cv2.putText(frame, f"Person {conf:.2f}", (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

        cv2.imshow("YOLO Human Detection", frame)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        HumanDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
