import sys
import cv2
import numpy as np
from PyQt5.QtWidgets import QApplication, QLabel, QPushButton, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QPixmap, QImage
from deepface import DeepFace
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading

EMOJI_MAP = {
    'happy': '😄',
    'sad': '😢',
    'angry': '😠',
    'surprise': '😲',
    'neutral': '😐',
    'fear': '😨',
    'disgust': '🤢'
}

class EmotionPublisher(Node):
    def __init__(self):
        super().__init__('emotion_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def move_robot(self, emotion):
        twist = Twist()
        if emotion == 'happy':
            twist.linear.x = 0.2
            twist.angular.z = 1.0
        elif emotion == 'sad':
            twist.linear.x = -0.1
        elif emotion == 'angry':
            twist.angular.z = 2.0
        elif emotion == 'surprise':
            twist.linear.x = 0.4
        elif emotion == 'neutral':
            twist.linear.x = 0.1
            twist.angular.z = 0.2
        elif emotion == 'fear':
            twist.linear.x = -0.4
        elif emotion == 'disgust':
            twist.angular.z = 1.5
        self.publisher.publish(twist)

class EmotionBotGUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.setWindowTitle("EmotionBot")
        self.setFixedSize(600, 500)

        self.ros_node = ros_node
        self.camera = cv2.VideoCapture(0)
        self.image_label = QLabel()
        self.emoji_label = QLabel("Emotion: None")

        self.detect_btn = QPushButton("Detect Emotion")
        self.detect_btn.clicked.connect(self.detect_emotion)

        layout = QVBoxLayout()
        layout.addWidget(self.image_label)
        layout.addWidget(self.emoji_label)
        layout.addWidget(self.detect_btn)
        self.setLayout(layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)

        self.emotion_timer = QTimer()
        self.emotion_timer.timeout.connect(self.detect_emotion)
        self.emotion_timer.start(7000)

    def update_frame(self):
        ret, frame = self.camera.read()
        if ret:
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.image_label.setPixmap(QPixmap.fromImage(qt_image))

    def detect_emotion(self):
        ret, frame = self.camera.read()
        if not ret:
            return

        try:
            result = DeepFace.analyze(frame, actions=['emotion'], enforce_detection=False)[0]
            emotion = result['dominant_emotion']
            emoji = EMOJI_MAP.get(emotion, '❓')
            self.emoji_label.setText(f"Detected: {emotion.capitalize()} {emoji}")
            self.ros_node.move_robot(emotion)
        except Exception as e:
            self.emoji_label.setText("Error detecting emotion")
            print("Detection error:", e)

    def closeEvent(self, event):
        self.camera.release()
        event.accept()

def ros_spin(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    ros_node = EmotionPublisher()

    ros_thread = threading.Thread(target=ros_spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    app = QApplication(sys.argv)
    gui = EmotionBotGUI(ros_node)
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
