import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import random

class EmotionBotNode(Node):
    def __init__(self):
        super().__init__('emotion_bot_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cap = cv2.VideoCapture(0)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def detect_emotion(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        avg = np.mean(gray)
        if avg > 130:
            return "happy"
        elif avg < 70:
            return "sad"
        return "neutral"

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Webcam not working")
            return

        emotion = self.detect_emotion(frame)
        self.get_logger().info(f"Emotion: {emotion}")
        twist = Twist()
        if emotion == "happy":
            twist.linear.x = 0.5
        elif emotion == "sad":
            twist.linear.x = -0.2
            twist.angular.z = 0.5
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = EmotionBotNode()
    rclpy.spin(node)
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()
