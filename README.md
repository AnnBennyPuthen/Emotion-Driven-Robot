🤖 Emotion-Driven Robot

Emotion-Driven Robot: AI + ROS-based simulation that reacts to human emotions in real time.


📖 Overview

This project simulates a robot that detects and reacts to human emotions using computer vision, deep learning, and the Robot Operating System (ROS2). A webcam captures facial expressions in real time, and the AI module predicts emotions. The recognized emotion is then translated into robot movement inside the Gazebo simulation environment.


🏗️ System Architecture

Webcam Input → Captures face in real time

Emotion Detector Node → Detects facial emotions and publishes to /emotion

Behavior Control Node → Subscribes to /emotion, decides robot movement, publishes to /cmd_vel

Gazebo Robot (TurtleBot3) → Executes movement commands in the simulated world


🛠️ Technologies & Tools

Programming: Python

AI/ML: TensorFlow, Keras, FER2013, DeepFace

Robotics: ROS2 (rclpy, geometry_msgs.msg.Twist), Gazebo

GUI: PyQt5 (live video, emotion labels, emoji display)

Other: Threading for ROS node + GUI concurrency


🚀 Features

Real-time emotion recognition (happy, sad, angry).

Emotion-based robot control in simulation.

GUI with live video, emotion labels, and emoji feedback.

~93% accuracy, <2s response time.
