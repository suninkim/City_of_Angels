import sys
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk

class ROS2Node(Node):
    def __init__(self):
        super().__init__('gui_publisher')
        self.joint_pub = self.create_publisher(Float64MultiArray, '/joint_angles', 10)
        self.os_pub = self.create_publisher(Float64MultiArray, '/os_cmd', 10)
        self.gripper_pub = self.create_publisher(Bool, '/gripper_control', 10)
        self.image_sub = self.create_subscription(CompressedImage, '/compressed_image', self.image_callback, 10)
        self.image_data = None

    def publish_joint_angles(self, angles):
        msg = Float64MultiArray()
        msg.data = angles
        self.joint_pub.publish(msg)

    def publish_os_cmd(self, cmd):
        msg = Float64MultiArray()
        msg.data = cmd
        self.os_pub.publish(msg)

    def publish_gripper(self, open):
        msg = Bool()
        msg.data = open
        self.gripper_pub.publish(msg)

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.image_data = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

class MainWindow(tk.Tk):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.title('Robot Control Panel')
        self.geometry('800x600')
        self.create_widgets()
        self.streaming = False

    def create_widgets(self):
        self.joint_inputs = [tk.Entry(self) for _ in range(6)]
        self.os_inputs = [tk.Entry(self) for _ in range(6)]
        self.image_label = tk.Label(self)

        for i, joint_input in enumerate(self.joint_inputs):
            tk.Label(self, text=f'j{i+1}').grid(row=i, column=0)
            joint_input.grid(row=i, column=1)

        send_joint_btn = tk.Button(self, text='send joint angle', command=self.send_joint_angles)
        send_joint_btn.grid(row=0, column=2, columnspan=2)

        for i, (label, os_input) in enumerate(zip(['x', 'y', 'z', 'r', 'p', 'y'], self.os_inputs)):
            tk.Label(self, text=label).grid(row=i, column=4)
            os_input.grid(row=i, column=5)

        send_os_btn = tk.Button(self, text='send os', command=self.send_os_cmd)
        send_os_btn.grid(row=0, column=6, columnspan=2)

        close_gripper_btn = tk.Button(self, text='close gripper', command=self.close_gripper)
        close_gripper_btn.grid(row=2, column=6, columnspan=2)

        open_gripper_btn = tk.Button(self, text='open gripper', command=self.open_gripper)
        open_gripper_btn.grid(row=3, column=6, columnspan=2)

        show_image_btn = tk.Button(self, text='show image', command=self.start_streaming)
        show_image_btn.grid(row=4, column=6, columnspan=2)

        self.image_label.grid(row=6, column=0, columnspan=8)

    def send_joint_angles(self):
        angles = [float(input.get()) for input in self.joint_inputs]
        self.ros_node.publish_joint_angles(angles)

    def send_os_cmd(self):
        cmd = [float(input.get()) for input in self.os_inputs]
        self.ros_node.publish_os_cmd(cmd)

    def close_gripper(self):
        self.ros_node.publish_gripper(False)

    def open_gripper(self):
        self.ros_node.publish_gripper(True)

    def start_streaming(self):
        self.streaming = True
        self.update_image()

    def update_image(self):
        if self.streaming and self.ros_node.image_data is not None:
            b, g, r = cv2.split(self.ros_node.image_data)
            img = cv2.merge((r, g, b))
            img = Image.fromarray(img)
            imgtk = ImageTk.PhotoImage(image=img)
            self.image_label.configure(image=imgtk)
            self.image_label.image = imgtk
        if self.streaming:
            self.after(100, self.update_image)  # Update every 100 ms

def ros_spin(ros_node):
    rclpy.spin(ros_node)

def main(args=None):
    rclpy.init(args=args)
    ros_node = ROS2Node()

    threading.Thread(target=ros_spin, args=(ros_node,), daemon=True).start()

    app = MainWindow(ros_node)
    app.mainloop()

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
