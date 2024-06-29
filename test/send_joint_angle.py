import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

def publish_joint_angles():
    rclpy.init()
    node = rclpy.create_node("joint_angle_publisher")
    publisher = node.create_publisher(Float64MultiArray, "/joint_angles", 10)

    msg = Float64MultiArray()
    msg.data = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]  # 예시로 임의의 관절 각도 설정

    while rclpy.ok():  # ROS2가 정상적으로 실행 중인 동안 반복
        node.get_logger().info("Publishing: %s" % msg.data)
        publisher.publish(msg)
        print("ASD")
        rclpy.spin_once(node)
        time.sleep(1)  # 1초마다 반복

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    publish_joint_angles()
