#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_euler
import math

class PoseSimulator(Node):
    def __init__(self):
        super().__init__('pose_simulator_node_charlie')
        self.publisher_ = self.create_publisher(Pose, '/robot1/pose', 10)
        self.timer = self.create_timer(0.1, self.publish_pose)  # 10 Hz

        self.t = 0.0
        self.get_logger().info('✅ Nodo pose_simulator_node_charlie iniciado (publicando /robot1/pose)')

    def publish_pose(self):
        pose = Pose()

        # 🧠 Puedes elegir entre posición fija o en movimiento:
        # --- OPCIÓN 1: Fija ---
        # pose.position.x = 0.0
        # pose.position.y = 0.0
        # pose.position.z = 0.0

        # --- OPCIÓN 2: Movimiento circular ---
        pose.position.x = 1.0 * math.cos(self.t)
        pose.position.y = 1.0 * math.sin(self.t)
        pose.position.z = 0.0

        # Orientación (quaternion)
        q = quaternion_from_euler(0.0, 0.0, 0.0)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        self.publisher_.publish(pose)
        self.t += 0.1
        self.get_logger().info(
            f"Pose publicada: x={pose.position.x:.2f}, y={pose.position.y:.2f}, yaw=0.0"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PoseSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
