#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import LaserScan
import numpy as np

class PoseToTFNode(Node):
    def __init__(self):
        super().__init__('pose_broadcaster_node')

        # Suscripciones
        self.pose_sub = self.create_subscription(Pose, '/robot1/pose', self.pose_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Publicadores
        self.tf_broadcaster = TransformBroadcaster(self)
        self.scan_pub = self.create_publisher(LaserScan, '/scan2', 10)

        # Variables internas
        self.latest_pose = None
        self.latest_scan = None

        # Timer (10 Hz)
        self.timer_period = 1.0 / 10.0
        self.timer = self.create_timer(self.timer_period, self.broadcast_tf)

        self.get_logger().info("✅ Nodo PoseToTF inicializado correctamente.")

    def pose_callback(self, msg: Pose):
        self.latest_pose = msg

    def scan_callback(self, msg: LaserScan):
        """Guarda el último scan y actualiza su frame"""
        msg.header.frame_id = "laser_frame"
        self.latest_scan = msg

    def broadcast_tf(self):
        if self.latest_pose is None or self.latest_scan is None:
            return

        # === Tiempo actual ===
        now = self.get_clock().now().to_msg()
        msg = self.latest_pose

        # --- odom2 -> base_link2 ---
        t1 = TransformStamped()
        t1.header.stamp = now
        t1.header.frame_id = 'odom2'
        t1.child_frame_id = 'base_link2'
        t1.transform.translation.x = msg.position.x
        t1.transform.translation.y = msg.position.y
        t1.transform.translation.z = msg.position.z
        t1.transform.rotation = msg.orientation
        self.tf_broadcaster.sendTransform(t1)

        # --- base_link2 -> laser_frame ---
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = 'base_link2'
        t2.child_frame_id = 'laser_frame'
        t2.transform.translation.x = 0.2
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.15
        q = quaternion_from_euler(0.0, 0.0, np.pi)
        t2.transform.rotation.x, t2.transform.rotation.y, t2.transform.rotation.z, t2.transform.rotation.w = q
        self.tf_broadcaster.sendTransform(t2)

        # --- Publicar scan sincronizado ---
        scan_copy = LaserScan()
        scan_copy = self.latest_scan
        scan_copy.header.stamp = now        # ⬅️ MISMO tiempo que TF
        scan_copy.header.frame_id = "laser_frame"
        self.scan_pub.publish(scan_copy)


def main(args=None):
    rclpy.init(args=args)
    node = PoseToTFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
