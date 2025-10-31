#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import threading

class CmdVelCtrlNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_ctrl_node_charlie')
        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel_ctrl', 10)
        self.timer = self.create_timer(0.1, self.publish_cmd)  # 10 Hz
        self.linear_input = 0.0
        self.angular_input = 0.0

        # Hilo aparte para lectura de teclado (no bloquea el timer)
        self.input_thread = threading.Thread(target=self.read_inputs, daemon=True)
        self.input_thread.start()

    def publish_cmd(self):
        msg = TwistStamped()
        # msg.header.frame_id = 'base_link2'
        # msg.header.stamp = 
        
        msg.twist.linear.x = self.remap(self.linear_input, -100, 100, -0.5, 0.5)
        msg.twist.angular.z = self.remap(self.angular_input, -100, 100, -0.5, 0.5)
        self.publisher_.publish(msg)

    def read_inputs(self):
        while rclpy.ok():
            try:
                lin = float(input("Ingrese velocidad lineal (-100 a 100): "))
                # ang = float(input("Ingrese velocidad angular (-100 a 100): "))
                self.linear_input = lin
                self.angular_input = 0
            except ValueError:
                self.get_logger().warn("❗ Entrada inválida. Use números.")

    @staticmethod
    def remap(value, in_min, in_max, out_min, out_max):
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelCtrlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
