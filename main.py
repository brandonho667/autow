from utils.vesc import VESC
import signal
import sys
import rclpy
from rclpy.node import Node

class Driver(Node):
    def __init__(self):
        super().__init__('driver')
        self.create_timer(0.2, self.timer_callback)
    def timer_callback(self):
        self.get_logger().info("Hello ROS2")


def main(args=None):
    rclpy.init(args=args)
    node = Driver()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()

    v = VESC('/dev/ttyACM0')

    while True:
        v.run(0.10,0.2)