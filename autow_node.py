from utils.vesc import VESC
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from autow import Autow


class AutowControl(Node):
    def __init__(self):
        super().__init__('autow')
        self.vesc = VESC('/dev/ttyACM0')
        self.autow = Autow(target_aruco_id=13)
        self.autow_pub = self.create_subscription(
            String, 'autow', self.autow_callback, 10)

    def autow_callback(self, msg):
        if msg.data == "start":
            self.autow.run()
        elif msg.data == "stop":
            self.autow.stop()


def main(args=None):
    rclpy.init(args=args)
    node = AutowControl()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
