from .utils.vesc import VESC
import inputs
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import signal


class Driver(Node):
    def __init__(self):
        super().__init__('driver')
        self.vesc = VESC('/dev/ttyACM0')
        self.drive_sub = self.create_subscription(
            Float64MultiArray, 'drive', self.driver_callback, 10)


    def driver_callback(self, msg):
        if msg.data[0] == -1:
            self.vesc.set_throttle(msg.data[1])
        else:
            self.vesc.run(msg.data[0], msg.data[1])
        
    
    def stop(self):
        self.vesc.close()
        self.stop = True


def main(args=None):
    rclpy.init(args=args)
    node = Driver()
    rclpy.spin(node)
    node.stop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
