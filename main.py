from utils.vesc import VESC
import inputs
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

MAX_JOYSTICK = 32767


class Driver(Node):
    def __init__(self):
        super().__init__('driver')
        pads = inputs.devices.gamepads
        if len(pads) == 0:
            raise Exception("Couldn't find any Gamepads!")
        self.vesc = VESC('/dev/ttyACM0')
        self.create_timer(0.1, self.controller_callback)
        self.autow_pub = self.create_publisher(String, 'autow', 10)

    def controller_callback(self):
        events = inputs.get_gamepad()
        move = {'steer': 0, 'throttle': 0, 'stop_autow': False}
        for event in events:
            if event.code == "ABS_X":
                print(f"steer: {event.state/MAX_JOYSTICK}")
                steer = (event.state+MAX_JOYSTICK) / (2*MAX_JOYSTICK)
                if abs(steer-0.5) > 0.005:
                    self.move['steer'] = steer
                else:
                    self.move['steer'] = 0.5
            elif event.code == "ABS_RY":
                print(f"throttle: {event.state/MAX_JOYSTICK}")
                throttle = -event.state / MAX_JOYSTICK
                if abs(throttle) > 0.005:
                    self.move['throttle'] = throttle*0.2
                else:
                    self.move['throttle'] = 0
            elif event.code == "BTN_NORTH":
                print("Start Pressed, running autow")
                self.autow_pub.publish(String(data="start"))
            elif event.code == "BTN_SOUTH":
                print("Stop Pressed, stopping autow")
                move['stop_autow'] = True
                self.autow_pub.publish(String(data="stop"))
        if not move['stop_autow']:
            print(f"Steer: {move['steer']}, Throttle: {move['throttle']}")
            self.vesc.set_throttle(move['throttle'])
            self.vesc.set_steer(move['steer'])


def main(args=None):
    rclpy.init(args=args)
    node = Driver()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
