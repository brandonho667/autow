import inputs
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
import signal

MAX_JOYSTICK = 32767


class Controller(Node):
    def __init__(self):
        super().__init__('driver')
        signal.signal(signal.SIGTERM, self.e_stop)
        signal.signal(signal.SIGINT, self.e_stop)
        pads = inputs.devices.gamepads
        if len(pads) == 0:
            raise Exception("Couldn't find any Gamepads!")
        self.create_timer(0, self.controller_callback)
        self.autow_pub = self.create_publisher(String, 'autow', 10)
        self.autow_sub = self.create_subscription(
            String, 'autow_status', self.autow_callback, 10)
        self.driver_pub = self.create_publisher(Float64MultiArray, 'drive', 10)
        self.move = {'steer': 0.5, 'throttle': 0, 'autow_run': False}

    def autow_callback(self, msg):
        if msg.data == "done":
            print("Autow done")
            self.move['autow_run'] = False

    def controller_callback(self):
        events = inputs.get_gamepad()
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
            elif event.code == "BTN_NORTH" and not self.move['autow_run'] and event.state == 1:
                print("Start Pressed, running autow")
                self.move['autow_run'] = True
                self.autow_pub.publish(String(data="start"))
            elif event.code == "BTN_SOUTH" and self.move['autow_run'] and event.state == 1:
                print("Stop Pressed, stopping autow")
                self.move['autow_run'] = False
                self.autow_pub.publish(String(data="stop"))
        if not self.move['autow_run']:
            self.driver_pub.publish(Float64MultiArray(data=[
                self.move['steer'], self.move['throttle']]))
    
    def e_stop(self, signal, frame):
        self.stop = True


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
