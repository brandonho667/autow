from utils.vesc import VESC
from autow import Autow
import inputs
import signal

MAX_JOYSTICK = 32767


class Driver():
    def __init__(self):
        pads = inputs.devices.gamepads
        if len(pads) == 0:
            raise Exception("Couldn't find any Gamepads!")
        self.vesc = VESC('/dev/ttyACM0')
        self.autow = Autow(target_aruco_id=13)
        self.move = {'steer': 0.5, 'throttle': 0, 'stop_autow': False}

        self.stop = False

    def run(self):

        while not self.stop:
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
                elif event.code == "BTN_NORTH":
                    print("Start Pressed, running autow")
                    self.autow.run()

            self.vesc.set_throttle(self.move['throttle'])
            self.vesc.set_steer(self.move['steer'])

    def e_stop(self, signal, frame):
        self.autow.e_stop()
        self.vesc.close()
        self.stop = True


def main():
    node = Driver()
    signal.signal(signal.SIGTERM, node.e_stop)
    signal.signal(signal.SIGINT, node.e_stop)
    node.run()


if __name__ == '__main__':
    main()
