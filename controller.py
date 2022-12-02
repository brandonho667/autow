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
        self.stop = False

    def run(self):

        while not self.stop:
            events = inputs.get_gamepad()
            move = {'steer': 0, 'throttle': 0, 'stop_autow': False}
            for event in events:
                if event.code == "ABS_X":
                    steer = (event.state+MAX_JOYSTICK) / (2*MAX_JOYSTICK)
                    if steer > 0.001:
                        move['steer'] = steer
                elif event.code == "ABS_RY":
                    throttle = (-event.state + MAX_JOYSTICK) / MAX_JOYSTICK
                    if throttle > 0.001:
                        move['throttle'] = throttle
                elif event.code == "BTN_NORTH":
                    print("Start Pressed, running autow")
                    self.autow.run()

            if not move['stop_autow']:
                self.vesc.set_throttle(move['throttle'])
                self.vesc.set_steering(move['steer'])

        def e_stop(self):
            self.autow.stop()
            self.stop = True


def main():
    node = Driver()
    signal.signal(signal.SIGTERM, node.e_stop)
    signal.signal(signal.SIGINT, node.e_stop)
    node.run()


if __name__ == '__main__':
    main()
