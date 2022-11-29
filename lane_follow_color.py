from utils import vesc
import cv2
import numpy as np
from utils.lane_detect import *
import depthai as dai
import signal
import numpy as np
from Blue_Green_Detect import blue_green_steer_direction


class LaneFollower:
    def __init__(self):
        self.stopped = False

        self.vesc = vesc.VESC("/dev/ttyACM0")
        self.vesc.run(0.5, 0)

        # Create pipeline
        self.pipeline = dai.Pipeline()
        self.cam = self.pipeline.create(dai.node.ColorCamera)
        # self.cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
        self.cam.setPreviewSize(640, 480)
        self.cam.setInterleaved(False)

        self.xout = self.pipeline.create(dai.node.XLinkOut)
        self.xout.setStreamName("rgb")
        self.cam.preview.link(self.xout.input)

        self.steer = 0.5

    def run(self):
        # Connect to device and start pipeline
        with dai.Device(self.pipeline) as device:

            qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            print("UVC running")
            while not self.stopped:
                frame = qRgb.get()

                if frame is None:
                    print(' No captured frame -- yikes!')
                    continue
                frame = frame.getCvFrame()
                curr_steer = blue_green_steer_direction(frame)
                if self.steer != curr_steer:
                    self.steer = curr_steer
                    self.vesc.run(self.steer, 0.2 - abs(self.steer-0.5)/5)
                    print("Steer: ", self.steer)
            device.close()
            self.vesc.run(0.5, 0)
            self.vesc.close()

    def stop(self, signal, frame):
        print("gracefully stopping...")
        self.stopped = True


def main():
    lf = LaneFollower()
    signal.signal(signal.SIGTERM, lf.stop)
    signal.signal(signal.SIGINT, lf.stop)
    lf.run()


if __name__ == "__main__":
    main()