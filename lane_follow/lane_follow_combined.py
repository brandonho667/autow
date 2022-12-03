from utils import vesc
import cv2
import numpy as np
from utils.lane_detect import *
import depthai as dai
import signal
import numpy as np
from lane_find import lane_steer, center_steer


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

        self.steer_buff = []

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
                frame = cv2.flip(frame, -1)

                ls = lane_steer(frame)
                cs = center_steer(frame)

                if ls is None and cs is None:
                    print(' No lane found -- yikes!')
                    continue

                if ls is None:
                    steer = cs
                elif cs is None:
                    steer = ls
                else:
                    steer = (0.9*ls + 0.1*cs)
                
                self.steer_buff.append(steer)
                if len(self.steer_buff) >= 5:
                    ave_steer = np.average(
                        self.steer_buff, weights=np.linspace(0, 1, len(self.steer_buff)))
                    smooth = ave_steer
                    if ave_steer < 0.3:
                        smooth = 0.3
                    elif ave_steer > 0.7:
                        smooth = 0.7
                    else:
                        smooth = ave_steer
                    self.vesc.run(smooth, 0.15 - abs(smooth-0.5)/10)
                    self.steer_buff = []
                # black_lines = display_lines(copy, averaged_lines)
                # # taking wighted sum of original image and lane lines image
                # lanes = cv2.addWeighted(copy, 0.8, black_lines, 1, 1)
                # cv2.circle(lanes, (int(intersection[0]), int(
                #     intersection[1])), 50, (0, 0, 255), -1)
                # show_image('lanes', lanes)
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