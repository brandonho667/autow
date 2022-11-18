from utils import vesc
import cv2
import numpy as np
from utils.lane_detect import *
import depthai as dai
import signal
import numpy as np


class LaneFollower:
    def __init__(self):
        self.stopped = False

        self.vesc = vesc.VESC("/dev/ttyACM0")
        self.vesc.run(0.5,0)

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
                frame = cv2.cvtColor(frame.getCvFrame(), cv2.COLOR_BGR2RGB)
                frame = cv2.flip(frame, -1)

                copy = np.copy(frame)
                edges = canny(gauss(hsv(copy)))
                isolated = region(edges)
                # show_image('edges', edges)
                # show_image('isolated', isolated)

                # DRAWING LINES: (order of params) --> region of interest, bin size (P, theta), min intersections needed, placeholder array,
                lines = cv2.HoughLinesP(
                    isolated, 2, np.pi/180, 100, np.array([]), minLineLength=40, maxLineGap=5)
                if lines is None or len(lines) < 2:
                    continue
                # print(f"lines: {lines}, {lines.shape}")
                averaged_lines = average(copy, lines)
                # print(
                #     f"averaged_lines: {averaged_lines}, {averaged_lines.shape}")

                # print(f"pt 1 type {type(averaged_lines[0][0])}")
                intersection = self.find_intersection(
                    averaged_lines[0], averaged_lines[1])
                if intersection == -1:
                    intersection = (copy.shape[1]//2, copy.shape[0]//2)
                # print(f"intersection @ {intersection}")
                self.steer_buff.append(
                    self.calc_angle(copy.shape[1], intersection))
                if len(self.steer_buff) >= 7:
                    ave_steer = np.average(self.steer_buff, weights=np.linspace(0,1,len(self.steer_buff)))
                    self.vesc.run(ave_steer, 0.2 - abs(ave_steer-0.5)/5)
                    self.steer_buff = []
                # black_lines = display_lines(copy, averaged_lines)
                # # taking wighted sum of original image and lane lines image
                # lanes = cv2.addWeighted(copy, 0.8, black_lines, 1, 1)
                # cv2.circle(lanes, (int(intersection[0]), int(
                #     intersection[1])), 50, (0, 0, 255), -1)
                # show_image('lanes', lanes)
            device.close()
            self.vesc.run(0.5,0)
            self.vesc.close()

    def find_intersection(self, line1, line2):
        x1, y1, x2, y2 = line1
        x3, y3, x4, y4 = line2
        a1 = y2-y1
        b1 = x1-x2
        c1 = a1*(x1) + b1*(y1)

        # Line CD represented as a2x + b2y = c2
        a2 = y4-y3
        b2 = x3-x4
        c2 = a2*(x3) + b2*(y3)

        determinant = a1*b2 - a2*b1
        if (determinant == 0):
            # The lines are parallel. This is simplified
            # by returning a pair of FLT_MAX
            return -1
        else:
            x = (b2*c1 - b1*c2)/determinant
            y = (a1*c2 - a2*c1)/determinant
            return x, y

    # calc steering for given center point
    def calc_angle(self, width, center_pt):
        x, y = center_pt
        print(x/width)
        return x/width

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
