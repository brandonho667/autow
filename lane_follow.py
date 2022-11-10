from utils import vesc
import cv2
import numpy as np
from utils.lane_detect import *
import depthai as dai
import signal


class LaneFollower:
    def __init__(self):
        self.stopped = False

        # self.vesc = vesc.VESC("/dev/ttyACM0")

        # Create pipeline
        self.pipeline = dai.Pipeline()
        self.cam = self.pipeline.create(dai.node.ColorCamera)
        # self.cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
        self.cam.setPreviewSize(640, 480)
        self.cam.setInterleaved(False)

        self.xout = self.pipeline.create(dai.node.XLinkOut)
        self.xout.setStreamName("rgb")
        self.cam.preview.link(self.xout.input)

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
                edges = canny(gauss(copy))
                isolated = region(edges)
                show_image('edges', edges)
                show_image('isolated', isolated)

                # DRAWING LINES: (order of params) --> region of interest, bin size (P, theta), min intersections needed, placeholder array,
                lines = cv2.HoughLinesP(
                    isolated, 2, np.pi/180, 100, np.array([]), minLineLength=40, maxLineGap=5)
                if lines is None or len(lines) < 2:
                    continue
                print(f"lines: {lines}, {lines.shape}")
                averaged_lines = average(copy, lines)
                print(
                    f"averaged_lines: {averaged_lines}, {averaged_lines.shape}")

                print(f"pt 1 type {type(averaged_lines[0][0])}")
                intersection = self.find_intersection(
                    averaged_lines[0], averaged_lines[1])
                print(f"intersection @ {intersection}")
                # self.vesc.run(self.calc_angle(
                #     copy.shape[1], lines[0], lines[1]), 0.1)
                black_lines = display_lines(copy, averaged_lines)
                # taking wighted sum of original image and lane lines image
                lanes = cv2.addWeighted(copy, 0.8, black_lines, 1, 1)
                cv2.circle(lanes, (int(intersection[0]), int(
                    intersection[1])), 5, (0, 0, 255), -1)
                show_image('lanes', lanes)
            device.close()

    def find_intersection(self, line1, line2):
        x1, y1, x2, y2 = line1
        x3, y3, x4, y4 = line2
        px = x2-x1
        py = y2-y1
        dAB = px*px + py*py
        u = ((x4 - x3) * px + (y4 - y3) * py) / float(dAB)
        x = x1 + u * px
        y = y1 + u * py
        return x, y

    # find the intersection point of two lines for angle
    def calc_angle(self, width, line1, line2, alpha=0.5):
        x, y = self.find_intersection(line1, line2)
        return alpha*(x-width)

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
