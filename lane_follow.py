from utils import vesc
import cv2
import numpy as np
from utils.lane_detect import *
import depthai as dai
import signal


class LaneFollower:
    def __init__(self):
        self.stopped = False

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

                copy = np.copy(frame)
                edges = canny(copy)
                isolated = region(edges)
                show_image('edges', edges)
                show_image('isolated', isolated)

                # DRAWING LINES: (order of params) --> region of interest, bin size (P, theta), min intersections needed, placeholder array,
                lines = cv2.HoughLinesP(
                    isolated, 2, np.pi/180, 100, np.array([]), minLineLength=40, maxLineGap=5)
                if lines is None:
                    continue
                print(f"lines: {lines}, {lines.shape}")
                averaged_lines = average(copy, lines)
                print(
                    f"averaged_lines: {averaged_lines}, {averaged_lines.shape}")
                black_lines = display_lines(copy, averaged_lines)
                # taking wighted sum of original image and lane lines image
                lanes = cv2.addWeighted(copy, 0.8, black_lines, 1, 1)
                print(lanes)
                show_image('lanes', lanes)
            device.close()

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
