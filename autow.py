from utils import vesc
import cv2
import cv2.aruco as aruco
import numpy as np
from utils.lane_detect import *
import depthai as dai
import signal
import numpy as np


class Autow:
    def __init__(self, target_aruco_id=0):
        self.stopped = False

        self.vesc = vesc.VESC("/dev/ttyACM0")

        # Create pipeline
        self.pipeline = dai.Pipeline()
        self.cam = self.pipeline.create(dai.node.ColorCamera)
        # self.cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
        self.cam.setPreviewSize(640, 480)
        self.cam.setInterleaved(False)

        self.xout = self.pipeline.create(dai.node.XLinkOut)
        self.xout.setStreamName("rgb")
        self.cam.preview.link(self.xout.input)
        self.target_id = target_aruco_id

        self.steer_buff = []

    def run(self):
        arucoDict = aruco.Dictionary_get(aruco.DICT_6X6_50)
        arucoParams = aruco.DetectorParameters_create()
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
                # frame = cv2.flip(frame, -1)

                # find center of ar tag
                (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict,
                                                                   parameters=arucoParams)
                if len(corners) == 0:
                    self.vesc.set_throttle(0)
                    continue
                ids = ids.flatten()
                if self.target_id not in ids:
                    self.vesc.set_throttle(0)
                    continue
                idx = np.where(ids == self.target_id)[0][0]
                target_corners = corners[idx][0].reshape((4, 2))
                # rvec, tvec, _ = aruco.estimatePoseSingleMarkers(target_corners, 0.05, mtx, dist)
                target_center = np.mean(target_corners, axis=0)
                target_height = abs(target_corners[2, 1]-target_corners[0, 1])
                # print(f"target @ {target_center} with height {target_height/frame.shape[0]}")
                print(f"target angle ")
                self.steer_buff.append(
                    self.calc_angle(frame.shape[1], target_center))
                if len(self.steer_buff) >= 5:
                    ave_steer = np.average(
                        self.steer_buff, weights=np.linspace(0, 1, len(self.steer_buff)))
                    self.vesc.run(ave_steer, -(0.2 - abs(ave_steer-0.5)/5)*(1-target_height/frame.shape[0]))
                    self.steer_buff = []

            device.close()
            self.vesc.run(0.5, 0)
            self.vesc.close()

    def get_area(self, x, y):
        return 0.5*np.abs(np.dot(x, np.roll(y, 1))-np.dot(y, np.roll(x, 1)))

    # calc steering for given center point
    def calc_angle(self, width, center_pt):
        x, y = center_pt
        print(x/width)
        return 1-x/width

    def stop(self, signal, frame):
        print("gracefully stopping...")
        self.stopped = True


def main():
    lf = Autow()
    signal.signal(signal.SIGTERM, lf.stop)
    signal.signal(signal.SIGINT, lf.stop)
    lf.run()


if __name__ == "__main__":
    main()
