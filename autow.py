from utils import vesc
import cv2
import cv2.aruco as aruco
import numpy as np
from utils.lane_detect import *
import depthai as dai
import signal
import numpy as np
import yaml
import math
import time


class Autow:
    def __init__(self, target_aruco_id=0):
        self.stopped = False

        self.vesc = vesc.VESC("/dev/ttyACM0")

        # Create pipeline
        pipeline = dai.Pipeline()
        cam = pipeline.create(dai.node.ColorCamera)
        # self.cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
        cam.setPreviewSize(640, 480)
        cam.setInterleaved(False)

        xout = self.pipeline.create(dai.node.XLinkOut)
        xout.setStreamName("rgb")
        cam.preview.link(self.xout.input)

        self.arucoDict = aruco.Dictionary_get(aruco.DICT_6X6_50)
        self.arucoParams = aruco.DetectorParameters_create()
        mtx, dist = yaml.safe_load(open('calibration.yaml'))['camera_matrix'], yaml.safe_load(
            open('calibration.yaml'))['dist_coeff']
        self.mtx, self.dist = np.array(mtx), np.array(dist)
        self.camera = dai.Device(self.pipeline)
        print("UVC running")

        self.target_id = target_aruco_id
        self.hitch_ar = 0.065
        self.hitch_cam = 0.146

        self.steer_buff = []
        self.init_time = time.perf_counter()

    def run(self):
        self.stopped = False
        self.qRgb = self.camera.getOutputQueue(
            name="rgb", maxSize=4, blocking=False)
        # Connect to device and start pipeline
        while not self.stopped:
            frame = self.qRgb.get()

            if frame is None:
                print(' No captured frame -- yikes!')
                continue
            frame = frame.getCvFrame()
            # frame = cv2.flip(frame, -1)

            # find center of ar tag
            (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, self.arucoDict,
                                                               parameters=self.arucoParams)
            if len(corners) == 0:
                self.vesc.set_throttle(0)
                continue
            ids = ids.flatten()
            if self.target_id not in ids:
                self.vesc.set_throttle(0)
                continue
            idx = np.where(ids == self.target_id)[0][0]
            target_corners = corners[idx][0].reshape((4, 2))
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, 0.05, self.mtx, self.dist)
            theta = (rvecs[0, 0, 0]/math.pi*rvecs[0, 0, 2])
            # ts = (rvecs[0, 0, 0],rvecs[0,0,1],rvecs[0, 0, 2])
            # print(f"theta: {theta}, x: {-tvecs[0,0,0]}, z: {tvecs[0, 0, 2]}")
            target_center = np.mean(target_corners, axis=0)
            target_height = abs(target_corners[2, 1]-target_corners[0, 1])
            # print(f"target @ {target_center} with height {target_height/frame.shape[0]}")
            if target_height/frame.shape[0] >= 0.26:
                print("target reached")
                self.vesc.set_throttle(-0.05)
                time.sleep(0.1)
                print("hitching")
                self.vesc.set_throttle(0)
                time.sleep(1)
                print("hitched")
                self.vesc.run(0.5, 0.15)
                time.sleep(0.1)
                break
            # self.steer_buff.append(
            #     self.calc_angle(frame.shape[1], target_center))
            self.steer_buff.append(self.calc_angle_hitch(
                self.hitch_ar, self.hitch_cam, -tvecs[0, 0, 0], tvecs[0, 0, 2], theta))
            if len(self.steer_buff) >= 5:
                # print(f"time elapsed: {time.perf_counter()-self.init_time}")
                self.init_time = time.perf_counter()
                ave_steer = np.average(
                    self.steer_buff, weights=np.linspace(0, 1, len(self.steer_buff)))*1.15
                print(f"ratio {target_height/frame.shape[0]}")
                throttle = -(0.1 - abs(ave_steer-0.5)/15) * \
                    (1-target_height/frame.shape[0]*2)
                print(f"steer: {ave_steer}, throttle: {throttle}")
                self.vesc.run(ave_steer, throttle)
                self.steer_buff = []

            self.camera.close()
            self.vesc.run(0.5, 0)
            self.vesc.close()

    def get_area(self, x, y):
        return 0.5*np.abs(np.dot(x, np.roll(y, 1))-np.dot(y, np.roll(x, 1)))

    def calc_angle_hitch(self, h_ar, h_cam, x, z, theta):
        """
        Calculate steering angle for given hitch offset and target position
        Args:
            h_ar (float): hitch offset of ar tag
            h_cam (float): hitch offset of camera
            x (float): target x position
            z (float): target z position
            theta (float): target angle
        Returns:
            float: steering angle
        """
        z = z-h_cam-0.08
        theta_a = np.arctan(x/z)
        # print(f"theta_a: {theta_a}")
        d = math.sqrt(x**2 + z**2)
        theta_hd = np.arctan(z/x)-(math.pi/2-theta)
        theta_o = np.arctan(h_ar*math.sin(theta_hd) /
                            (d-h_ar*math.cos(theta_hd)))*x/abs(x)
        theta_s = (theta_a-theta_o)*3
        # print(theta_s)
        return (theta_s+math.pi/2)/math.pi

    # calc steering for given center point
    def calc_angle(self, width, center_pt):
        x, y = center_pt
        print(x/width)
        return 1-x/width

    def stop(self):
        print("Stopping")
        self.stopped = True

    def e_stop(self, signal, frame):
        print("gracefully stopping...")
        self.stopped = True


def main():
    lf = Autow(target_aruco_id=13)
    signal.signal(signal.SIGTERM, lf.e_stop)
    signal.signal(signal.SIGINT, lf.e_stop)
    lf.run()


if __name__ == "__main__":
    main()
