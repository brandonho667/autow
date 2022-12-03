import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from .autow import Autow
import cv2
import cv2.aruco as aruco
import numpy as np
import depthai as dai
import time
import yaml
import math
import signal

class AutowControl(Node):
    def __init__(self):
        super().__init__('autow')
        signal.signal(signal.SIGTERM, self.e_stop)
        signal.signal(signal.SIGINT, self.e_stop)
        # Create pipeline
        target_aruco_id = 13
        self.pipeline = dai.Pipeline()
        self.cam = self.pipeline.create(dai.node.ColorCamera)
        # self.cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
        self.cam.setPreviewSize(640, 480)
        self.cam.setInterleaved(False)

        self.xout = self.pipeline.create(dai.node.XLinkOut)
        self.xout.setStreamName("rgb")
        self.cam.preview.link(self.xout.input)
        self.target_id = target_aruco_id
        self.hitch_ar = 0.065
        self.hitch_cam = 0.146

        self.steer_buff = []
        self.autow_sub = self.create_subscription(
            String, 'autow_run', self.autow_callback, 10)
        self.autow_status = self.create_publisher(String, 'autow_status', 10)
        self.driver_pub = self.create_publisher(Float64MultiArray, 'driver', 10)
        self.calibration = yaml.safe_load(open('share/config/calibration.yaml'))



    def autow_callback(self, msg):
        if msg.data == "start":
            self.run()
            self.autow_status.publish(String(data="done"))
        elif msg.data == "stop":
            self.stopped = True

    def run(self):
        arucoDict = aruco.Dictionary_get(aruco.DICT_6X6_50)
        arucoParams = aruco.DetectorParameters_create()
        mtx, dist = np.array(self.calibration['camera_matrix']), np.array(self.calibration['dist_coeff'])
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
                # frame = cv2.flip(frame, -1)

                # find center of ar tag
                (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict,
                                                                   parameters=arucoParams)
                if len(corners) == 0:
                    self.driver_pub.publish(Float64MultiArray(data=[-1, 0]))
                    continue
                ids = ids.flatten()
                if self.target_id not in ids:
                    self.driver_pub.publish(Float64MultiArray(data=[-1, 0]))
                    continue
                idx = np.where(ids == self.target_id)[0][0]
                target_corners = corners[idx][0].reshape((4, 2))
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners, 0.05, mtx, dist)
                theta = (rvecs[0, 0, 0]/math.pi*rvecs[0, 0, 2])
                # ts = (rvecs[0, 0, 0],rvecs[0,0,1],rvecs[0, 0, 2])
                # print(f"theta: {theta}, x: {-tvecs[0,0,0]}, z: {tvecs[0, 0, 2]}")
                target_center = np.mean(target_corners, axis=0)
                target_height = abs(target_corners[2, 1]-target_corners[0, 1])
                # print(f"target @ {target_center} with height {target_height/frame.shape[0]}")
                if target_height/frame.shape[0] >= 0.26:
                    print("target reached")
                    self.driver_pub.publish(Float64MultiArray(data=[-1, -0.05]))
                    time.sleep(0.1)
                    print("hitching")
                    self.driver_pub.publish(Float64MultiArray(data=[-1, 0]))
                    time.sleep(1)
                    print("hitched")
                    break
                # self.steer_buff.append(
                #     self.calc_angle(frame.shape[1], target_center))
                self.steer_buff.append(self.calc_angle_hitch(
                    self.hitch_ar, self.hitch_cam, -tvecs[0,0,0], tvecs[0, 0, 2], theta))
                if len(self.steer_buff) >= 5:
                    # print(f"time elapsed: {time.perf_counter()-self.init_time}")
                    ave_steer = np.average(
                        self.steer_buff, weights=np.linspace(0, 1, len(self.steer_buff)))*1.15
                    print(f"ratio {target_height/frame.shape[0]}")
                    throttle = -(0.15 - abs(ave_steer-0.5)/15)*(1-target_height/frame.shape[0]*2)
                    print(f"steer: {ave_steer}, throttle: {throttle}")
                    self.driver_pub.publish(Float64MultiArray(data=[ave_steer, throttle]))
                    self.steer_buff = []

            device.close()

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
        theta_o = np.arctan(h_ar*math.sin(theta_hd)/(d-h_ar*math.cos(theta_hd)))*x/abs(x)
        theta_s = (theta_a-theta_o)*3
        # print(theta_s)
        return (theta_s+math.pi/2)/math.pi

    # calc steering for given center point
    def calc_angle(self, width, center_pt):
        x, y = center_pt
        print(x/width)
        return 1-x/width

    def e_stop(self, signal, frame):
        self.stopped = True

def main(args=None):
    rclpy.init(args=args)
    node = AutowControl()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
