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
from playsound import playsound

class AutowControl(Node):
    def __init__(self):
        super().__init__('autow')
        # Create pipeline
        target_aruco_id = 13
        self.pipeline = dai.Pipeline()
        self.cam = self.pipeline.create(dai.node.ColorCamera)
        self.cam.setPreviewSize(640, 480)
        self.cam.setInterleaved(False)

        self.xout = self.pipeline.create(dai.node.XLinkOut)
        self.xout.setStreamName("rgb")
        self.cam.preview.link(self.xout.input)
        self.target_id = target_aruco_id
        self.hitch_ar = 0.095
        self.hitch_cam = 0.11

        self.steer_buff = []
        
        calibration = yaml.safe_load(open('src/autow/config/calibration.yaml'))
        self.arucoDict = aruco.Dictionary_get(aruco.DICT_6X6_50)
        self.arucoParams = aruco.DetectorParameters_create()
        self.mtx, self.dist = np.array(calibration['camera_matrix']), np.array(calibration['dist_coeff'])
        self.stopped = True
        self.foundAR = False

        self.autow_sub = self.create_subscription(
            String, 'autow_run', self.autow_callback, 10)
        self.autow_status = self.create_publisher(String, 'autow_status', 10)
        self.driver_pub = self.create_publisher(Float64MultiArray, 'drive', 10)
        self.create_timer(0, self.run)
        self.device = dai.Device(self.pipeline)
        self.qRgb = self.device.getOutputQueue(name="rgb", maxSize=1, blocking=False)



    def autow_callback(self, msg):
        if msg.data == "start":
            self.stopped = False
        elif msg.data == "stop":
            self.foundAR = False
            self.stopped = True

    def run(self):
       
        if not self.stopped:
            frame = self.qRgb.get()

            if frame is None:
                print(' No captured frame -- yikes!')
                return
            frame = frame.getCvFrame()
            # find center of ar tag
            (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, self.arucoDict,
                                                                parameters=self.arucoParams)
            if len(corners) == 0:
                if not self.foundAR:
                    playsound('src/autow/sounds/lost_ar.mp3', False)
                    self.autow_status.publish(String(data="done"))
                    self.stopped = True
                self.driver_pub.publish(Float64MultiArray(data=[-1, 0]))
                return
            self.foundAR = True
            ids = ids.flatten()
            if self.target_id not in ids:
                self.driver_pub.publish(Float64MultiArray(data=[-1, 0]))
                return
            idx = np.where(ids == self.target_id)[0][0]
            target_corners = corners[idx][0].reshape((4, 2))
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, 0.05, self.mtx, self.dist)
            theta = (rvecs[0, 0, 0]/math.pi*rvecs[0, 0, 2])

            target_height = abs(target_corners[2, 1]-target_corners[0, 1])
            if target_height/frame.shape[0] >= 0.24:
                # target is big enough, hitched
                self.driver_pub.publish(Float64MultiArray(data=[0.5, 0]))
                self.autow_status.publish(String(data="done"))
                self.foundAR = False
                self.stopped = True
                return

            self.steer_buff.append(self.calc_angle_hitch(
                self.hitch_ar, self.hitch_cam, -tvecs[0,0,0]-0.01, tvecs[0, 0, 2], theta))
            if len(self.steer_buff) >= 4:
                ave_steer = np.average(
                    self.steer_buff, weights=np.linspace(0, 1, len(self.steer_buff)))*1.15
                print(f"ratio {target_height/frame.shape[0]}")
                throttle = -(0.15 - abs(ave_steer-0.5)/15)*(1-target_height/frame.shape[0]*3)
                print(f"steer: {ave_steer}, throttle: {throttle}")
                self.driver_pub.publish(Float64MultiArray(data=[ave_steer, throttle]))
                self.steer_buff = []


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

    def stop(self):
        self.qRgb.close()
        self.device.close()
        self.stopped = True
        self.foundAR = False


def main(args=None):
    rclpy.init(args=args)
    node = AutowControl()
    rclpy.spin(node)
    node.stop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
