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
            self.vesc.set_throttle(0.2)
            while not self.stopped:
                frame = qRgb.get()

                if frame is None:
                    print(' No captured frame -- yikes!')
                    continue
                frame = frame.getCvFrame()
                frame = cv2.flip(frame, -1)
                image = cv2.resize(frame,(650,500))
                image = self.region_of_interest(image)

                #Convert to hsv color space
                lane_image = np.copy(image)
                # print(lane_image.shape)
                lane_image = cv2.cvtColor(lane_image,cv2.COLOR_BGR2HSV)
                # show_image('hsv_input',lane_image)


                #Defining upper and lower boundaries for yellow color
                #Create mask 
                lower_yellow = np.array([22,33,100])
                upper_yellow = np.array([39,164,203])
                lane_yellow_mask = cv2.inRange(lane_image,lower_yellow,upper_yellow)
                # show_image('BW lines',lane_yellow_mask)


                result = cv2.bitwise_and(image, image, mask=lane_yellow_mask)
                # show_image('yellow lines',result)


                ## Converting to binary threshold and finding contours
                gray = cv2.cvtColor(result,cv2.COLOR_RGB2GRAY)
                blurred = cv2.GaussianBlur(gray, (5, 5), 0)
                # apply binary thresholding apply Otsu's automatic thresholding which automatically determines the best threshold value
                ret, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
                # visualize the binary image
                # show_image('Binary image', thresh)
                # contours, hierarchy = cv2.findContours(lane_yellow_mask, 1, cv2.CHAIN_APPROX_NONE)
                contours, hierarchies = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
                for contour in contours:
                    cv2.drawContours(result, contour, -1, (0, 255, 0), 3)
                # show_image('Image with contours',result)

                centers = []
                ##Finding center of contours
                for i in contours:
                    M = cv2.moments(i)
                    if M['m00'] != 0:
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                    
                        cv2.drawContours(image, [i], -1, (0, 255, 0), 2)
                        cv2.circle(image, (cx, cy), 7, (0, 0, 255), -1)
                        cv2.putText(image, "center", (cx - 20, cy - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                        centers.append((cx,cy))
                if len(centers) == 0:
                    print("No centers detected")
                    continue
                show_image('Image with centers',image)

                nearest = min(centers, key=lambda x: x[1])
                print(nearest, image.shape)
                self.steer_buff.append(
                    self.calc_angle(image, nearest))
                if len(self.steer_buff) >= 10:
                    ave_steer = np.average(
                        self.steer_buff, weights=np.linspace(0, 1, len(self.steer_buff)))
                    smooth = ave_steer
                    if ave_steer < 0.3:
                        smooth = 0.3
                    elif ave_steer > 0.7:
                        smooth = 0.7
                    else:
                        smooth = ave_steer
                    self.vesc.set_steer(smooth)
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

    def region_of_interest(self, image): #function for extracting region of interest
        #bounds in (x,y) format
        bounds = np.array([[[0,500],[0,290],[650,290],[650,550]]],dtype=np.int32)
        # bounds = np.array([[[0,image.shape[0]],[0,image.shape[0]/2],[900,image.shape[0]/2],[900,image.shape[0]]]],dtype=np.int32)
        mask=np.zeros_like(image)
        cv2.fillPoly(mask,bounds,[255,255,255])
        # show_image('inputmask',mask)
        masked_image = cv2.bitwise_and(image,mask)
        # show_image('mask',masked_image) 
        return masked_image

    # calc steering for given center point
    def calc_angle(self, frame, center_pt):
        width, height = frame.shape[1], frame.shape[0]
        x, y = center_pt
        x -= width/2
        if abs(x) < 0.01:
            return 0.5
        y = height - y
        angle = np.arctan(y/x)*1.3
        return angle/np.pi + 0.5

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