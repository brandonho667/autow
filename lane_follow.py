from utils import vesc
import cv2
import numpy as np
from utils.lane_detect import *
import depthai as dai

# Create pipeline
pipeline = dai.Pipeline()
cam = pipeline.create(dai.node.ColorCamera)
cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
cam.setPreviewSize(1280, 720)
xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("rgb")
cam.preview.link(xout.input)
# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    print("UVC running")
    while True:
        frame = qRgb.get().getFrame()

        if frame is None:
            print(' No captured frame -- Break!')
            break
        lane_image = np.copy(frame)
        lane_canny = find_canny(lane_image, 100, 200)
        # show_image('canny',lane_canny)
        lane_roi = region_of_interest(lane_canny)
        lane_lines = cv2.HoughLinesP(lane_roi, 1, np.pi/180, 50, 40, 5)
        lane_lines_plotted = draw_lines(lane_image, lane_lines)
        show_image('lines', lane_lines_plotted)
        result_lines = compute_average_lines(lane_image, lane_lines)
        print(result_lines)
        final_lines_mask = draw_lines(lane_image, result_lines)
        # show_image('final',final_lines_mask)
        # for points in result_lines:
        #     x1,y1,x2,y2 = points[0]
        #     cv2.line(frame,(x1,y1),(x2,y2),(0,0,255),2)
        # show_image('output',frame)
