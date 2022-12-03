import cv2
import cv2.aruco as aruco
import yaml
import depthai as dai
import numpy as np
import math

arucoDict = aruco.Dictionary_get(aruco.DICT_6X6_50)
arucoParams = aruco.DetectorParameters_create()

all_camera_idx_available = []
pipeline = dai.Pipeline()
cam = pipeline.create(dai.node.ColorCamera)
cam.setPreviewSize(640, 480)
cam.setInterleaved(False)
xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("rgb")
cam.preview.link(xout.input)

mtx, dist = yaml.safe_load(open('calibration.yaml'))['camera_matrix'], yaml.safe_load(
    open('calibration.yaml'))['dist_coeff']
mtx, dist = np.array(mtx), np.array(dist)
print(mtx, dist)

# for camera_idx in range(10):
#     cap = cv2.VideoCapture(camera_idx)
#     if cap.isOpened():
#         print(f'Camera index available: {camera_idx}')
#         all_camera_idx_available.append(camera_idx)
#         cap.release()
# print(all_camera_idx_available)
# # Create a VideoCapture object
# cap = cv2.VideoCapture(all_camera_idx_available[0])

# # Check if camera opened successfully
# if (cap.isOpened() == False):
#     print("Error opening video stream or file")
with dai.Device(pipeline) as device:
    qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    print("UVC running")

    while True:
        # ret, image = cap.read()
        frame = qRgb.get()
        image = frame.getCvFrame()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict,
                                                           parameters=arucoParams)

        if len(corners) > 0:
            ids = ids.flatten()
            aruco.drawDetectedMarkers(image, corners, ids)

            for (markerCorner, markerID) in zip(corners, ids):
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    markerCorner, 0.05, mtx, dist)
                aruco.drawAxis(image, mtx, dist, rvecs, tvecs, 0.1)
                print(rvecs.shape)
                # corners = markerCorner.reshape((4, 2))
                # print(
                #     f"{(-rvecs[0,0,0]/math.pi*rvecs[0,0,2])*180/math.pi:.2f}")
                print(f"{tvecs[0,0,0]:.2f}",
                      f"{tvecs[0,0,1]:.2f}", f"{tvecs[0,0,2]:.2f}")
            # (topLeft, topRight, bottomRight, bottomLeft) = corners
            # topRight = (int(topRight[0]), int(topRight[1]))
            # bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            # bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            # topLeft = (int(topLeft[0]), int(topLeft[1]))

            # # rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
            # #                                                            distortion_coefficients)
            # # (rvec - tvec).any()  # get rid of that nasty numpy value array error
            # # # Draw A square around the markers
            # # aruco.drawDetectedMarkers(frame, corners)
            # # aruco.drawAxis(frame, matrix_coefficients,
            # #                distortion_coefficients, rvec, tvec, 0.01)  # Draw axis

            # # draw the bounding box of the ArUCo detection
            # cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
            # cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            # cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            # cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
            # # compute and draw the center (x, y)-coordinates of the ArUco
            # # marker
            # cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            # cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            # cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
            # # draw the ArUco marker ID on the image
            # cv2.putText(image, str(markerID),
            #             (topLeft[0], topLeft[1] -
            #              15), cv2.FONT_HERSHEY_SIMPLEX,
            #             0.5, (0, 255, 0), 2)
            print("[INFO] ArUco marker ID: {}".format(markerID))
        # show the output image
        cv2.imshow("Image", image)
        cv2.waitKey(1)
