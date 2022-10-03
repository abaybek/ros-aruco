import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math



marker_size = 15 # - [cm]
cap = cv2.VideoCapture('video.mp4')
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)


calib_path = ""
camera_matrix = np.loadtxt('cameraMatrix.txt', delimiter=',')
camera_distortion = np.loadtxt('cameraDistortion.txt', delimiter=',')


aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()


count = 0
SKIP_FACTOR = 30

while(cap.isOpened()):
    ret, frame = cap.read()

    if not ret: continue
    count += 1
    if count % SKIP_FACTOR != 0: continue

    cv2.imwrite('images_clean/frame{}.jpg'.format(count), frame)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected = aruco.detectMarkers(image=gray,
                                                 dictionary=aruco_dict,
                                                 parameters=parameters,
                                                 cameraMatrix=camera_matrix,
                                                 distCoeff=camera_distortion)

    if ids is not  None:
        ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

        if count % SKIP_FACTOR == 0:
            aruco.drawDetectedMarkers(frame, corners)

            for rvec, tvec in zip(ret[0], ret[1]):
                aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)
            # rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
            # rvec1,tvec1= ret[0][1,0,:], ret[1][1,0,:]
            # aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)
            # aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec1,tvec1,10)
            cv2.imwrite('images/frame{}.jpg'.format(count), frame)


cap.release()
cv2.destroyAllWindows()
