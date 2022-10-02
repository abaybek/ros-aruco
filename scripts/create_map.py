#!/usr/bin/env python3

import rospy
import cv2
import tf2_ros
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from functools import partial


aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
parameters = cv2.aruco.DetectorParameters_create()
aruco_size = 0.5


def get_camera_configs():
    camera_info = rospy.wait_for_message("/camera_info", CameraInfo, timeout=2)
    camera_matrix = np.reshape(np.array(camera_info.K), (3, 3))
    camera_distortion = np.array(camera_info.D)
    return camera_matrix, camera_distortion


def process_frame(image, camera_matrix: np.array, camera_distortion: np.array):

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected = cv2.aruco.detectMarkers(
        image=gray,
        dictionary=aruco_dict,
        parameters=parameters,
        cameraMatrix=camera_matrix,
        distCoeff=camera_distortion
    )
    if ids is None:
        rospy.loginfo("nothing found")
        return
    
    rvec, tvec, cc = cv2.aruco.estimatePoseSingleMarkers(corners, aruco_size, camera_matrix, camera_distortion)
    # rospy.loginfo(rvec)
    rospy.loginfo(tvec)
    # rospy.loginfo(cc)
    return 


    ret = cv2.aruco.estimatePoseSingleMarkers(corners, aruco_size, camera_matrix, camera_distortion)
    cv2.aruco.drawDetectedMarkers(image, corners, ids)
    for rvec, tvec in zip(ret[0], ret[1]):
        cv2.aruco.drawAxis(image, camera_matrix, camera_distortion, rvec, tvec, 10)

    return image


def callback(data, camera_matrix: np.array, camera_distortion: np.array):
    br = CvBridge()
    rospy.loginfo("receiving video frame")

    current_frame = br.imgmsg_to_cv2(data)
    current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
    process_frame(current_frame, camera_matrix, camera_distortion)



def main():
    rospy.init_node("create_map_node", anonymous=True)
    camera_matrix, camera_distortion = get_camera_configs()

    rospy.Subscriber("image_raw", Image, partial(callback, camera_matrix=camera_matrix, camera_distortion=camera_distortion))
    rospy.loginfo("*"*100)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    # Close down the video stream when done
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
