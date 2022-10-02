#!/usr/bin/env python3
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image, CameraInfo # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from functools import partial, lru_cache
import numpy as np

CAMERA_MATRIX = np.array([
    [1499.137795, 0.000000, 967.812559],
    [0.000000, 1502.565724, 562.783866],
    [0.000000, 0.000000, 1.000000]
])


CAMERA_DISTORTION = np.array([
    0.307289, -0.595430, -0.000126, 0.002014, 0.000000
])

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
parameters = cv2.aruco.DetectorParameters_create()


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
        return image

    ret = cv2.aruco.estimatePoseSingleMarkers(corners, 15, camera_matrix, camera_distortion)
    cv2.aruco.drawDetectedMarkers(image, corners, ids)
    for rvec, tvec in zip(ret[0], ret[1]):
        cv2.aruco.drawAxis(image, camera_matrix, camera_distortion, rvec, tvec, 10)

    return image

def callback(data, camera_matrix: np.array, camera_distortion: np.array):
 
    # Used to convert between ROS and OpenCV images
    br = CvBridge()
    # Output debugging information to the terminal
    rospy.loginfo("receiving video frame")

    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2(data)
    current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
 
    current_frame = process_frame(current_frame, camera_matrix=camera_matrix, camera_distortion=camera_distortion)
    # Display image
    cv2.imshow("camera", current_frame)

    cv2.waitKey(1)


def receive_message():
    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random
    # numbers are added to the end of the name. 
    rospy.init_node('video_sub_py', anonymous=True)

    # Node is subscribing to the video_frames topic
    camera_info = rospy.wait_for_message("/camera_info", CameraInfo, timeout=2)
    camera_matrix = np.reshape(np.array(camera_info.K), (3, 3))
    camera_distortion = np.array(camera_info.D)
    rospy.Subscriber('image_raw', Image, partial(callback, camera_matrix=camera_matrix, camera_distortion=camera_distortion))
    rospy.loginfo(camera_info)
    rospy.loginfo("*"*100)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    # Close down the video stream when done
    cv2.destroyAllWindows()
  
if __name__ == '__main__':
    receive_message()
