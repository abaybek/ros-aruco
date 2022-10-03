#!/usr/bin/env python3

import rospy
import cv2
import tf
import tf2_ros
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
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


def process_frame(image, camera_matrix: np.array, camera_distortion: np.array, publisher):

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
        return image
    
    rvecs, tvecs, cc = cv2.aruco.estimatePoseSingleMarkers(corners, aruco_size, camera_matrix, camera_distortion)
    cv2.aruco.drawDetectedMarkers(image, corners, ids)
    for rvec, tvec, _id in zip(rvecs, tvecs, ids):
        cv2.aruco.drawAxis(image, camera_matrix, camera_distortion, rvec, tvec, 0.3)
        pub_func(rvec[0], tvec[0], _id, publisher)

    print(ids)
    return image


def callback(data, camera_matrix: np.array, camera_distortion: np.array, publisher, image_pub):
    br = CvBridge()
    rospy.loginfo("receiving video frame")

    current_frame = br.imgmsg_to_cv2(data)
    current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
    current_frame = process_frame(current_frame, camera_matrix, camera_distortion, publisher)

    image_pub.publish(br.cv2_to_imgmsg(current_frame))
    # for _id, rvec, tvec in process_frame(current_frame, camera_matrix, camera_distortion):
    #     pub_func(rvec, tvec, _id, publisher)


def pub_func(rvec, tvec, _id, publisher):
    # we need a homogeneous matrix but OpenCV only gives us a 3x3 rotation matrix
    # rotation_matrix = np.array([[0, 0, 0, 0],
    #                             [0, 0, 0, 0],
    #                             [0, 0, 0, 0],
    #                             [0, 0, 0, 1]],
    #                             dtype=float)
    # rotation_matrix[:3, :3], _ = cv2.Rodrigues([rvec[2], rvec[1], rvec[0]])

    # convert the matrix to a quaternion
    # quaternion = tf.transformations.quaternion_from_matrix(rotation_matrix)
    # quaternion = tf.transformations.quaternion_from_euler(rvec[2], rvec[1], rvec[0])
    print(rvec)
    print(tvec)
    print(_id)
    quaternion = tf.transformations.quaternion_from_euler(rvec[2], rvec[1] + 1.5, rvec[0])

    # To visualize in rviz, you can, for example, publish a PoseStamped message:
    pose = Marker()
    # breakpoint()
    pose.ns = "basic_shapes"
    pose.id = int(_id)
    pose.type = Marker.CUBE
    pose.action = Marker.ADD
    # pose.lifetime.secs = 100.0
    pose.header.frame_id = "camera_link"
    pose.pose.position.x = tvec[2]
    pose.pose.position.y = tvec[1]
    pose.pose.position.z = -tvec[0]
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]
    pose.scale.x = 0.2
    pose.scale.y = 0.2
    pose.scale.z = 0.02

    pose.color.r = 1.0
    pose.color.g = 1.0
    pose.color.b = 1.0
    pose.color.a = 1.0
    print(pose)
    publisher.publish(pose)


def main():
    rospy.init_node("create_map_node", anonymous=True)
    camera_matrix, camera_distortion = get_camera_configs()

    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 10)
    image_pub = rospy.Publisher("/image_processed", Image, queue_size = 1)
    rospy.Subscriber("image_raw", Image, partial(callback, camera_matrix=camera_matrix, camera_distortion=camera_distortion, publisher=marker_pub, image_pub=image_pub))
    rospy.loginfo("*"*100)
    
    rospy.spin() # simply keeps python from exiting until this node is stopped
    # while not rospy.is_shutdown():
        # do whatever you want here
        # pub.publish(foo)
        # rospy.sleep(0.01)  # sleep for one second


    # Close down the video stream when done
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
