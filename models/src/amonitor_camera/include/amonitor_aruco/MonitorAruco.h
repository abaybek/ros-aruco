#pragma once
#define _USE_MATH_DEFINES
#include <math.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <string> 

#include <opencv2/opencv.hpp>
#include <experimental/filesystem>
// #include <opencv2/highgui/highgui.hpp>

#include <aruco/markermap.h>
#include <aruco/markerdetector.h>
#include <aruco/posetracker.h>
#include <aruco/cameraparameters.h>
#include <aruco/dictionary.h>
#include <aruco/aruco.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>




namespace amonitor_aruco
{
    class MonitorAruco
    {
    public:
        MonitorAruco();
        ~MonitorAruco();

    private:
        // ROS Node handles
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        // image transport pub/sub
        image_transport::ImageTransport it_;
        image_transport::CameraSubscriber image_sub_;
        image_transport::Publisher image_pub_;
        

        // Ros publishers
        ros::Publisher marker_pub;
        ros::Publisher marker_pub_debug_;

        //
        // Methods
        //
        double markerSize_;
        aruco::MarkerMap mmConfig_;
        aruco::MarkerDetector mDetector_;
        aruco::MarkerMapPoseTracker mmPoseTracker_;
        aruco::CameraParameters camParams_;

        // Debugging
        bool showOutputVideo_;
        bool publishToRviz_;
        bool debugSaveInputFrames_;
        bool debugSaveOutputFrames_;
        std::string debugImagePath_;

        // Visualization consts
        static constexpr double RVIZ_MARKER_HEIGHT = 0.01;
        static constexpr double RVIZ_MARKER_LIFETIME_FOREVER = 0.0;
        static constexpr double RVIZ_MARKER_LIFETIME = 0.5;
        static constexpr double RVIZ_MARKER_COLOR_R = 1.0;
        static constexpr double RVIZ_MARKER_COLOR_G = 1.0;
        static constexpr double RVIZ_MARKER_COLOR_B = 1.0;
        static constexpr double RVIZ_MARKER_COLOR_A = 1.0;


        // image_transport camera subsriber
        void cameraCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cinfo);
        void processImage(cv::Mat& frame, bool drawDetections);

        // rviz visualization
        void publishMarker(aruco::Marker3DInfo marker);
        void publishMarkerText(aruco::Marker3DInfo marker);
        void publishMarkerSeen(aruco::Marker marker);
        void publishCameraLocation(cv::Mat rvec, cv::Mat tvec);

        // Convert ROS CameraInfo message to ArUco style CameraParameters
        aruco::CameraParameters ros2arucoCamParams(const sensor_msgs::CameraInfoConstPtr& cinfo);

        // tf 
        tf::Transform aruco2tf(const cv::Mat& rvec, const cv::Mat& tvec);
        tf::Quaternion rodriguesToTFQuat(const cv::Mat& rvec);

        // helper method
        void saveInputFrame(const cv::Mat& frame);
        void saveOutputFrame(const cv::Mat& frame);
        void saveFrame(const cv::Mat& frame, std::string format_spec, unsigned int img_num);
    };
}