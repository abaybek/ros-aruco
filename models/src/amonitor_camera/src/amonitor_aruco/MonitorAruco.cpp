#include "amonitor_aruco/MonitorAruco.h"

namespace amonitor_aruco
{
    MonitorAruco::MonitorAruco() :
        nh_(ros::NodeHandle()), nh_private_("~"), it_(nh_)
    {
        ROS_INFO("Constructor created");

        std::string mmConfigFile = nh_private_.param<std::string>("markermap_config", "");
        markerSize_ = nh_private_.param<double>("marker_size", 0.5);
        
        nh_private_.param<bool>("publish_to_rviz", publishToRviz_, false);
        nh_private_.param<bool>("show_output_video", showOutputVideo_, false);
        nh_private_.param<bool>("debug_save_input_frames", debugSaveInputFrames_, false);
        nh_private_.param<bool>("debug_save_output_frames", debugSaveOutputFrames_, false);
        nh_private_.param<std::string>("debug_image_path", debugImagePath_, "/tmp/arucoimages");
        
        // Subscribe to input video feed
        it_ = image_transport::ImageTransport(nh_);
        image_sub_ = it_.subscribeCamera("input_image", 1, &MonitorAruco::cameraCallback, this);
    
        // ROS publishers
        marker_pub = nh_private_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        marker_pub_debug_ = nh_private_.advertise<visualization_msgs::Marker>("visualization_marker_camera", 1);

        //
        // Aruco configuration
        //
        ROS_INFO("Aruco configuration started");
        mmConfig_.readFromFile(mmConfigFile);
        mDetector_.setDictionary(mmConfig_.getDictionary());

        ROS_INFO("Aruco configuration ended");

        if (mmConfig_.isExpressedInPixels())
            mmConfig_ = mmConfig_.convertToMeters(markerSize_);
        

        
    }

    MonitorAruco::~MonitorAruco()
    {
        ROS_INFO("Destructor");
        // cv::destroyWindow("view");
    }

    // Methods
    void MonitorAruco::cameraCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cinfo)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            // cv::imshow("view", cv_bridge::toCvShare(image, "bgr8")->image);
            // cv::waitKey(30);
            cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        // Configure the Pose Tracker if it has not been configured before
        if (!mmPoseTracker_.isValid()) {
            // Extract ROS camera_info (i.e., K and D) for ArUco library
            camParams_ = ros2arucoCamParams(cinfo);
            // Now, if the camera params have been ArUco-ified, set up the tracker
            if (camParams_.isValid()){
                mmPoseTracker_.setParams(camParams_, mmConfig_);
            }
        }


        cv::Mat frame = cv_ptr->image;

        // saveInputFrame(frame);
        processImage(frame, 0);
        if(publishToRviz_){
            for (auto m : mmConfig_){
                publishMarker(m);
                publishMarkerText(m);
            }
        }
        // cv::imshow("detections", frame);
        // cv::waitKey(30);

    }

    void MonitorAruco::processImage(cv::Mat& frame, bool drawDetections)
    {
        ROS_INFO("---------------------");
        std::vector<aruco::Marker> detected_markers = mDetector_.detect(frame);
        

        if (drawDetections){
            for (auto marker: detected_markers){
                ROS_INFO("found %i", marker.id);
                marker.draw(frame, cv::Scalar(0,0,255), 2);
                marker.calculateExtrinsics(markerSize_, camParams_, false);

                ROS_INFO("Found Rvec");
                ROS_INFO_STREAM(marker.Rvec);
                ROS_INFO("Found Tvec");
                ROS_INFO_STREAM(marker.Tvec);

                if(camParams_.isValid()){
                    if(marker.isPoseValid()){
                        aruco::CvDrawingUtils::draw3dCube(frame, marker, camParams_);
                        aruco::CvDrawingUtils::draw3dAxis(frame, marker, camParams_);
                        // publishMarkerSeen(marker);
                    }
                }
            }
        }
        if (mmConfig_.isExpressedInMeters() && camParams_.isValid())
        {

            if (mmPoseTracker_.estimatePose(detected_markers))  // if pose correctly computed, print the reference system
                {
                aruco::CvDrawingUtils::draw3dAxis(frame, camParams_, mmPoseTracker_.getRvec(), mmPoseTracker_.getTvec(),
                                                  mmConfig_[0].getMarkerSize() * 2);
                
                std::vector<float> vec{0.0, 0.0, 0.0, 1.0};
                cv::Mat m1( vec );
                ROS_INFO_STREAM(m1);
                
                // ROS_INFO_STREAM(mmPoseTracker_.getRTMatrix());
                // ROS_INFO_STREAM(mmPoseTracker_.getTvec());
                // ROS_INFO_STREAM(mmPoseTracker_.getRvec());
                // ROS_INFO_STREAM(mmPoseTracker_.getRTMatrix());

                cv::Mat tran = mmPoseTracker_.getRTMatrix().inv();
                cv::Mat result = tran * m1;
                // ROS_INFO_STREAM(result);
                // ROS_INFO_STREAM();
                publishCameraLocation(mmPoseTracker_.getRvec(), result);
                mmPoseTracker_.reset();
                }
        }

        
        // ROS_INFO();
    }

    void MonitorAruco::publishMarkerSeen(aruco::Marker marker)
    {
        static const uint32_t shape = visualization_msgs::Marker::CUBE;
        visualization_msgs::Marker vis_marker;
        vis_marker.header.frame_id = "/my_frame";
        vis_marker.header.stamp = ros::Time::now();

        vis_marker.ns = "basic_shapes";
        vis_marker.id = marker.id;
        vis_marker.type = shape;
        vis_marker.action = visualization_msgs::Marker::ADD;
        vis_marker.pose.position.x = marker.Tvec.at<float>(0);
        vis_marker.pose.position.y = marker.Tvec.at<float>(1);
        vis_marker.pose.position.z = marker.Tvec.at<float>(2);

        // Represent Rodrigues parameters as a quaternion
        tf::Quaternion quat = rodriguesToTFQuat(marker.Rvec);
        tf::quaternionTFToMsg(quat, vis_marker.pose.orientation);
        
        vis_marker.scale.x = markerSize_;
        vis_marker.scale.y = markerSize_;
        vis_marker.scale.z = amonitor_aruco::MonitorAruco::RVIZ_MARKER_HEIGHT;

        vis_marker.color.r = amonitor_aruco::MonitorAruco::RVIZ_MARKER_COLOR_R;
        vis_marker.color.g = amonitor_aruco::MonitorAruco::RVIZ_MARKER_COLOR_G;
        vis_marker.color.b = amonitor_aruco::MonitorAruco::RVIZ_MARKER_COLOR_B;
        vis_marker.color.a = amonitor_aruco::MonitorAruco::RVIZ_MARKER_COLOR_A;

        vis_marker.lifetime = ros::Duration(amonitor_aruco::MonitorAruco::RVIZ_MARKER_LIFETIME);
        marker_pub.publish(vis_marker);
    }
    
    void MonitorAruco::publishCameraLocation(cv::Mat rvec, cv::Mat tvec)
    {
        static const uint32_t shape = visualization_msgs::Marker::CUBE;
        visualization_msgs::Marker vis_marker;
        
        vis_marker.header.frame_id = "/my_frame";
        vis_marker.header.stamp = ros::Time::now();

        // Create the transform from the camera to the ArUco Marker Map
        tf::Transform transform = aruco2tf(rvec, tvec);
        // ROS_INFO_STREAM(transform);

        vis_marker.ns = "basic_shapes";
        vis_marker.id = 99999;
        vis_marker.type = shape;
        vis_marker.action = visualization_msgs::Marker::ADD;
        // vis_marker.pose.position.x = tvec.at<double>(0);
        // vis_marker.pose.position.y = tvec.at<double>(2);
        // vis_marker.pose.position.z = tvec.at<double>(1);
        // vis_marker.pose.orientation.x = 0.0;
        // vis_marker.pose.orientation.y = 0.0;
        // vis_marker.pose.orientation.z = 0.0;
        // vis_marker.pose.orientation.w = 1.0;
        tf::poseTFToMsg(transform, vis_marker.pose);

        vis_marker.scale.x = markerSize_;
        vis_marker.scale.y = markerSize_;
        vis_marker.scale.z = amonitor_aruco::MonitorAruco::RVIZ_MARKER_HEIGHT;

        vis_marker.color.r = 0.0f;
        vis_marker.color.g = 0.0f;
        vis_marker.color.b = 0.0f;
        vis_marker.color.a = amonitor_aruco::MonitorAruco::RVIZ_MARKER_COLOR_A;

        vis_marker.lifetime = ros::Duration(amonitor_aruco::MonitorAruco::RVIZ_MARKER_LIFETIME);
        marker_pub.publish(vis_marker);
        marker_pub_debug_.publish(vis_marker);
    }

    void MonitorAruco::publishMarker(aruco::Marker3DInfo marker)
    {
        static const uint32_t shape = visualization_msgs::Marker::LINE_STRIP;
        visualization_msgs::Marker vis_marker;
        
        vis_marker.header.frame_id = "/my_frame";
        vis_marker.header.stamp = ros::Time::now();
        
        vis_marker.ns = "basic_shapes";
        vis_marker.id = marker.id;
        vis_marker.type = shape;
        vis_marker.action = visualization_msgs::Marker::ADD;

        geometry_msgs::Point p;
        p.x = marker.points[0].x;
        p.y = marker.points[0].y;
        p.z = marker.points[0].z;

        for( uint32_t i = 0; i < 4; i++)
        {
            geometry_msgs::Point p;
            p.x = marker.points[i].x;
            p.y = marker.points[i].y;
            p.z = marker.points[i].z;
            vis_marker.points.push_back(p);
        }
        vis_marker.points.push_back(p);

        vis_marker.scale.x = 0.01;
        // vis_marker.scale.x = markerSize_;
        // vis_marker.scale.y = markerSize_;
        // vis_marker.scale.z = amonitor_aruco::MonitorAruco::RVIZ_MARKER_HEIGHT;

        vis_marker.color.r = amonitor_aruco::MonitorAruco::RVIZ_MARKER_COLOR_R;
        vis_marker.color.g = amonitor_aruco::MonitorAruco::RVIZ_MARKER_COLOR_G;
        vis_marker.color.b = amonitor_aruco::MonitorAruco::RVIZ_MARKER_COLOR_B;
        vis_marker.color.a = amonitor_aruco::MonitorAruco::RVIZ_MARKER_COLOR_A;

        vis_marker.lifetime = ros::Duration(amonitor_aruco::MonitorAruco::RVIZ_MARKER_LIFETIME);
        marker_pub.publish(vis_marker);
    }

    void MonitorAruco::publishMarkerText(aruco::Marker3DInfo marker)
    {
        static const uint32_t shape = visualization_msgs::Marker::TEXT_VIEW_FACING;
        visualization_msgs::Marker vis_marker;
        
        vis_marker.header.frame_id = "/my_frame";
        vis_marker.header.stamp = ros::Time::now();
        
        vis_marker.ns = "basic_shapes";
        vis_marker.id = marker.id + 2000;
        vis_marker.text = std::to_string(marker.id);
        vis_marker.type = shape;
        vis_marker.action = visualization_msgs::Marker::ADD;
        vis_marker.pose.position.x = ((marker.points[0].x + marker.points[1].x + marker.points[2].x + marker.points[3].x) / 4.0f);
        vis_marker.pose.position.y = ((marker.points[0].y + marker.points[1].y + marker.points[2].y + marker.points[3].y) / 4.0f);
        vis_marker.pose.position.z = ((marker.points[0].z + marker.points[1].z + marker.points[2].z + marker.points[3].z) / 4.0f);
        vis_marker.pose.orientation.x = 0.0;
        vis_marker.pose.orientation.y = 0.0;
        vis_marker.pose.orientation.z = 0.0;
        vis_marker.pose.orientation.w = 1.0;


        vis_marker.scale.z = amonitor_aruco::MonitorAruco::RVIZ_MARKER_HEIGHT * 10;

        vis_marker.color.r = amonitor_aruco::MonitorAruco::RVIZ_MARKER_COLOR_R;
        vis_marker.color.g = amonitor_aruco::MonitorAruco::RVIZ_MARKER_COLOR_G;
        vis_marker.color.b = amonitor_aruco::MonitorAruco::RVIZ_MARKER_COLOR_B;
        vis_marker.color.a = amonitor_aruco::MonitorAruco::RVIZ_MARKER_COLOR_A;

        vis_marker.lifetime = ros::Duration(amonitor_aruco::MonitorAruco::RVIZ_MARKER_LIFETIME);
        marker_pub.publish(vis_marker);
    }

    void MonitorAruco::saveInputFrame(const cv::Mat& frame)
    {
        static unsigned int counter = 0;
        saveFrame(frame, "aruco%03i_in.png", counter++);
    }

    void MonitorAruco::saveOutputFrame(const cv::Mat& frame)
    {
        static unsigned int counter = 0;
        saveFrame(frame, "aruco%03i_out.png", counter++);
    }

    // From camera frame to ArUco marker
    tf::Transform MonitorAruco::aruco2tf(const cv::Mat& rvec, const cv::Mat& tvec) {
        // convert tvec to a double
        cv::Mat tvec64; tvec.convertTo(tvec64, CV_64FC1);

        // Convert Rodrigues paramaterization of the rotation to quat
        tf::Quaternion q1 = rodriguesToTFQuat(rvec);

        tf::Vector3 origin(tvec64.at<double>(0), tvec64.at<double>(1), tvec64.at<double>(2));

        // The measurements coming from the ArUco lib are vectors from the
        // camera coordinate system pointing at the center of the ArUco board.
        return tf::Transform(q1, origin);
    }

    tf::Quaternion MonitorAruco::rodriguesToTFQuat(const cv::Mat& rvec)
    {
        // convert rvec to double
        cv::Mat rvec64; rvec.convertTo(rvec64, CV_64FC1);

        // Unpack Rodrigues paramaterization of the rotation
        cv::Mat rot(3, 3, CV_64FC1);
        cv::Rodrigues(rvec64, rot);

        // Convert OpenCV to tf matrix
        tf::Matrix3x3 tf_rot(rot.at<double>(0,0), rot.at<double>(0,1), rot.at<double>(0,2),
                            rot.at<double>(1,0), rot.at<double>(1,1), rot.at<double>(1,2),
                            rot.at<double>(2,0), rot.at<double>(2,1), rot.at<double>(2,2));

        // convert rotation matrix to an orientation quaternion
        tf::Quaternion quat;
        tf_rot.getRotation(quat);

        // For debugging
        // double r, p, y;
        // tf::Matrix3x3(quat).getRPY(r,p,y);
        // std::cout << "RPY: [ " << r*(180.0/M_PI) << ", " << p*(180.0/M_PI) << ", " << y*(180.0/M_PI) << " ]\t";

        return quat;
    }

    void MonitorAruco::saveFrame(const cv::Mat& frame, std::string format_spec, unsigned int img_num) {
        // Create a filename
        std::stringstream ss;
        char buffer[100];
        sprintf(buffer, format_spec.c_str(), img_num);
        ss << "/tmp/arucoimages" << "/" << buffer;
        std::string filename = ss.str();

        // save the frame
        cv::imwrite(filename, frame);
    }

    aruco::CameraParameters MonitorAruco::ros2arucoCamParams(const sensor_msgs::CameraInfoConstPtr& cinfo) {
        cv::Mat cameraMatrix(3, 3, CV_64FC1);
        cv::Mat distortionCoeff(4, 1, CV_64FC1);
        cv::Size size(cinfo->height, cinfo->width);

        // Make a regular 3x3 K matrix from CameraInfo
        for(int i=0; i<9; ++i)
            cameraMatrix.at<double>(i%3, i-(i%3)*3) = cinfo->K[i];

        // The ArUco library requires that there are only 4 distortion params (k1, k2, p1, p2, 0) 
        if (cinfo->D.size() == 4 || cinfo->D.size() == 5) {

            // Make a regular 4x1 D matrix from CameraInfo
            for(int i=0; i<4; ++i)
                distortionCoeff.at<double>(i, 0) = cinfo->D[i];

        } else {

            ROS_WARN("[aruco] Length of distortion matrix is not 4, assuming zero distortion.");
            for(int i=0; i<4; ++i)
                distortionCoeff.at<double>(i, 0) = 0;

        }
        // ROS_INFO("Camera matrix");
        // ROS_INFO_STREAM(cameraMatrix);
        // ROS_INFO("Distort coeff");
        // ROS_INFO_STREAM(distortionCoeff);
        

        return aruco::CameraParameters(cameraMatrix, distortionCoeff, size);
    }
}