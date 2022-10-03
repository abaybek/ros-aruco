#include <ros/ros.h>

#include "amonitor_aruco/MonitorAruco.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "aruco_node");
  amonitor_aruco::MonitorAruco thing;

  ros::spin();
  return 0;
}
