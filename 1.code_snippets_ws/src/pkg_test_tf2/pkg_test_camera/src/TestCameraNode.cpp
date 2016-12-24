#include "pkg_test_camera/TestCamera.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "node_test_camera");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ROS_INFO(pnh.getNamespace().c_str());
  pkg_test_camera::TestCamera test_camera(nh, pnh);
  ros::spin();
  return 0;
}
