#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_log_node");
  ros::NodeHandle n;
  ros::Rate r(1);
  while(1)
  {
    ROS_DEBUG_NAMED("test_log_node", "hello world");
    ROS_INFO("111...");
    r.sleep();
  }
  return 0;
}
