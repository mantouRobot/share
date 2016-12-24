#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_node_namespace");
  ros::NodeHandle nh1;
  std::cout << "the namespace is " << nh1.getNamespace() << std::endl;
  ros::NodeHandle nh2("rtabmap");
  std::cout << "the namespace is " << nh2.getNamespace() << std::endl;
  ros::NodeHandle nh3("/rtabmap");
  std::cout << "the namespace is " << nh3.getNamespace() << std::endl;
  ros::spin();
  return 0;
}
