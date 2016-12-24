#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <iostream>

void imgCB(const sensor_msgs::ImageConstPtr &img)
{
  std::cout << "img." << std::endl;
}

void depthCB(const sensor_msgs::ImageConstPtr &depth)
{
  std::cout << "depth." << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "node_test_image_transport");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  image_transport::ImageTransport it(pnh);
  image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, imgCB);
  image_transport::Subscriber sub2 = it.subscribe("/camera/depth/image", 1, depthCB);
  ros::spin();
  return 0;
}
