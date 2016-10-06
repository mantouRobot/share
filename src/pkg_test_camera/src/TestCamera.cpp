//#include "pkg_test_camera/include/pkg_test_camera/TestCamera.h"
#include "pkg_test_camera/TestCamera.h"
//#include <nodelet/nodelet.h>
#include <iostream>
#include <std_msgs/String.h>

namespace pkg_test_camera{

TestCamera::TestCamera(ros::NodeHandle nh, ros::NodeHandle pnh) :
  nh_(nh),
  pnh_(pnh),
  info_sub_(nh_, "camera/rgb/camera_info", 100)
{
//  ros::NodeHandle rgb_nh(nh_, "rgb");
//  ros::NodeHandle depth_nh(nh_, "depth");
//  ros::NodeHandle rgb_pnh(pnh_, "rgb");
//  ros::NodeHandle depth_pnh(pnh_, "depth");

//  ros::NodeHandle rgb_nh, depth_nh, rgb_pnh, depth_pnh;
//  image_transport::ImageTransport it_rgb(rgb_nh);
//  image_transport::ImageTransport it_depth(depth_nh);
//  image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);
//  image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), depth_pnh);
//    rgb_sub_.subscribe(it_rgb, "/camera/rgb/image_raw", 100, hintsRgb);
//  depth_sub_.subscribe(it_depth, "/camera/depth_regist/image", 100, hintsDepth);

  image_transport::ImageTransport it(nh_);
  rgb_sub_.subscribe(it, "/camera/rgb/image_raw", 1);
  depth_sub_.subscribe(it, "camera/depth_regist/image", 1);

//  std::cout << depth_sub_.getTopic() << std::endl;

//  sync_ = new message_filters::Synchronizer<MyPolice>(MyPolice(100), depth_sub_, info_sub_);
//  sync_->registerCallback(boost::bind(&TestCamera::combineCB, this, _1, _2));

  sync_ = new message_filters::Synchronizer<MyPolice>(MyPolice(10), rgb_sub_, depth_sub_, info_sub_);
  sync_->registerCallback(boost::bind(&TestCamera::combineCB, this, _1, _2, _3));

  pub_ = nh_.advertise<std_msgs::String>("combine", 10);
}

TestCamera::~TestCamera()
{
  if(sync_)
    delete sync_;
}

//void TestCamera::combineCB(const sensor_msgs::ImageConstPtr &depth, const sensor_msgs::CameraInfoConstPtr &info)
void TestCamera::combineCB(const sensor_msgs::ImageConstPtr &rgb, const sensor_msgs::ImageConstPtr &depth, const sensor_msgs::CameraInfoConstPtr &info)
{
  ROS_INFO("combineCB.");
//  NODELET_INFO("combineCB.");
  std::cout << "combineCB 666." << std::endl;
  std_msgs::String str;
  str.data = "hello";
  pub_.publish(str);
}

}
