#ifndef TESTCAMERA
#define TESTCAMERA

#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace pkg_test_camera{
class TestCamera
{
public:
  TestCamera(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~TestCamera();
//  void combineCB(const sensor_msgs::ImageConstPtr &depth, const sensor_msgs::CameraInfoConstPtr &info);
  void combineCB(const sensor_msgs::ImageConstPtr &rgb, const sensor_msgs::ImageConstPtr &depth, const sensor_msgs::CameraInfoConstPtr &info);

  ros::NodeHandle nh_, pnh_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
  image_transport::SubscriberFilter rgb_sub_;
  image_transport::SubscriberFilter depth_sub_;

//  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> MyPolice;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MyPolice;

  message_filters::Synchronizer<MyPolice> *sync_;

  ros::Publisher pub_;
};
}//end namespace
#endif
