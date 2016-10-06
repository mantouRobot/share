#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/thread.hpp>
#include <geometry_msgs/PointStamped.h>

namespace pkg_test_message_filter{

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PointStamped, geometry_msgs::PointStamped> MyPolicies;

class TestMessageFilter
{
public:
  ros::NodeHandle nh_;
  ros::Publisher pub1_;
  ros::Publisher pub2_;
  ros::Publisher pub_;
  message_filters::Subscriber<geometry_msgs::PointStamped> sub1_;
  message_filters::Subscriber<geometry_msgs::PointStamped> sub2_;
  message_filters::Synchronizer<MyPolicies> *sync_;
  boost::thread *pub_thread1, *pub_thread2;

  TestMessageFilter() :
    sub1_(nh_, "pub1", 1),
    sub2_(nh_, "pub2", 1),
    pub_thread1(0),
    pub_thread2(0)
  {
    pub1_ = nh_.advertise<geometry_msgs::PointStamped>("pub1", 1);
    pub2_ = nh_.advertise<geometry_msgs::PointStamped>("pub2", 1);
    pub_ = nh_.advertise<std_msgs::Float32>("pub", 1);
    sync_ = new message_filters::Synchronizer<MyPolicies>(MyPolicies(10), sub1_, sub2_);
    sync_->registerCallback(boost::bind(&TestMessageFilter::callback, this, _1, _2));
    pub_thread1 = new boost::thread(boost::bind(&TestMessageFilter::pubLoop1, this));
    pub_thread2 = new boost::thread(boost::bind(&TestMessageFilter::pubLoop2, this));
  }

  ~TestMessageFilter()
  {
    if(pub_thread1)
    {
      pub_thread1->join();
      delete pub_thread1;
    }
    if(pub_thread2)
    {
      pub_thread2->join();
      delete pub_thread2;
    }
    if(sync_)
    {
      delete sync_;
    }
  }

  void pubLoop1()
  {
    ros::Rate r(1);
    geometry_msgs::PointStamped tmp;
    tmp.point.x = 0;
    while(1)
    {
      tmp.header.stamp = ros::Time::now();
      pub1_.publish(tmp);
      tmp.point.x += 1;
      r.sleep();
    }
  }

  void pubLoop2()
  {
    ros::Rate r(10);
    geometry_msgs::PointStamped tmp;
    tmp.point.x = 0;
    while(1)
    {
      tmp.header.stamp = ros::Time::now();
      pub2_.publish(tmp);
      tmp.point.x += 1;
      r.sleep();
    }
  }

  void callback(const geometry_msgs::PointStampedConstPtr &p1, const geometry_msgs::PointStampedConstPtr &p2)
  {
    std_msgs::Float32 tmp;
    tmp.data = p1->point.x + p2->point.x;
    pub_.publish(tmp);
  }
};
}//end the namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "node_test_message_filter");
  pkg_test_message_filter::TestMessageFilter node_test;
  ros::spin();
  return 0;
}














