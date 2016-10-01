#include <ros/ros.h>
#include <nodelet/nodelet.h>

namespace pkg_test_nodelet{
class TestNode
{
public:
  TestNode()
  {
//    ros::NodeHandle nh_;
//    ros::NodeHandle pnh_("~");
//    ROS_INFO("I am a node.");
  }
  ~TestNode(){}
};
}

//int main(int argc, char** argv)
//{
//  ros::init(argc, argv, "node_test_nodelet");
//  pkg_test_nodelet::TestNode test_node;
//  ros::spin();
//  return 0;
//}
