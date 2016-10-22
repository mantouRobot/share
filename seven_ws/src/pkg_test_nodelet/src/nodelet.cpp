#include "node_test_nodelet.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>
//#include <vector>
//#include <string>

namespace pkg_test_nodelet{
class TestNodelet : public nodelet::Nodelet
{
public:
  TestNodelet(){}
  ~TestNodelet(){}

  virtual void onInit()
  {
    p_.reset(new pkg_test_nodelet::TestNode());
    ros::NodeHandle pnh = getNodeHandle();
    std::string str;
    pnh.param<std::string>("param1", str, "str");
    NODELET_INFO("%s", str.c_str());
    std::cout << "I am a nodelet." << std::endl;
    std::vector<std::string> argv = getMyArgv();
    std::cout << "argument: " << argv.size() << std::endl;
    for(int i = 0; i < argv.size(); i++)
      std::cout << argv[i].c_str() << std::endl;

    ros::NodeHandle nh, p("~");
//    NODELET_ERROR("%s", nh.getNamespace().c_str());
    NODELET_INFO("For the node method:");
    ROS_INFO("%s", nh.getNamespace().c_str());
    ROS_INFO("%s", p.getNamespace().c_str());

    nh = getNodeHandle();
    p = getPrivateNodeHandle();
    NODELET_INFO("For the nodelet method: ");
    ROS_INFO("%s", nh.getNamespace().c_str());
    ROS_INFO("%s", p.getNamespace().c_str());
  }
  boost::shared_ptr<pkg_test_nodelet::TestNode> p_;
};
}

PLUGINLIB_EXPORT_CLASS(pkg_test_nodelet::TestNodelet, nodelet::Nodelet);
