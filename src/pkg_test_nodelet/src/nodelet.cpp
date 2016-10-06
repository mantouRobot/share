#include "node_test_nodelet.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>

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
  }
  boost::shared_ptr<pkg_test_nodelet::TestNode> p_;
};
}

PLUGINLIB_EXPORT_CLASS(pkg_test_nodelet::TestNodelet, nodelet::Nodelet);
