#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <pkg_test_actionlib/Cout.h>

#include <actionlib/client/simple_action_client.h>
#include <pkg_test_actionlib/CoutAction.h>

class CoutActionClient
{
public:
  ros::NodeHandle nh_;
  ros::ServiceServer sub_;
  actionlib::SimpleActionClient<pkg_test_actionlib::CoutAction> ac_;
  CoutActionClient() :
    ac_("server_node", true)//true: 不需要spin,有线程在spin
  {
    ROS_DEBUG_NAMED("test_actionlib", "%s", "hello world");//for console
    sub_ = nh_.advertiseService("/s_buildmap", &CoutActionClient::srvCb, this);
    ac_.waitForServer(ros::Duration(5));
  }

  bool srvCb(pkg_test_actionlib::Cout::Request &req, pkg_test_actionlib::Cout::Response &res)
  {
    pkg_test_actionlib::CoutGoal goal;
    goal.times = req.times;
    if(goal.times == 0)
    {
      ac_.cancelAllGoals();
      res.result = false;
    }
    else
    {
      ac_.sendGoal(goal);
      res.result = true;
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "client_node");
  CoutActionClient client;
  ros::spin();
  return 0;
}
