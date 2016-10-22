#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pkg_test_actionlib/CoutAction.h>

class CoutActionClient
{
public:
  actionlib::SimpleActionClient<pkg_test_actionlib::CoutAction> ac_;
  CoutActionClient() :
    ac_("server_node", true)//true: 不需要spin,有线程在spin
  {
    ac_.waitForServer(ros::Duration(1));
    pkg_test_actionlib::CoutGoal goal;
    goal.times = 5;
    ac_.sendGoal(goal);
    ac_.waitForResult(ros::Duration(30));
    ac_.cancelAllGoals();
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "client_node");
  CoutActionClient client;
  ros::spin();
  return 0;
}
