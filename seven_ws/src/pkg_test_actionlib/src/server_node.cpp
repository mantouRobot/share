#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <pkg_test_actionlib/CoutAction.h>

class CoutActionServer
{
public:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<pkg_test_actionlib::CoutAction> as_;
  pkg_test_actionlib::CoutGoal goal_;
  pkg_test_actionlib::CoutResult result_;

  CoutActionServer(ros::NodeHandle nh) :
    nh_(nh),
    as_(nh_, "server_node", boost::bind(&CoutActionServer::executeCB, this, _1), false)//bool: auto start
  {
    as_.start();
  }

  void executeCB(const pkg_test_actionlib::CoutGoalConstPtr &goal)//一个const都不能少
  {
    ros::Rate r(1);
    int tmp = goal->times;
    while(ros::ok())
    {
      if(as_.isPreemptRequested() ||  !ros::ok())
      {
        ROS_ERROR("Preempted!");
        as_.setPreempted();
        result_.result = false;
        break;
      }
      tmp++;
      ROS_INFO("Cout action: %d", tmp);
      r.sleep();
    }
//    as_.setSucceeded();
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "server_node");
  ros::NodeHandle nh;
  CoutActionServer as(nh);
  ros::spin();
  return 0;
}
