#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <pkg_test_actionlib/CoutAction.h>

class EmbedCoutSrv
{
public:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<pkg_test_actionlib::CoutAction> as_;
  EmbedCoutSrv(ros::NodeHandle nh) :
    nh_(nh),
    as_(nh, "embed_server", boost::bind(&EmbedCoutSrv::executeCb, this, _1), false)
  {
    as_.registerPreemptCallback(boost::bind(&EmbedCoutSrv::preemptCb, this));
    as_.start();
  }

  void executeCb(const pkg_test_actionlib::CoutGoalConstPtr &goal)
  {
    int tmp = goal->times;
    ros::Rate r(1);
    while(ros::ok() && as_.isActive())
    {
      ROS_INFO("embed server --: %d", tmp--);
      if(tmp == -1)
      {
        as_.setSucceeded();
        break;
      }
      r.sleep();
    }
//    as_.setSucceeded();
//    as_.setAborted();
//    as_.setPreempted();
  }

  void preemptCb()
  {
    ROS_ERROR("Embed srv preempted!");
    if(as_.isActive())
      as_.setPreempted();
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "embed_srv_node");
  ros::NodeHandle nh;
  EmbedCoutSrv embed_srv(nh);
  ros::spin();
  return 0;
}
