#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <pkg_test_actionlib/CoutAction.h>

class CoutActionServer
{
public:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<pkg_test_actionlib::CoutAction> as_;
  actionlib::SimpleActionClient<pkg_test_actionlib::CoutAction> embed_ac_;
  boost::mutex embed_ac_lock_;

  pkg_test_actionlib::CoutGoal goal_;
  pkg_test_actionlib::CoutResult result_;

  CoutActionServer(ros::NodeHandle nh) :
    nh_(nh),
    embed_ac_("embed_server", true),
    as_(nh_, "server_node", boost::bind(&CoutActionServer::executeCB, this, _1), false)//bool: auto start
  {
    embed_ac_.waitForServer(ros::Duration(5));
    as_.registerPreemptCallback(boost::bind(&CoutActionServer::preemptCb, this));
    as_.start();
  }

  /**
   * @brief preemptCb 又一个客户请求则进入抢占回调。
   *                  在抢占回调里应该实现：
   *                    1. 取消本服务里的其他可能在执行的内层服务
   *                    2. 将as_设置为抢占即可结束当前服务
   *                    3. 如有需要，发布上一服务的结果（逻辑上为未完成）或此时的中间反馈值
   */
  void preemptCb()
  {
    ROS_ERROR("New goal will preempt and execute. Old goal will aborted!");
    boost::unique_lock<boost::mutex> lock(embed_ac_lock_);//生命结束，自动解锁
    embed_ac_.cancelGoalsAtAndBeforeTime(ros::Time::now());
    if(as_.isActive())
    {
      as_.setPreempted();//抢占
    }
  }

  /**
   * @brief executeCB 服务的执行函数。
   * @param goal
   */
  void executeCB(const pkg_test_actionlib::CoutGoalConstPtr &goal)//一个const都不能少
  {
    ros::Rate r(1);
    int tmp = goal->times;
    while(ros::ok() && as_.isActive())
    {
      pkg_test_actionlib::CoutGoal embed_goal;
      embed_goal.times = 5;
      embed_ac_.sendGoal(embed_goal);
      embed_ac_.waitForResult(ros::Duration(10));
      tmp++;
      ROS_INFO("Cout action: %d", tmp);
      if(as_.isActive() && tmp == 8)
      {
        as_.setSucceeded();
        break;
      }
      r.sleep();
    }
//    as_.setSucceeded();
//    as_.setPreempted();
//    as_.setAborted();
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
