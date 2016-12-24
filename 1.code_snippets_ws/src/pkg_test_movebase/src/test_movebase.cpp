#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_movebase_node");
  ros::NodeHandle nh;
  ros::Publisher plan_pub = nh.advertise<nav_msgs::Path>("/my_plan", 1);
  tf::TransformListener tf_listener(ros::Duration(5));
  costmap_2d::Costmap2DROS costmap("global_costmap", tf_listener);
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client("move_base", true);
  ros::ServiceClient make_plan_client = nh.serviceClient<nav_msgs::GetPlan>("/move_base_node/make_plan");
  nav_msgs::Path path;
  ros::Rate r(1);
  while(ros::ok())
  {
    nav_msgs::GetPlan plan;
    tf::Stamped<tf::Pose> tmp;
    costmap.getRobotPose(tmp);
    plan.request.start.header.frame_id = "map";
    plan.request.start.header.stamp = ros::Time::now();
    plan.request.start.pose.position.x = tmp.getOrigin().getX();
    plan.request.start.pose.position.y = tmp.getOrigin().getY();
    plan.request.start.pose.position.z = tmp.getOrigin().getZ();
    plan.request.start.pose.orientation.x = tmp.getRotation().getX();
    plan.request.start.pose.orientation.y = tmp.getRotation().getY();
    plan.request.start.pose.orientation.z = tmp.getRotation().getZ();
    plan.request.start.pose.orientation.w = tmp.getRotation().getW();

    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
    goal.pose.position.x = 2;
    goal.pose.orientation.w = 1.0;
    plan.request.goal = goal;
    plan.request.tolerance = 1.0;

    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();
    if(make_plan_client.call(plan))
    {
      path.poses = plan.response.plan.poses;
      plan_pub.publish(path);
    }
    else
      ROS_ERROR("Plan failed.");
    r.sleep();
    ros::spinOnce();
  }
  return 0;
}
