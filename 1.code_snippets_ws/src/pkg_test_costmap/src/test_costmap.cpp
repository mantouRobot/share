#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_costmap_node");
  ros::NodeHandle nh;
  tf::TransformListener tf_listener(ros::Duration(5));
  costmap_2d::Costmap2DROS my_global_costmap("global_costmap", tf_listener);
  ros::spin();
  return 0;
}
