#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_main");
    tf::TransformListener tf(ros::Duration(5));
    costmap_2d::Costmap2DROS costmap("my_costmap", tf);
    ros::spin();
}
