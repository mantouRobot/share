#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_rviz_node");
  ros::NodeHandle nh;
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/my_marker", 1);
  visualization_msgs::Marker marker;
  ros::Rate r(0.01);
//  while(ros::ok())
//  {
  ros::Rate r_2(1);
  r_2.sleep();
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";
    marker.ns = "my_ns";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 2;
    marker.pose.orientation.w = 1.0;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.color.g = 1.0f;
    marker.color.a = 1.0;
    geometry_msgs::Point tmp;
    tmp.x = 1.5;
    marker.points.push_back(tmp);
    marker_pub.publish(marker);
    r.sleep();
//  }
    ros::spin();
  return 0;
}
