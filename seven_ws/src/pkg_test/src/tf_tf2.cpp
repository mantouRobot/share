#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "node_tf_tf2");
  ros::NodeHandle nh;
  tf2_ros::StaticTransformBroadcaster static_tf;

  geometry_msgs::TransformStamped data;
  tf2::Quaternion q;
  q.setRPY(30, 30, 30);
  data.header.stamp = ros::Time::now();
  data.header.frame_id = "parent";
  data.child_frame_id = "children";
  data.transform.translation.x = 1;
  data.transform.translation.y = 2;
  data.transform.translation.z = 3;
  data.transform.rotation.x = q.x();
  data.transform.rotation.y = q.y();
  data.transform.rotation.z = q.z();
  data.transform.rotation.w = q.w();
  static_tf.sendTransform(data);
  ros::spin();
  return 0;
}
