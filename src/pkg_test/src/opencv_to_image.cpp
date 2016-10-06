#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "node_opencv_image");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/opencv_to_msg", 1);
    cv::VideoCapture capture(0);
    cv::Mat frame;
    sensor_msgs::Image image;

    ros::Rate r(20);
    while(ros::ok())
    {
        capture >>  frame;
        cv_bridge::CvImage cv_image;
        cv_image.header.stamp = ros::Time::now();
        cv_image.encoding = sensor_msgs::image_encodings::RGB8;
        cv_image.image = frame;
        pub.publish(cv_image.toImageMsg());
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}

