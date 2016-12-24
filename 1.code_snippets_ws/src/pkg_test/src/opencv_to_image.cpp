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


    ros::Rate r(1);
    while(ros::ok())
    {
//        capture >>  frame;
        frame.create(480, 640, CV_8UC1);
        for(int i = 0; i < 480; i++){
          for(int j = 0; j < 640; j++){
            frame.at<uchar>(i, j) = 127;
          }
        }
//        cv::cvtColor(frame, frame, CV_BGR2GRAY);
        cv::Mat mat;
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(55, 55));
        cv::erode(frame, mat, element);
        cv::imshow("raw", frame);
        cv::waitKey(10);
//        cv::Mat mat = frame.clone();

        ROS_INFO("!!!");
//        std::cout << mat << std::endl;
        cv_bridge::CvImage cv_image;
        cv_image.header.stamp = ros::Time::now();
        cv_image.encoding = sensor_msgs::image_encodings::RGB8;
        cv_image.image = frame;
//        cv::imshow("raw", cv_image.image);
        pub.publish(cv_image.toImageMsg());
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}

