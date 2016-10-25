#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <boost/thread.hpp>

class Gridmap2Mat
{
public:
  ros::NodeHandle nh_;
  ros::Subscriber gridmap_sub_;
  cv::Mat mat_src_, mat_erode_;
  boost::thread *thrd;

  Gridmap2Mat(ros::NodeHandle nh) :
    nh_(nh)
  {
    gridmap_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &Gridmap2Mat::Cb, this);
  }

  virtual ~Gridmap2Mat()
  {
    if (thrd != NULL) {
      thrd->join();
      delete thrd;
      thrd == NULL;
    }
  }

  void Cb(const nav_msgs::OccupancyGridConstPtr &gridmap)
  {
    ROS_INFO("Received the map.");
    int height = gridmap->info.height;
    int width = gridmap->info.width;

    ROS_INFO("%d, %d", height, width);
    mat_src_.create(height, width, CV_8UC1);
//    mat_erode_.create(height, width, CV_8UC1);

    for(int i = 0; i < height; ++i){
      uchar *data = mat_src_.ptr<uchar>(i);
      for(int j = 0; j < width; ++j){
        int8_t tmp = gridmap->data[i*width + j];
        if(tmp == 100)
          tmp = 0;
        else if(tmp == 0)
          tmp = 255;
        else if(tmp == -1)
          tmp = 127;
        mat_src_.at<uchar>(height - i - 1,j) = tmp;
//        data[j] = gridmap->data[i*width + j];
      }
    }
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));
//    mat_src_.copyTo(mat_erode_);
    ROS_INFO("1");
    cv::Mat out;// = mat_src_.clone();
    ROS_INFO("2");
//    std::cout << mat_src_ << std::endl;
    cv::erode(mat_src_, out, element);
    ROS_INFO("3");
//    thrd = new boost::thread(boost::bind(&Gridmap2Mat::showThread, this));
    gridmap_sub_.shutdown();
  }

  void showThread()
  {
    ROS_INFO("Start the show thread.");
    while(ros::ok())
    {
      cv::imshow("raw", mat_src_);
//      cv::imshow("erode", mat_erode_);
      cv::waitKey(1000);
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  Gridmap2Mat gridmap_to_mat(nh);
//  cv::startWindowThread();
  ros::spin();
//  cv::destroyWindow("window");
  return 0;
}
