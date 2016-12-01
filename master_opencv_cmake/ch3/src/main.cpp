#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{
  cv::VideoCapture capture(0);
  while(1)
  {
    cv::waitKey(50);
    cv::Mat frame;
    capture >> frame;
    cv::imshow("frame", frame);
  }
  return 0;
}
