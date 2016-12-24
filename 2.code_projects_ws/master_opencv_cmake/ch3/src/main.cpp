#include <opencv2/opencv.hpp>

void findMarker(cv::Mat &img_in)
{
  //转为灰度
  cv::cvtColor(img_in, img_in, CV_BGRA2GRAY);
  //阈值化：151这个值需要根据被检测物体的大小确定。值越小，自适应阈值越接近边缘检测。最后一个值作为噪声滤除
  cv::adaptiveThreshold(img_in, img_in, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 151, 7);
  //轮廓检测
  std::vector<std::vector<cv::Point> > all_contours;
}

int main(int argc, char** argv)
{
  cv::VideoCapture capture(0);
  while(1)
  {
    cv::waitKey(50);
    cv::Mat frame;
    capture >> frame;
    findMarker(frame);
    cv::imshow("frame", frame);
  }
  return 0;
}
