#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/format.hpp>

using namespace std;

/**
 * @brief pixel2cam 将像素坐标根据内参矩阵转换到归一化平面坐标
 * @param p
 * @param K
 * @return
 */
cv::Point2d pixel2cam ( const cv::Point2d& p, const cv::Mat& K )
{
  return cv::Point2d(((p.x - K.at<double>(0,2)) / K.at<double>(0,0)), (p.y - K.at<double>(1,2)) / K.at<double>(1,1));
}

/**
 * @brief poseEstimate2dTo2d 2dTo2d使用极线约束，求出本质矩阵，再分解得到R，t
 * @param kp1
 * @param kp2
 * @param good_matchers
 * @param R
 * @param t
 */
void poseEstimate2dTo2d(std::vector<cv::KeyPoint> kp1, std::vector<cv::KeyPoint> kp2, std::vector<cv::DMatch> good_matchers,
                        cv::Mat &R, cv::Mat &t)
{
  //相机内参
  cv::Mat K = (cv::Mat_<double>(3,3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

  //把传入的匹配点从matcher中找到索引号，再从kp中根据这个索引号找到像素坐标
  std::vector<cv::Point2f> points1, points2;
  for(int i = 0; i < good_matchers.size(); i++)
  {
    points1.push_back(kp1[good_matchers[i].queryIdx].pt);
    points2.push_back(kp2[good_matchers[i].trainIdx].pt);
  }

  //计算基础矩阵
  cv::Mat fundamental_matrix;
  fundamental_matrix = cv::findFundamentalMat(points1, points2, CV_FM_8POINT);
  cout << "fundamental_matrix is " << endl << fundamental_matrix << endl;

  //计算本质矩阵opencv2没有findEssential
  cv::Point2d principal_point(325.1, 249.7);//光心
  int focal_length = 521;//焦距
  cv::Mat essential_matrix;
  essential_matrix = cv::findEssentialMat(points1, points2, focal_length, principal_point, cv::RANSAC);
  cout << "essential_matrix is " << endl << essential_matrix << endl;

  //计算单应矩阵
  cv::Mat homography_matrix;
  homography_matrix = cv::findHomography(points1, points2, cv::RANSAC, 3, cv::noArray(), 2000, 0.99);
  cout << "homograhpy_matrix is " << endl << homography_matrix << endl;


  //从基础矩阵回复R，t
  cv::recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
  cout << "R is " << endl << R << endl;
  cout << "t is " << endl << t << endl;
}


int main(int argc, char** argv)
{
  if(argc != 3)
  {
    cout << "请输入： .exe img_path1 img_path2" << endl;
    return 1;
  }

  //图片数据读入
  cv::Mat img1 = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  cv::Mat img2 = cv::imread(argv[2], CV_LOAD_IMAGE_COLOR);
  if(img1.data == NULL || img2.data == NULL)
  {
    cerr << "no img data!" << endl;
    return 1;
  }

  //特征点提取，并通过暴力匹配配对，再滤出错误匹配，显示
  cv::Ptr<cv::ORB> orb = cv::ORB::create(500, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
  std::vector<cv::KeyPoint> kp1, kp2;
  cv::Mat descriptor1, descriptor2;//描述子一张图片的一个500行的矩阵，每一行是一个特征点的描述子，ORB描述子是32*uchar
  std::vector<cv::DMatch> raw_matchs;
  std::vector<cv::DMatch> good_matchs;

  orb->detect(img1, kp1);
  orb->detect(img2, kp2);
  orb->compute(img1, kp1, descriptor1);
  orb->compute(img2, kp2, descriptor2);

  cv::BFMatcher bf(cv::NORM_HAMMING);//orb 为brief描述子，一般用norm_hamming
  bf.match(descriptor1, descriptor2, raw_matchs);//descriptor1作为query从0开始递增，找到在train(descriptor2)中的最小距离作为配对，并记录这个距离
  double max_dist = 0, min_dist = 10000;
  for(int i = 0; i < raw_matchs.size(); i++)//找到最大最小距离
  {
    if(raw_matchs[i].distance > max_dist) max_dist = raw_matchs[i].distance;
    if(raw_matchs[i].distance < min_dist) min_dist = raw_matchs[i].distance;
  }
  for(int i = 0; i < raw_matchs.size(); i++)//滤除错误匹配点
  {
    if(raw_matchs[i].distance < std::max(min_dist*2, 30.0))
      good_matchs.push_back(raw_matchs[i]);
  }
  cout << "--- Max dist :" << max_dist << endl;
  cout << "--- Min dist :" << min_dist << endl;
  cout << "一共找到了好的匹配对数 " << good_matchs.size() << endl;
//  cv::Mat outimg;
//  cv::drawMatches(img1, kp1, img2, kp2, good_matchs, outimg);
//  cv::imshow("good_matchers", outimg);

  //计算E/F/H
  cv::Mat R, t;
  poseEstimate2dTo2d(kp1, kp2, good_matchs, R, t);

  //验证E=t^R*scale
  cv::Mat t_x = (cv::Mat_<double>(3,3) << 0, -t.at<double>(2,0), t.at<double>(1,0), t.at<double>(2,0), 0, -t.at<double>(0,0), -t.at<double>(1,0), t.at<double>(0,0), 0);
  cout << "t^R=" << endl << t_x*R << endl;

  //验证对极约束
  cv::Mat K = (cv::Mat_<double>(3,3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
  for(auto m : good_matchs)
  {
    cv::Point2d pt1 = pixel2cam(kp1[m.queryIdx].pt, K);
    cv::Mat y1 = (cv::Mat_<double>(3,1) << pt1.x, pt1.y, 1);
    cv::Point2d pt2 = pixel2cam(kp2[m.trainIdx].pt, K);
    cv::Mat y2 = (cv::Mat_<double>(3,1) << pt2.x, pt2.y, 1);
    cv::Mat d = y2.t()*t_x*R*y1;
    cout << "epiplar constraint = " << d << endl;
//    break;
  }

  cv::waitKey(0);
}
















