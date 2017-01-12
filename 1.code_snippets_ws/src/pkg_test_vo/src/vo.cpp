#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/format.hpp>

using namespace std;

int main(int argc, char** argv)
{
//  if(argc != 3)
//  {
//    cout << "请输入： .exe img_path1 img_path2" << endl;
//    return 1;
//  }

  //图片数据读入
  std::vector<cv::Mat> v_img;
  for(int i = 0; i < 10000; i++)
  {
    boost::format format("/home/mantou/sensor_data/demo_room_img/rgb/%d.jpg");
    cv::Mat rgb_raw = cv::imread((format%(i+1)).str(), CV_LOAD_IMAGE_COLOR);
    if(rgb_raw.data == NULL)
    {
      cerr << "no img" << i << " data." << endl;
      break;
    }
    v_img.push_back(rgb_raw);
    cout << "img" << i << endl;
  }

  //特征点定义
  cv::OrbFeatureDetector orb_detect;
  std::vector<cv::KeyPoint> kp;

  //计算特征点并查看每张图片的原始ORB特征点
//  for(int i = 0; i < v_img.size(); i++)
//  {
//    orb_detect.detect(v_img[i], kp);
//    cv::Mat out_img;
//    cv::drawKeypoints(v_img[i], kp, out_img);
//    cv::imshow("orb_kp", out_img);
//    cout << i << ".jpg" << "have " << kp.size() << " orb keypoints." << endl;
//    cv::waitKey(0);
//  }

  //计算连续两帧图片的特征描述符并匹配
  std::vector<cv::KeyPoint> kp1, kp2;
  cv::OrbDescriptorExtractor orb_extractor;
  cv::Mat descriptor1, descriptor2;

  for(int i = 0; i < v_img.size() - 1; i++)
  {
    cv::Mat img1 = v_img[i].clone();
    cv::Mat img2 = v_img[i+1].clone();
    orb_detect.detect(img1, kp1);
    orb_detect.detect(img2, kp2);
    orb_extractor.compute(img1, kp1, descriptor1);
    orb_extractor.compute(img2, kp2, descriptor2);
    //使用BRIEF Hamming距离，对两幅图像中的描述子进行匹配
    std::vector<cv::DMatch> matchers;
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    matcher.match(descriptor1, descriptor2, matchers);
    //找出所有匹配之间的最小距离和最大距离，即找到最相似和最不相似的两组点之间的距离
    cout << "descriptors.size == " << descriptor1.rows << endl;
    cout << "matchers.size == " << matchers.size() << endl;
    double min_dist = 10000, max_dist = 0;
    for(int i = 0; i < descriptor1.rows; i++)
    {
      double dist = matchers[i].distance;//matchers.size == descriptor.size, matchers[i]是一个结构体，包含queryIdx, trainIdx, imgIdx, distance。distance是暴力匹配后的结果
      if(dist > max_dist) max_dist = dist;
      if(dist < min_dist) min_dist = dist;
    }
    cout << "Max dist: " << max_dist << endl;
    cout << "Min dist: " << min_dist << endl;
    //当描述子之间的距离大于两倍的最小距离时，即认为匹配有误
    //但有时候最小距离会非常小，设置一个经验值作为下限
    std::vector<cv::DMatch> good_matchers;
    for(int i = 0; i < matchers.size(); i++)
    {
      if(matchers[i].distance < std::max(min_dist*2, 30.0))
        good_matchers.push_back(matchers[i]);
    }
    //绘制匹配结果
    cv::Mat img_match, img_goodmatch;
    cv::drawMatches(img1, kp1, img2, kp2, matchers, img_match);
    cv::drawMatches(img1, kp1, img2, kp2, good_matchers, img_goodmatch);
    cv::imshow("所有匹配点对", img_match);
    cv::imshow("过滤后匹配点对", img_goodmatch);
    cv::waitKey(0);

  }


}
















//cv::Mat img1 = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
//cv::Mat img2 = cv::imread(argv[2], CV_LOAD_IMAGE_COLOR);

//if(img1.data == NULL || img2.data == NULL)
//{
//  cout << "no img data." << endl;
//  return 1;
//}
//cv::OrbFeatureDetector orb;
//std::vector<cv::KeyPoint> kp1, kp2;
//cv::Mat descritors1, descriptors2;

//orb.detect(img1, kp1);
//cv::Mat out_img1, out_img2;
//cv::drawKeypoints(img1, kp1, out_img1);
//cv::imshow("raw_kp", out_img1);
//cv::waitKey(0);
