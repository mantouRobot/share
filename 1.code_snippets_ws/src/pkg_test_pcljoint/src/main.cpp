#include <sstream>
#include <fstream>
#include <Eigen/Core>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <boost/lexical_cast.hpp>

/**
 * @brief main RGBD图像对拼接demo
 * @param      已知：图像对序列；和每对图像对应的相机位姿
 * @param      思路：1对图像对就是一真点云，将每真点云加入到一起即可
 * @return     算法：根据内参矩阵和rgb恢复基于相机系的X,Y并找到对应的Z
 */

struct CameraParams
{
  int width, height;
  double fx, fy, cx, cy;
  double scale;
};

//返回一个共享指针的引用和返回一个共享指针有什么区别？
pcl::PointCloud<pcl::PointXYZRGB>::Ptr img2PointCloud(cv::Mat &rgb, cv::Mat &depth, CameraParams &camera_params)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_point(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointXYZRGB point;
  for(int i = 0; i < camera_params.height; i++){
    for(int j = 0; j < camera_params.width; j++){
      if(depth.at<unsigned short>(i,j) == 0)
        continue;
      point.z = depth.at<unsigned short>(i,j)/camera_params.scale;
      point.x = (j - camera_params.cx)*point.z/camera_params.fx;
      point.y = (i - camera_params.cy)*point.z/camera_params.fy;
      point.b = rgb.at<cv::Vec3b>(i, j)[0];
      point.g = rgb.at<cv::Vec3b>(i, j)[1];
      point.r = rgb.at<cv::Vec3b>(i, j)[2];
      cloud_point->points.push_back(point);
    }
  }
  std::cout << "local ptr: " << cloud_point->size() << std::endl;
  return cloud_point;
}

void getTransform(std::string &line, Eigen::Vector3f &offset, Eigen::Quaternionf &rotation)
{
  std::stringstream ss(line);
  double data[7] = {0};
  for(int i = 0; i < 7; i++)
  {
    std::string str;
    ss >> str;
    data[i] = boost::lexical_cast<double>(str);
  }
  offset[0] = data[0];
  offset[1] = data[1];
  offset[2] = data[2];
  rotation.x() = data[3];
  rotation.y() = data[4];
  rotation.z() = data[5];
  rotation.w() = data[6];
}

int main(int argc, char ** argv)
{
  CameraParams camera_params;
  camera_params.width = 640;
  camera_params.height = 480;
  camera_params.fx = 518.0;
  camera_params.fy = 519.0;
  camera_params.cx = 325.5;
  camera_params.cy = 253.5;
  camera_params.scale = 1000.0;

  std::ifstream file;
  file.open("/home/mantou/share/1.code_snippets_ws/src/pkg_test_pcljoint/data/pose.txt");

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);

  for(int i = 0; i < 5; ++i)
  {
    std::stringstream ss1,ss2;
    ss1 << "/home/mantou/share/1.code_snippets_ws/src/pkg_test_pcljoint/data/color/" << i+1 << ".png";
    ss2 << "/home/mantou/share/1.code_snippets_ws/src/pkg_test_pcljoint/data/depth/" << i+1 << ".pgm";
    std::string str1 = ss1.str();
    std::string str2 = ss2.str();
    cv::Mat rgb, depth;
    rgb = cv::imread(str1);
    depth = cv::imread(str2);
    if(!rgb.data || !depth.data)
    {
      std::cerr << "rgb/depth img read error!" << std::endl;
      return 1;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>),
                                           cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
       cloud_in = img2PointCloud(rgb, depth, camera_params);

    Eigen::Vector3f offset;
    Eigen::Quaternionf rotation;
    std::string line;
    std::getline(file, line, '\n');
    getTransform(line, offset, rotation);

    std::cout << cloud_in->size() << std::endl;
    std::cout << offset << std::endl;
    std::cout << rotation.coeffs().transpose() << std::endl;
    pcl::transformPointCloud<pcl::PointXYZRGB>(*cloud_in, *cloud_out, offset, rotation);
    *output += *cloud_out;
  }

  output->is_dense = false;
  pcl::io::savePCDFileBinary("main.pcd", *output);
  return 0;
}













