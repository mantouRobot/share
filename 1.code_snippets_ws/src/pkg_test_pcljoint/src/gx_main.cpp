#include <iostream>
#include <fstream>

#include <boost/format.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{
  std::vector<cv::Mat> color_imgs, depth_imgs;
  std::vector<Eigen::Isometry3d> poses;

  std::ifstream fin("/home/mantou/share/1.code_snippets_ws/src/pkg_test_pcljoint/data/pose.txt");
  if(!fin)
  {
    std::cerr << "找不到pose.txt" << endl;
    return 1;
  }

  for(int i = 0; i < 5; i++)
  {
    boost::format fmt("/home/mantou/share/1.code_snippets_ws/src/pkg_test_pcljoint/data/%s/%d.%s");
    color_imgs.push_back(cv::imread((fmt%"color"%(i+1)%"png").str()));
    depth_imgs.push_back(cv::imread((fmt%"depth"%(i+1)%"pgm").str()));

    double data[7] = {0};
    for(auto& d : data)
      fin >> d;//文件流直接转成了double
    Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
    Eigen::Isometry3d T(q);
    T.pretranslate(Eigen::Vector3d(data[0], data[1], data[2]));
    poses.push_back(T);
  }

  double cx = 325.5;
  double cy = 253.5;
  double fx = 518.0;
  double fy = 519.0;
  double scale = 1000.0;

  std::cout << "正在将图像转换为点云..." << endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  for(int i = 0; i < 5; i++)
  {
    cv::Mat color = color_imgs[i];
    cv::Mat depth = depth_imgs[i];
    Eigen::Isometry3d T = poses[i];
    for(int v = 0; v < color.rows; v++){
      for(int u = 0; u < color.cols; u++){
        unsigned int d = depth.ptr<unsigned short>(v)[u];
        if(d == 0) continue;
        Eigen::Vector3d point;
        point[2] = double(d)/scale;
        point[0] = (u-cx)*point[2]/fx;
        point[1] = (v-cy)*point[2]/fy;
        Eigen::Vector3d point_w = T*point;

        pcl::PointXYZRGB p;
        p.x = point_w[0];
        p.y = point_w[1];
        p.z = point_w[2];
        p.b = color.data[v*color.step + u*color.channels()];
        p.g = color.data[v*color.step + u*color.channels()+1];
        p.r = color.data[v*color.step + u*color.channels()+2];

        point_cloud->points.push_back(p);
      }
    }
  }

  point_cloud->is_dense = false;
  std::cout << "点云共有" << point_cloud->size() << "个点." << endl;
  pcl::io::savePCDFileBinary("map.pcd", *point_cloud);

  return 0;

}


























