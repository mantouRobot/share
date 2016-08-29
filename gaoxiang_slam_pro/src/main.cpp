#include <iostream>
#include <string>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


int main(int argc, char** argv)
{
     double camera_factor = 1000;
     double camera_cx = 325.5;
     double camera_cy = 253.5;
     double camera_fx = 518.0;
     double camera_fy = 519.0;

    cv::Mat img_rgb, img_depth;
    img_rgb = cv::imread("../data/rgb.png");
    img_depth = cv::imread("../data/depth.png", -1);//-1表示原始数据不做任何修改
    if(!img_rgb.data || !img_depth.data)
    {
        std::cerr << "Image data read error!." << std::endl;
        return 1;
    }
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

    //下面生成各种三维点存入点云

    for(int m = 0; m < img_depth.rows; ++m){
        for(int n = 0; n < img_depth.cols; ++n){
            ushort d = img_depth.ptr<ushort>(m)[n] ;/// camera_factor;
            if(d == 0)
                continue;
            pcl::PointXYZRGBA point;
            point.z = (double)(d) / camera_factor;
            point.x = (n - camera_cx) * point.z / camera_fx;
            point.y = (m - camera_cy) * point.z / camera_fy;
            point.b = img_rgb.ptr<uchar>(m)[n*3];
            point.g = img_rgb.ptr<uchar>(m)[n*3+1];
            point.r = img_rgb.ptr<uchar>(m)[n*3+2];
            pCloud->points.push_back(point);
        }
    }

    pCloud->height = 1;
    pCloud->width = pCloud->points.size();
    std::cout << "Point cloud size = "<< pCloud->points.size() << std::endl;
    pCloud->is_dense = false;
    pcl::io::savePCDFile("../data/pointcloud.pcd", *pCloud);
    pCloud->points.clear();

    return 0;







}
