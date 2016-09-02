#include "slamBase.h"
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

int main()
{
    FramePair frame1, frame2;
    frame1.rgb = cv::imread("../data/rgb1.png");
    frame1.depth = cv::imread("../data/depth1.png", -1);
    frame2.rgb = cv::imread("../data/rgb2.png");
    frame2.depth = cv::imread("../data/depth2.png", -1);
    if(!frame1.rgb.data || !frame1.depth.data || !frame2.rgb.data || !frame2.depth.data)
        cerr << "no img." << endl;

    cv::FileStorage param_file("../param/param.yaml", cv::FileStorage::READ);
    if(!param_file.isOpened())
        std::cerr << "Can't open the param." << std::endl;
    string detector = (string)param_file["detector"];
    string descriptor = (string)param_file["descriptor"];

    extractKeyPointAndDesp(frame1, detector, descriptor);
    extractKeyPointAndDesp(frame2, detector, descriptor);

    cout << "key points detect done." << endl;

    //算相对运动
    ResultOfPnP result;
    CameraIntrinsicParams camera;
    camera.cx = (double)param_file["cx"];
    camera.cy = (double)param_file["cy"];
    camera.fx = (double)param_file["fx"];
    camera.fy = (double)param_file["fy"];
    camera.scale = (double)param_file["scale"];
    result = estimateMotion(frame1, frame2, camera);//求解PnP
    cout << result.rvec << endl;
    cout << result.tvec << endl;

    //处理result
    //将旋转向量转化为旋转矩阵
    cv::Mat R;
    cv::Rodrigues(result.rvec, R);

    Eigen::Matrix3d r;
    cv::cv2eigen(R, r);

    cout << r << endl;

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd angle(r);
    cout << "translation" << endl;
    Eigen::Translation<double, 3> trans(result.tvec.at<double>(0, 0),
                                        result.tvec.at<double>(0, 1),
                                        result.tvec.at<double>(0, 2)
                                        );
    T = angle;
    T(0, 3) = result.tvec.at<double>(0, 0);
    T(1, 3) = result.tvec.at<double>(0, 1);
    T(2, 3) = result.tvec.at<double>(0, 2);

    //image转点云
    cout << "converting image to clouds." << endl;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1 = image2PointCloud(frame1.rgb, frame1.depth, camera);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 = image2PointCloud(frame2.rgb, frame2.depth, camera);

    //合并点云
    cout << "combining clouds." << endl;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::transformPointCloud(*cloud1, *output, T.matrix());
    *output += *cloud2;
    pcl::io::savePCDFile("../data/result.pcd", *output);
    cout << "Final result saved." << endl;

    pcl::visualization::CloudViewer viewer("viewer");
    viewer.showCloud(output);
    while(!viewer.wasStopped())
    {}










    return 0;














}






