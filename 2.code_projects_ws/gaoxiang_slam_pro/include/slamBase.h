# pragma once

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/core/eigen.hpp>

#include <vector>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

//the following are UBUNTU/LINUX ONLY terminal color
#define RESET "\033[0m"
#define BLACK "\033[30m" /* Black */
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m" /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m" /* Cyan */
#define WHITE "\033[37m" /* White */
#define BOLDBLACK "\033[1m\033[30m" /* Bold Black */
#define BOLDRED "\033[1m\033[31m" /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m" /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m" /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m" /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m" /* Bold White */

struct CameraIntrinsicParams
{
    double fx, fy, cx, cy, scale;
};

/**
 * @brief image2PointCloud
 * @param rgb
 * @param depth
 * @param camera
 * @return Ptr
 */
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr image2PointCloud(
        cv::Mat &rgb,
        cv::Mat &depth,
        CameraIntrinsicParams &camera);

/**
 * @brief point2dTo3d摄像机坐标系
 * @param point:(u,v,d)像素坐标系
 * @param camera
 * @return
 */
cv::Point3f point2dTo3d(
        cv::Point3f &point,
        CameraIntrinsicParams &camera);

/**
 * @brief The FRAME struct
 */
struct FramePair
{
    int frameID;
    cv::Mat rgb, depth;
    cv::Mat desp;//描述子
    std::vector<cv::KeyPoint> kp;//关键点
};

/**
 * @brief The ResultOfPnP struct
 */
struct ResultOfPnP
{
    cv::Mat rvec, tvec;
    int inliers;
};

/**
 * @brief extractKeyPointAndDesp
 * @param frame_pair
 * @param detector
 * @param descriptor
 */
void extractKeyPointAndDesp(FramePair &frame_pair, std::string detector, std::string descriptor);

/**
 * @brief estimateMotion
 * @param frame_pair1
 * @param frame_pair2
 * @param camera
 * @return
 */
ResultOfPnP estimateMotion(FramePair &frame_pair1, FramePair &frame_pair2, CameraIntrinsicParams &camera);

/**
 * @brief cvMat2Eigen
 * @param rvec
 * @param tvec
 * @return
 */
Eigen::Isometry3d cvMat2Eigen( cv::Mat& rvec, cv::Mat& tvec );

/**
 * @brief joinPointCloud
 * @param original
 * @param newFrame
 * @param T
 * @param camera
 * @return
 */
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr joinPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr original, FramePair& newFrame, Eigen::Isometry3d T, CameraIntrinsicParams& camera , double gridsize);












