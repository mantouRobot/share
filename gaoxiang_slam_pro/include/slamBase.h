# pragma once

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <vector>
#include <string>

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















