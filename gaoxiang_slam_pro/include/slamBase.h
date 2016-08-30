# pragma once

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>

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
 * @brief point2dTo3d
 * @param point:(u,v,d)
 * @param camera
 * @return
 */
cv::Point3f& point2dTo3d(
        cv::Point3f &point,
        CameraIntrinsicParams &camera);

