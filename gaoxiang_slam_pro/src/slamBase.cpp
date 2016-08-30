#include <slamBase.h>


pcl::PointCloud<pcl::PointXYZRGBA>::Ptr image2PointCloud(cv::Mat &rgb, cv::Mat &depth, CameraIntrinsicParams &camera)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    for(int m = 0; m < rgb.rows; ++m){
        for(int n = 0; n < rgb.cols; ++n){
            double d = depth.ptr<ushort>(m)[n];
            if( d == 0)
                continue;
            pcl::PointXYZRGBA p;
            p.z = d / camera.scale;
            p.x = (m - camera.cx) * p.z / camera.fx;
            p.y = (n - camera.cy) * p.z / camera.fy;

            p.b = rgb.ptr<uchar>(m)[3*n];
            p.g = rgb.ptr<uchar>(m)[3*n + 1];
            p.r = rgb.ptr<uchar>(m)[3*n + 2];

            pCloud->push_back(p);
        }
    }
    pCloud->height = 1;
    pCloud->width = pCloud->points.size();
    pCloud->is_dense = false;
    return pCloud;
}

cv::Point3f& point2dTo3d(cv::Point3f &point, CameraIntrinsicParams &camera)
{
    point.z = point.z / camera.scale;
    point.x = (point.x - camera.cx) * point.z / camera.fx;
    point.y = (point.y - camera.cy) * point.z / camera.fy;
    return point;
}
