#include <slamBase.h>

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr image2PointCloud(cv::Mat &rgb, cv::Mat &depth, CameraIntrinsicParams &camera)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    for(int m = 0; m < rgb.rows; ++m){
        for(int n = 0; n < rgb.cols; ++n){
            ushort d = depth.ptr<ushort>(m)[n];
            if( d == 0)
                continue;
            pcl::PointXYZRGBA p;
            p.z = double(d) / camera.scale;
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

cv::Point3f point2dTo3d(cv::Point3f &point, CameraIntrinsicParams &camera)
{
    cv::Point3f p;
    p.z = double(point.z) / camera.scale;
    p.x = (point.x - camera.cx) * p.z / camera.fx;
    p.y = (point.y - camera.cy) * p.z / camera.fy;
    return p;
}

/**
 * @brief extractKeyPointAndDesp
 * 1.initNonfree_module
 * 2.creat detect descriptor
 * 3.detect keypoint
 * 4.computer desp
 */
void extractKeyPointAndDesp(FramePair &frame_pair, std::string detector, std::string descriptor)
{
    cv::initModule_nonfree();
    cv::Ptr<cv::FeatureDetector> _detect;
    cv::Ptr<cv::DescriptorExtractor> _descriptor;

    _detect = cv::FeatureDetector::create(detector.c_str());
    _descriptor = cv::DescriptorExtractor::create(descriptor.c_str());

    _detect->detect(frame_pair.rgb, frame_pair.kp);
    _descriptor->compute(frame_pair.rgb, frame_pair.kp, frame_pair.desp);

    return;
}

ResultOfPnP estimateMotion(FramePair &frame_pair1, FramePair &frame_pair2, CameraIntrinsicParams &camera)
{
    //粗匹配
    std::vector<cv::DMatch> matches;
    cv::FlannBasedMatcher matcher;
    matcher.match(frame_pair1.desp, frame_pair2.desp, matches);

    std::cout << "find total " << matches.size() << " matches." << std::endl;

    //精匹配
    std::vector<cv::DMatch> good_matches;
    double min_dist = 9999;
    double good_match_threshold = 4;
    for(int i = 0; i < matches.size(); ++i)
    {
        if(matches[i].distance < min_dist)
            min_dist = matches[i].distance;
    }
    for(int i = 0; i < matches.size(); ++i)
    {
        if(matches[i].distance < good_match_threshold*min_dist)
            good_matches.push_back(matches[i]);
    }

    ResultOfPnP result;
    std::cout << "good matches: " <<good_matches.size() << std::endl;
    if(good_matches.size() <= 5)
    {
        result.inliers = -1;
        return result;
    }
    //PnP
    std::vector<cv::Point3f> pts_obj;
    std::vector<cv::Point2f> pts_img;
    for(int i = 0; i < good_matches.size(); i++)
    {
        cv::Point2f p = frame_pair1.kp[good_matches[i].queryIdx].pt;
        ushort d = frame_pair1.depth.ptr<ushort>((int)p.y)[(int)p.x];
        if( d == 0 )
            continue;
        cv::Point3f p_xyz, p_uvd(p.x, p.y, d);
        p_xyz = point2dTo3d(p_uvd, camera);
        pts_obj.push_back(p_xyz);

        pts_img.push_back(frame_pair2.kp[good_matches[i].trainIdx].pt);
    }

    double camera_matrix_data[3][3] = {
        {camera.fx, 0, camera.cx},
        {0, camera.fy, camera.cy},
        {0, 0, 1}
        };
    std::cout << "solving pnp..." << std::endl;

    cv::Mat cameraMatrix(3, 3, CV_64F, camera_matrix_data);
    cv::Mat rvec, tvec, inliers;
    cv::solvePnPRansac(pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 100, inliers);


    result.rvec = rvec;
    result.tvec = tvec;
    result.inliers = inliers.rows;

    return result;

}

Eigen::Isometry3d cvMat2Eigen(cv::Mat &rvec, cv::Mat &tvec)
{
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    Eigen::Matrix3d r;
    cv::cv2eigen(R, r);
    Eigen::AngleAxisd angle(r);
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T = angle;

    Eigen::Translation<double, 3> trans(tvec.at<double>(0,0), tvec.at<double>(0,1),tvec.at<double>(0,2));
    T(0,3) = tvec.at<double>(0,0);
    T(1,3) = tvec.at<double>(0,1);
    T(2,3) = tvec.at<double>(0,2);
    return T;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr joinPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr original,
                                                       FramePair &newFrame, Eigen::Isometry3d T,
                                                       CameraIntrinsicParams &camera,
                                                       double gridsize)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr newcloud = image2PointCloud(newFrame.rgb, newFrame.depth, camera);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::transformPointCloud(*original, *output, T.matrix());
    *newcloud += *output;

    //voxel grid 滤波降采样
    static pcl::VoxelGrid<pcl::PointXYZRGBA> voxel;
    voxel.setLeafSize( gridsize, gridsize, gridsize );
    voxel.setInputCloud( newcloud );
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp( new pcl::PointCloud<pcl::PointXYZRGBA> );
    voxel.filter( *tmp );
    return tmp;
}









