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

    _detect->create(detector.c_str());
    _descriptor->create(descriptor.c_str());

    _detect->detect(frame_pair.rgb, frame_pair.kp);
    _descriptor->compute(frame_pair.rgb, frame_pair.kp, frame_pair.desp);

    return;
}

ResultOfPnP estimateMotion(FramePair &frame_pair1, FramePair &frame_pair2, CameraIntrinsicParams &camera)
{
    cv::FileStorage param_file("../param/param.yaml", cv::FileStorage::READ);
    if(!param_file.isOpened())
        std::cerr << "Can't open the param." << std::endl;
    camera.cx = (double)param_file["cx"];
    camera.cy = (double)param_file["cy"];
    camera.fx = (double)param_file["fx"];
    camera.fy = (double)param_file["fy"];
    camera.scale = (double)param_file["scale"];

    //粗匹配
    std::vector<cv::DMatch> matches;
    cv::FlannBasedMatcher matcher;
    matcher.match(frame_pair1.desp, frame_pair2.desp, matches);

    std::cout << "find total " << matches.size() << " matches." << std::endl;

    //精匹配
    std::vector<cv::DMatch> good_matches;
    double min_dist = 9999;
    double good_match_threshold = (double)param_file["good_match_threshold"];
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
    std::cout << "good matches: " <<good_matches.size() << std::endl;

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

    ResultOfPnP result;
    result.rvec = rvec;
    result.tvec = tvec;
    result.inliers = inliers.rows;

    return result;

}

























