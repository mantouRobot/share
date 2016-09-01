#include <vector>
#include <iostream>

#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <slamBase.h>

int main()
{
    cv::Mat rgb1, rgb2, depth1, depth2;
    rgb1 = cv::imread("../data/rgb1.png");
    rgb2 = cv::imread("../data/rgb2.png");
    depth1 = cv::imread("../data/depth1.png", -1);
    depth2 = cv::imread("../data/depth2.png", -1);

    if(!rgb1.data || !rgb2.data || !depth1.data || !depth2.data)
    {
        std::cerr << "Img read error!" << std::endl;
        return 1;
    }

    //声明特征提取器与描述子提取器
    cv::Ptr<cv::FeatureDetector> _detector;
    cv::Ptr<cv::DescriptorExtractor> _descriptor;

    //构建提取器
    cv::initModule_nonfree();
    _detector = cv::FeatureDetector::create("SIFT");
    _descriptor = cv::DescriptorExtractor::create("SIFT");

    std::vector<cv::KeyPoint> kp1, kp2;
    _detector->detect(rgb1, kp1);
    _detector->detect(rgb2, kp2);

    std::cout << "Key points of two image: " << kp1.size() << ", " << kp2.size() << std::endl;

    cv::Mat img_show;
    cv::drawKeypoints(rgb1, kp1, img_show, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow("Keypoints", img_show);
    cv::imwrite("../data/keypoints.png", img_show);
    cv::waitKey(0);
    cv::drawKeypoints(rgb2, kp2, img_show, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow("Keypoints", img_show);
    cv::waitKey(0);

    //计算描述子
    cv::Mat desp1, desp2;
    _descriptor->compute(rgb1, kp1, desp1);
    _descriptor->compute(rgb2, kp2, desp2);

    //匹配描述子
    std::vector<cv::DMatch> matches;
    cv::FlannBasedMatcher matcher;
    matcher.match(desp1, desp2, matches);
    std::cout << "Find total " << matches.size() << " matches." << std::endl;

    //可视化，显示匹配的特征
    cv::Mat img_match;
    cv::drawMatches(rgb1, kp1, rgb2, kp2, matches, img_match);
    cv::imshow("matches", img_match);
    cv::imwrite("../data/matches.png", img_match);
    cv::waitKey(0);

    //筛选匹配，去掉距离太大的
    //筛选准则：去掉大于四倍最小距离的匹配
    std::vector<cv::DMatch> good_matches;
    double min_dist = 9999;
    for(std::size_t i = 0; i < matches.size(); ++i)
    {
        if(matches[i].distance < min_dist)
            min_dist = matches[i].distance;
    }

    for(std::size_t i = 0; i < matches.size(); ++i)
    {
        if(matches[i].distance > 4.0 * min_dist)
            continue;
        good_matches.push_back(matches[i]);
    }

    //显示good match
    std::cout << "good match size: " << good_matches.size() << std::endl;
    cv::Mat img_good_match;
    cv::drawMatches(rgb1, kp1, rgb2, kp2, good_matches, img_good_match);
    cv::imshow("matches", img_good_match);
    cv::imwrite("../data/goodmatched.png", img_good_match);
    cv::waitKey(0);

    //计算图像间的运动关系
    //关键函数： cv::solvePnpRansac(), 该函数需要两帧图像的三维点向量

    //第一帧的三维点
    std::vector<cv::Point3f> pts_obj;
    //第二帧的像素点
    std::vector<cv::Point2f> pts_img;

    //相机内参
    CameraIntrinsicParams camera;
    camera.cx = 325.5;
    camera.cy = 253.5;
    camera.fx = 518.0;
    camera.fy = 519.0;
    camera.scale = 1000.0;

    for(std::size_t i = 0; i < good_matches.size(); ++i)
    {
        cv::Point2f p = kp1[good_matches[i].queryIdx].pt;
        ushort d = depth1.ptr<ushort>(int(p.y))[int(p.x)];
        if(d == 0)
            continue;
        cv::Point3f pt_uvd(p.x, p.y, d);//(u, v, d)->(x, y, z)
        cv::Point3f pt_xyz = point2dTo3d(pt_uvd, camera);
        pts_obj.push_back(pt_xyz);

        pts_img.push_back(kp2[good_matches[i].trainIdx].pt);
    }

    double camera_matrix_data[3][3] = {
        {camera.fx, 0, camera.cx},
        {0, camera.fy, camera.cy},
        {0, 0, 1}
        };

    //构建相机矩阵
    cv::Mat cameraMatrix(3, 3, CV_64F, camera_matrix_data);
    cv::Mat rvec, tvec, inliers;

    //求解PnP
    cv::solvePnPRansac(pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 100, inliers);

    std::cout << "inliers: " << inliers.rows << std::endl;
    std::cout << "R=" << rvec << std::endl;
    std::cout << "T=" << tvec << std::endl;

    //画出inliers匹配
    std::vector<cv::DMatch> matchesShow;
    for(std::size_t i = 0; i < inliers.rows; ++i)
    {
        matchesShow.push_back(good_matches[inliers.ptr<int>(i)[0]]);
    }
    cv::drawMatches(rgb1, kp1, rgb2, kp2, matchesShow, img_good_match);
    cv::imshow("inlier matches", img_good_match);
    cv::imwrite("../data/inliers.png", img_good_match);
    cv::waitKey(0);


}//end of file


