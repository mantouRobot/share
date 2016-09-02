#include <iostream>
#include <string>
#include <slamBase.h>
#include <sstream>

using namespace std;

FramePair readFrame( int index );
double normofTransform( cv::Mat rvec, cv::Mat tvec )
{
    return fabs(min(cv::norm(rvec), 2*M_PI - cv::norm(rvec))) + fabs(cv::norm(tvec));
}

int main()
{
    CameraIntrinsicParams camera;
    int start_index, end_index;
    double voxel_grid;
    int min_good_match, min_inliers;
    double max_norm;
    string detector, descriptor;
    bool is_visualize;
    cv::FileStorage param("../param/param.yaml", cv::FileStorage::READ);

    camera.cx = (double)param["cx"];
    camera.cy = (double)param["cy"];
    camera.fx = (double)param["fx"];
    camera.fy = (double)param["fy"];
    camera.scale = (double)param["scale"];
    start_index = (int)param["start_index"];
    end_index = (int)param["end_index"];
    voxel_grid = (double)param["voxel_grid"];
    min_good_match = (int)param["min_good_match"];
    min_inliers = (int)param["min_inliers"];
    max_norm = (double)param["max_norm"];
    detector = (string)param["detector"];
    descriptor = (string)param["descriptor"];
    is_visualize = (int)param["visualize_pointcloud"];
    cout << "Initializing..." << endl;

    int curr_index = start_index;
    FramePair last_frame = readFrame( curr_index );
    extractKeyPointAndDesp(last_frame, detector, descriptor);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = image2PointCloud(last_frame.rgb, last_frame.depth, camera);

    pcl::visualization::CloudViewer viewer("viewer");

    for(curr_index = start_index + 1; curr_index < end_index; curr_index++)
    {
        cout << "Reading frame " << curr_index << endl;
        FramePair curr_frame = readFrame(curr_index);
        if(!curr_frame.rgb.data || !curr_frame.depth.data)
            cerr << "read img error!" << endl;
        extractKeyPointAndDesp(curr_frame, detector, descriptor);
        ResultOfPnP result = estimateMotion(last_frame, curr_frame, camera);
        if(result.inliers < min_inliers)
            continue;
        double norm = normofTransform(result.rvec, result.tvec);
        if(norm >= max_norm)
            continue;
        Eigen::Isometry3d T = cvMat2Eigen(result.rvec, result.tvec);
        cout << "T=" << T.matrix() << endl;

        cloud = joinPointCloud(cloud, curr_frame, T, camera, voxel_grid);

        if(is_visualize)
            viewer.showCloud(cloud);
        last_frame = curr_frame;
    }
}

FramePair readFrame( int index )
{
    stringstream ss;
    ss << "../data/rgb_png/" << index << ".png";
    string filename;
    ss >> filename;

    FramePair result;
    result.rgb = cv::imread(filename);

    ss.clear();
    filename.clear();

    ss << "../data/depth_png/" << index << ".png";
    ss >> filename;
    result.depth = cv::imread(filename, -1);

    return result;
}

