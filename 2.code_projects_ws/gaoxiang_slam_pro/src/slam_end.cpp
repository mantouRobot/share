#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;

#include "slamBase.h"
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

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

    //g2o的初始化
    //选择优化方法
    typedef g2o::BlockSolver_6_3 SlamBlockSolver;
    typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    //初始化求解器
    SlamLinearSolver *linear_solver = new SlamLinearSolver();
    linear_solver->setBlockOrdering(false);
    SlamBlockSolver *block_solver = new SlamBlockSolver(linear_solver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(block_solver);
    g2o::SparseOptimizer global_optimizer;
    global_optimizer.setAlgorithm(solver);
    //不输出调试信息
    global_optimizer.setVerbose(false);

    //向global_optimizer增加第一个顶点
    g2o::VertexSE3 *v = new g2o::VertexSE3();
    v->setId(curr_index);
    v->setEstimate(Eigen::Isometry3d::Identity());//估计为单位矩阵
    v->setFixed(true);//第一个顶点固定，不用优化
    global_optimizer.addVertex(v);

    int last_index = curr_index;

    for(curr_index = start_index + 1; curr_index < 180; curr_index++)
    {
        cout << "Reading frame " << curr_index << endl;
        FramePair curr_frame = readFrame(curr_index);
        if(!curr_frame.rgb.data || !curr_frame.depth.data)
            cerr << "read img error!" << endl;
        extractKeyPointAndDesp(curr_frame, detector, descriptor);
        ResultOfPnP result = estimateMotion(last_frame, curr_frame, camera);
        //inliers不够，放弃该帧
        if(result.inliers < min_inliers)
            continue;
        //评估运动范围是否太大
        double norm = normofTransform(result.rvec, result.tvec);
        if(norm >= max_norm)
            continue;
        Eigen::Isometry3d T = cvMat2Eigen(result.rvec, result.tvec);
        cout << "T=" << T.matrix() << endl;

        //cloud = joinPointCloud(cloud, curr_frame, T, camera, voxel_grid);

        //向g2o中增加这个顶点与上一帧联系的边
        //顶点部分
        //顶点只需设定id即可
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId(curr_index);
        v->setEstimate(Eigen::Isometry3d::Identity());
        global_optimizer.addVertex(v);
        //边部分
        g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
        //连接此边的两个顶点id
        edge->vertices()[0] = global_optimizer.vertex(last_index);
        edge->vertices()[1] = global_optimizer.vertex(curr_index);
        //信息矩阵
        Eigen::Matrix<double, 6, 6> infomation = Eigen::Matrix<double, 6, 6>::Identity();
        //信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
        //因为pose是6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且相互独立，
        //那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
        infomation(0,0) = infomation(1,1) = infomation(2,2) = 100;
        infomation(3,3) = infomation(4,4) = infomation(5,5) = 100;
        //也可以将角度设大一些，表示对角度的估计更加准确
        edge->setInformation(infomation);
        //边的估计即是pnp求解之结果
        edge->setMeasurement(T);
        //将此边加入图中
        global_optimizer.addEdge(edge);

        if(is_visualize)
            viewer.showCloud(cloud);
        last_frame = curr_frame;
        last_index = curr_index;
    }
    //优化所有边
    cout << "optimizing pose graph, vertices: " << global_optimizer.vertices().size() << endl;
    global_optimizer.save("../data/result_before.g2o");
    global_optimizer.initializeOptimization();
    global_optimizer.optimize(100);//指定优化步数
    global_optimizer.save("../data/result_after.g2o");
    cout << "Optimization done." << endl;

    global_optimizer.clear();

    return 0;
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

