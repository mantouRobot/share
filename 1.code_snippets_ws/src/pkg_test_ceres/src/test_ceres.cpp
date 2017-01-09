#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

//代价函数的计算模型
struct CURVE_FITTING_COST
{
  CURVE_FITTING_COST(double x, double y) : x_(x), y_(y) {}
  //残差的计算
  template<typename T>
  bool operator() (const T* const abc, T* residual)  //模型参数3维，残差1维
  {
    residual[0] = T(y_) - ceres::exp(abc[0]*T(x_)*T(x_)+abc[1]*T(x_)+abc[2]);
    return true;
  }

  const double x_, y_;
};

int main(int argc, char** argv)
{
  double a = 1.0, b = 2.0, c = 1.0; //真实参数值
  int N = 100;
  double w_sigma = 1.0; //噪声方差
  cv::RNG rng;
  double abc[3] = {0, 0, 0};

  std::vector<double> x_data, y_data;
  std::cout << "generating data: " << std::endl;

  for(int i = 0; i < N; ++i)
  {
    double x = i / 100.0;
    x_data.push_back(x);
    y_data.push_back(std::exp(a*x*x + b*x + c));
    std::cout << x_data[i] << " " << y_data[i] << std::endl;
  }

  //构建最小二乘问题
  ceres::Problem problem;
  for(int i = 0; i < N; i++)
  {
    problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(new CURVE_FITTING_COST(x_data[i], y_data[i])),
          nullptr, //核函数
          abc
          );
  }

  //配置求解器
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR; //增量方程求解
  options.minimizer_progress_to_stdout = true; //输出到cout

  ceres::Solver::Summary summary; //优化结果
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  ceres::Solve(options, &problem, &summary);
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  std::cout << "solve time cost = " << time_used.count() << " seconds." << std::endl;

  //输出结果
  std::cout << summary.BriefReport() << std::endl;
  std::cout << "estimated a, b, c = ";
  for(auto a : abc) std::cout << a << " ";
  std::cout << std::endl;

  return 0;
}
