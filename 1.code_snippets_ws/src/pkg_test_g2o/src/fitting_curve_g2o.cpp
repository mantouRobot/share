#include <iostream>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <chrono>

using namespace  std;

//曲线模型的顶点，模板参数：优化变量维度和数据类型
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  virtual void setToOriginImpl()
  {
    _estimate << 0, 0, 0;
  }

  virtual void oplusImpl(const double* update)
  {
    _estimate += Eigen::Vector3d(update);
  }

  virtual bool read(istream &in){}
  virtual bool write(ostream &os) const {}
};

//error model:
class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}

  virtual void computeError()
  {
    const CurveFittingVertex *v = static_cast<const CurveFittingVertex*>(_vertices[0]);
    const Eigen::Vector3d abc = v->estimate();
    _error(0, 0) = _measurement - std::exp(abc(0,0)*_x*_x+abc(1,0)*_x+abc(2,0));
  }

  virtual bool read(istream &is) {}
  virtual bool write(ostream &os) const {}

  double _x;
};

int main(int argc, char** argv)
{
  double a = 1, b = 2, c = 1;
  int N = 100;
  double w_sigma = 1.0;
  cv::RNG rng;
  double abc[3] = {0, 0, 0};
  vector<double> x_data, y_data;
  for(int i = 0; i < N; i++)
  {
    double x = i / 100.0;
    x_data.push_back(x);
    y_data.push_back(exp(a*x*x + b*x + c)+rng.gaussian(w_sigma));
    cout << x_data[i] << " " << y_data[i] << endl;
  }

  //construct g2o
  //matrix block, edge 3, error 1
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<3,1> > Block;
  //zengliang equa
  Block::LinearSolverType *linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
  Block* solve_ptr = new Block(linearSolver);
  //method: gn, lm, dogleg
  g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solve_ptr);
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);

  //add vertex
  CurveFittingVertex *v = new CurveFittingVertex();
  v->setEstimate(Eigen::Vector3d(0,0,0));
  v->setId(0);
  optimizer.addVertex(v);

  //add edge
  for(int i = 0; i < N; i++)
  {
    CurveFittingEdge *edge = new CurveFittingEdge(x_data[i]);
    edge->setId(i);
    edge->setVertex(0, v);
    edge->setMeasurement(y_data[i]);
    //info matrix, sigma-1
    edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity()*1/(w_sigma*w_sigma));
    optimizer.addEdge(edge);
  }

  //do optimiza
  cout << "start optimization" << endl;
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  optimizer.initializeOptimization();
  optimizer.optimize(100);
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  std::cout << "solve time cost = " << time_used.count() << " seconds." << std::endl;

  //out
  Eigen::Vector3d abc_estimate = v->estimate();
  cout << "estimated model: " << abc_estimate.transpose() << endl;
  return 0;
}






















