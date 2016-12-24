#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sophus/so3.h>
#include <sophus/se3.h>

using namespace std;
int main()
{
  //SO3,Eigen::Vector3d = SO3.log(),纯旋转特殊正交群
  //about z axis 90 degree
  Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,0,1)).toRotationMatrix();

  //下述方式等价
  Sophus::SO3 SO3_R(R);
  Sophus::SO3 SO3_v(0,0,M_PI/2);
  Sophus::SO3 SO3_q(Eigen::Quaterniond(R));
  cout << "SO3 from matrix: " << SO3_R << endl;
  cout << "SO3 from vector: " << SO3_v << endl;
  cout << "SO3 from quaternion: " << SO3_q << endl;

  //使用对数映射获得它的李代数
  Eigen::Vector3d so3 = SO3_R.log();
  cout << "so3 = " << so3.transpose() << endl;
  //hat 为向量到反对称矩阵
  cout << "so3 hat = " << Sophus::SO3::hat(so3) << endl;
  //vee 为反对称矩阵到向量
  cout << "so3 hav vee = " << Sophus::SO3::vee(Sophus::SO3::hat(so3)).transpose() << endl;

  //增量扰动模型的更新
  Eigen::Vector3d update_so3(1e-4, 0, 0);
  Sophus::SO3 SO3_updated = Sophus::SO3::exp(update_so3)*SO3_R;//左乘更新
  cout << "SO3 update = " << SO3_updated << endl;


  //特殊欧氏群,旋转平移都有
  Eigen::Vector3d t(1,0,0);
  Sophus::SE3 SE3_Rt(R, t);
  Sophus::SE3 SE3_qt(Eigen::Quaterniond(R), t);
  cout << "SE3 from R,t = " << endl << SE3_Rt << endl;
  cout << "SE3 from q,t = " << endl << SE3_qt << endl;

  //李代数se(3)是一个六维向量
  Eigen::Matrix<double, 6, 1> se3 = SE3_Rt.log();
  cout << "se3 = " << se3.transpose() << endl;
  //同样的,有hat和vee算符
  cout << "se3 hat = " << endl << Sophus::SE3::hat(se3) << endl;
  cout << "se3 hat's vee = " << endl << Sophus::SE3::vee(Sophus::SE3::hat(se3)).transpose() << endl;

  //计算更新
  Eigen::Matrix<double, 6, 1> update_se3;
  update_se3.setZero();
  update_se3(0, 0) = 1e-4;
  Sophus::SE3 SE3_updated = Sophus::SE3::exp(update_se3)*SE3_Rt;
  cout << "SE3 update = " << endl << SE3_updated.matrix() << endl;
  cout << "SE3 update(v) = " << endl << SE3_updated << endl;

  return 0;










}
