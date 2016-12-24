#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sophus/so3.h>
#include <sophus/se3.h>

int main()
{
  //about z axis 90 degree
  Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,0,1)).toRotationMatrix();

  Sophus::SO3 SO3_R(R);
  Sophus::SO3 SO3_v(0,0,M_PI/2);
  Sophus::SO3 SO3_q(Eigen::Quaterniond(R));













}
