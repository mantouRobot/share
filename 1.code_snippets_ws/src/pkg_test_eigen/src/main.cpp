#include <iostream>
#include <boost/lexical_cast.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

int main(int argc, char** argv)
{
  //题目一
  Eigen::Matrix<double, 8, 8> matrix_big = Eigen::Matrix<double, 8 ,8>::Random();
  //int order = boost::lexical_cast<int>(argv[1]);
  //cout << "You input order = " << order << endl;
  Eigen::Matrix<double, 3, 3>  matrix_block;
  for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++){
      matrix_block(i,j) = matrix_big(i,j);
      if(i == j)
        matrix_big(i,j) = 1;
      else
        matrix_big(i,j) = 0;
    }
  cout << matrix_big << endl;
  cout << matrix_block << endl;

  //题目2
  Eigen::Isometry3d world = Eigen::Isometry3d::Identity();
  cout << "world: " << endl << world.matrix() << endl;

  //萝卜1
  Eigen::Quaterniond q1_raw(0.35, 0.2, 0.3, 0.1);//Eigen quaternion构造的时候实部在前q(w,x,y,z)
  Eigen::Quaterniond q1_normalized(  q1_raw.normalized());//q.normalize()给自己归一化,q.normalized()返回一个归一化
  cout << "q1_raw = " << q1_raw.coeffs().transpose() << endl;//coeffs()输出的是实部在后coeffs->x,y,z,w
  cout << "q1_normalized = " << q1_normalized.coeffs().transpose() << endl;
  Eigen::Vector3d t1(0.3, 0.1, 0.1);
  Eigen::Isometry3d world_to_c1 = Eigen::Isometry3d::Identity();//不能少了Identity
  world_to_c1.rotate(q1_normalized);
  world_to_c1.pretranslate(t1);
  cout << "luobo1: " << endl << world_to_c1.matrix() << endl;

  //萝卜2
  Eigen::Isometry3d world_to_c2 = Eigen::Isometry3d::Identity();
  world_to_c2.rotate(Eigen::Quaterniond(-0.5, 0.4, -0.1, 0.2).normalized());
  world_to_c2.pretranslate(Eigen::Vector3d(-0.1, 0.5, 0.3));
  cout << "luobo2: " << endl << world_to_c2.matrix() << endl;

  Eigen::Isometry3d c2_to_world = Eigen::Isometry3d::Identity();
  c2_to_world.rotate(Eigen::Quaterniond(-0.5, 0.4, -0.1, 0.2).inverse().normalized());
  c2_to_world.pretranslate(-Eigen::Vector3d(-0.1, 0.5, 0.3));
  cout << "luobo2 inverse: " << endl << world_to_c2.matrix() << endl;

  //萝卜1到object
  Eigen::Vector4d c1_to_object(0.5, 0, 0.2, 1);//齐次坐标

  //求萝卜2到object
  //萝卜2到object = 萝卜1到物体*world_to_萝卜1*萝卜2_to_world
  Eigen::Vector4d result = c1_to_object*world_to_c1*(c2_to_world);
}


















