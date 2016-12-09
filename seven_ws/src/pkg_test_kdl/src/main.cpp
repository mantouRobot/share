#include <kdl/chain.hpp>
#include <ros/ros.h>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>

using namespace KDL;

int main(int argc, char** argv)
{
    KDL::Chain chain;
    chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,1.020))));
//    chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.480))));
//    chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.645))));
//    chain.addSegment(Segment(Joint(Joint::RotZ)));
//    chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.120))));
//    chain.addSegment(Segment(Joint(Joint::RotZ)));

    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);

    unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray jointpositions = JntArray(nj);

    for(unsigned int i = 0; i < nj; i++)
    {
        float myinput;
//        std::cout << "Enter the position of joint %i: " << i;
//        std::cin >> myinput;
        printf("Enter the position of joint %i:", i);
        scanf("%e", &myinput);
        jointpositions(i) = (double)myinput;
    }

    KDL::Frame cartpos;
    bool kinematics_status;
    kinematics_status = fksolver.JntToCart(jointpositions, cartpos);
    if(kinematics_status >= 0)
    {
        std::cout << cartpos <<std::endl;
        printf("%s \n","Success, thanks KDL.");

    }
    else
    {
        printf("%s \n", "Error:could not calculate forward kinematics.");
    }
}
