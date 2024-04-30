#ifndef MPCC_ROBOT_MODEL_TEST_H
#define MPCC_ROBOT_MODEL_TEST_H

#include "Model/robot_model.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include "gtest/gtest.h"

typedef std::chrono::high_resolution_clock hd_clock;
typedef std::chrono::duration<double, std::ratio<1> > second;
std::chrono::time_point<hd_clock> beg_;

TEST(TestRobotModel, TestGetEEPosition)
{
    std::shared_ptr<mpcc::RobotModel> robot;
    robot = std::make_shared<mpcc::RobotModel>();
    mpcc::JointVector q0;
    q0 <<  -0.002, -0.001,  0.002, -1.574,  0.006,  1.584,  0.789;
    Vector3d x0 = Vector3d::Zero();
    bool result;
    try
    {
        x0 = robot->getEEPosition(q0);
        // std::cout << "EE position: " << std::fixed << std::setprecision(3) <<
        //  x0.transpose() << std::endl;
        // real robot: 
        // 0.557 0.001 0.522
        result = true;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        result = false;
    }

    EXPECT_TRUE(result);
}

TEST(TestRobotModel, TestGetEEOrientation)
{
    std::shared_ptr<mpcc::RobotModel> robot;
    robot = std::make_shared<mpcc::RobotModel>();
    mpcc::JointVector q0;
    q0.setZero();
    q0 <<  -0.002, -0.001,  0.002, -1.574,  0.006,  1.584,  0.789;
    Matrix3d r0 = Matrix3d::Zero();
    bool result;
    try
    {
        r0 = robot->getEEOrientation(q0, PANDA_NUM_LINKS);
        // std::cout << "EE ori: \n" << std::fixed << std::setprecision(3) <<
        //  r0 << std::endl;
        result = true;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        result = false;
    }

    EXPECT_TRUE(result);
}

TEST(TestRobotModel, TestGetJacobianv)
{
    std::shared_ptr<mpcc::RobotModel> robot;
    robot = std::make_shared<mpcc::RobotModel>();
    mpcc::JointVector q0;
    q0 <<  -0.002, -0.001,  0.002, -1.574,  0.006,  1.584,  0.789;
    Matrix<double, 3, PANDA_DOF> J0;
    J0.setZero();
    bool result;
    try
    {
        J0 = robot->getJacobianv(q0);
        // std::cout << "EE jacobian : \n" << std::fixed << std::setprecision(3) 
        // << J0 << std::endl;
        // real robot:
        // 0.001 0.189 -0.001 0.128 0.000 0.209 0.000
        // 0.557 -0.000 0.557 -0.000 0.209 -0.001 -0.000
        // 0.000 -0.557 -0.000 0.474 0.001 0.090 -0.000
        result = true;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        result = false;
    }
    EXPECT_TRUE(result);
}

TEST(TestRobotModel, TestManipulability)
{
    std::shared_ptr<mpcc::RobotModel> robot;
    robot = std::make_shared<mpcc::RobotModel>();
    mpcc::JointVector q0;
    q0 <<  0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4;
    double mani = 0;
    VectorXd d_mani = VectorXd::Zero(PANDA_DOF);
    bool result;
    try
    {
        beg_ = hd_clock::now();
        mani = robot->getManipulability(q0);
        d_mani = robot->getDManipulability(q0);
        auto duration = std::chrono::duration_cast<second>(hd_clock::now() - beg_).count();
        // std::cout << "Manipulability: \n" << mani << std::endl;
        // std::cout << "Gradient of Manipulability: \n" << d_mani.transpose() << std::endl;
        std::cout << "Time to get manipulability index and its gradient: " << duration*1e+3 << " [ms]!"<<std::endl;
        result = true;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        result = false;
    }
    EXPECT_TRUE(result);
}

#endif