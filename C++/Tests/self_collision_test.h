#ifndef MPCC_SELF_COLLISION_TEST_H
#define MPCC_SELF_COLLISION_TEST_H

#include "Constraints/SelfCollision/SelfCollisionModel.h"
#include "types.h"
#include "gtest/gtest.h"

TEST(TestSelfCollision, TestCalculateMLPOutput)
{
    std::shared_ptr<mpcc::SelCollNNmodel> selcol;
    selcol = std::make_shared<mpcc::SelCollNNmodel>();
    mpcc::JointVector q0;
    q0 <<  0, 0, 0, -M_PI/2, 0,  M_PI/2,  M_PI/4;
    bool result;
    try
    {
        Eigen::Vector2d n_hidden;
        n_hidden << 128, 128;
        selcol->setNeuralNetwork(PANDA_DOF, 1, n_hidden, true);
        auto pred = selcol->calculateMlpOutput(q0, false);
        Eigen::VectorXd y = pred.first;
        Eigen::MatrixXd y_jac = pred.second;
        // std:cout<<"y_jac size: "<< y_jac.rows()<< " X "<< y_jac.cols()<<std::endl;0
        

        // std::cout << "y: \n" << std::fixed << std::setprecision(3) <<
        //  y.transpose() << std::endl;
        //  std::cout << "y_jac: \n" << std::fixed << std::setprecision(3) <<
        //  y_jac << std::endl;

        // model output
        // y = 21.50005
        // jac = -0.0242, -0.1494,  0.0898,  7.2674,  0.1791,  0.3918,  0.0249
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