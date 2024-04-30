// Copyright 2019 Alexander Liniger

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
#ifndef MPCC_COST_TEST_H
#define MPCC_COST_TEST_H

#include "Cost/cost.h"
#include "constraints_test.h"
#include "gtest/gtest.h"
#include <nlohmann/json.hpp>
using json = nlohmann::json;

TEST(TestCost, TestSPD)
{
    std::ifstream iConfig("Params/config.json");
    json jsonConfig;
    iConfig >> jsonConfig;

    mpcc::PathToJson json_paths {jsonConfig["model_path"],
                                 jsonConfig["cost_path"],
                                 jsonConfig["bounds_path"],
                                 jsonConfig["track_path"],
                                 jsonConfig["normalization_path"]};
    
    std::shared_ptr<mpcc::RobotModel> robot;
    robot = std::make_shared<mpcc::RobotModel>();

    mpcc::Cost cost = mpcc::Cost(json_paths, robot);
    mpcc::ArcLengthSpline track = mpcc::ArcLengthSpline(json_paths, robot);
    genRoundTrack(track);


    mpcc::StateVector xk_vec;
    xk_vec << 0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4, 0,   0;   // default state

    mpcc::InputVector uk_vec;
    uk_vec << 0.1, 0.2, 0.3,-0.3, -0.5, 0.1, 0.2, 0.05;

    // calculate cost matrix
    mpcc::CostMatrix cost_mat = cost.getCost(track,mpcc::vectorToState(xk_vec),mpcc::vectorToInput(uk_vec),1);

    bool is_symQ = (cost_mat.Q.transpose() - cost_mat.Q).norm() < 1e-5;
    bool is_symR = (cost_mat.R.transpose() - cost_mat.R).norm() < 1e-5;
    bool is_symZ = (cost_mat.Z.transpose() - cost_mat.Z).norm() < 1e-5;
    std::cout<< "Is symmetric(Q, R, Z): " << is_symQ << is_symR << is_symZ << std::endl;

    Eigen::EigenSolver<Eigen::MatrixXd> eigensolverQ, eigensolverR, eigensolverZ;
    eigensolverQ.compute(cost_mat.Q); eigensolverR.compute(cost_mat.R); eigensolverZ.compute(cost_mat.Z);

    Eigen::VectorXd eigen_valuesQ = eigensolverQ.eigenvalues().real();
    Eigen::VectorXd eigen_valuesR = eigensolverR.eigenvalues().real();
    Eigen::VectorXd eigen_valuesZ = eigensolverZ.eigenvalues().real();

    bool eigen_positiveQ = eigen_valuesQ.minCoeff() > 0;
    bool eigen_positiveR = eigen_valuesR.minCoeff() > 0;
    bool eigen_positiveZ = eigen_valuesZ.minCoeff() > 0;

    bool is_spdQ = is_symQ && eigen_positiveQ;
    bool is_spdR = is_symR && eigen_positiveR;
    bool is_spdZ = is_symZ && eigen_positiveZ;

    std::cout<< "Is semi positive definite(Q, R, Z): " << is_spdQ << is_spdR << is_spdZ << std::endl;
    if(!eigen_positiveQ) std::cout << "min eigenvalue Q: " << eigen_valuesQ.minCoeff() << std::endl;
    if(!eigen_positiveR) std::cout << "min eigenvalue R: " << eigen_valuesR.minCoeff() << std::endl;
    if(!eigen_positiveZ) std::cout << "min eigenvalue Z: " << eigen_valuesZ.minCoeff() << std::endl;

    
    EXPECT_TRUE(is_symQ && is_symR && is_symZ && is_spdQ && is_spdR && is_spdZ);
}

TEST(TestCost, TestLinearization)
{
    std::ifstream iConfig("Params/config.json");
    json jsonConfig;
    iConfig >> jsonConfig;

    mpcc::PathToJson json_paths {jsonConfig["model_path"],
                                 jsonConfig["cost_path"],
                                 jsonConfig["bounds_path"],
                                 jsonConfig["track_path"],
                                 jsonConfig["normalization_path"]};
    
    std::shared_ptr<mpcc::RobotModel> robot;
    robot = std::make_shared<mpcc::RobotModel>();

    mpcc::Cost cost = mpcc::Cost(json_paths, robot);
    mpcc::CostParam cost_param = mpcc::CostParam(json_paths.cost_path);
    mpcc::Param param = mpcc::Param(json_paths.param_path);
    mpcc::ArcLengthSpline track = mpcc::ArcLengthSpline(json_paths, robot);
    genRoundTrack(track);


    mpcc::StateVector xk_vec, d_xk_vec, xk1_vec;
    xk_vec << 0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4, 0,   0;   // default state
    d_xk_vec << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;


    mpcc::InputVector uk_vec, d_uk_vec, uk1_vec;
    uk_vec << 0, 0, 0, 0, 0, 0, 0, 0.05;
    d_uk_vec << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;


    // calculate q_dot which EE velocity is desired_ee_vel at xk3_vec state
    double desired_ee_vel = mpcc::Param(json_paths.param_path).desired_ee_velocity;
    xk_vec(8) = desired_ee_vel;
    Eigen::Vector3d tangent_vec = track.getDerivative(xk_vec(7));
    Eigen::Matrix<double,3,PANDA_DOF> jv = robot->getJacobianv(xk_vec.segment(0,PANDA_DOF));
    Eigen::Matrix<double,PANDA_DOF,3> j_pinv = jv.transpose() * (jv*jv.transpose()).inverse();
    uk_vec.segment(0,PANDA_DOF) = j_pinv * (desired_ee_vel*tangent_vec);

    // caluculate real cost at xk, uk
    // contouring cost
    const mpcc::JointVector q = xk_vec.segment(0,PANDA_DOF);
    const Eigen::Vector3d pos = robot->getEEPosition(q);
    const double s = xk_vec(PANDA_DOF);
    const Eigen::Vector3d pos_ref = track.getPostion(s);
    const Eigen::Vector3d dpos_ref = track.getDerivative(s);

    const Eigen::Vector3d total_error = pos - pos_ref;
    Eigen::Vector3d lag_error = (dpos_ref.dot(total_error)) * dpos_ref;
    Eigen::Vector3d contouring_error = total_error - lag_error;

    double cost_contouring = cost_param.q_c*contouring_error.squaredNorm() + cost_param.q_l*lag_error.squaredNorm() - cost_param.q_vs*xk_vec(PANDA_DOF+1);

    // control input cost
    Matrix<double, 3, PANDA_DOF> Jv = robot->getJacobianv(q);
    const mpcc::JointVector dq = uk_vec.segment(0,PANDA_DOF);
    double ee_vel = (Jv * dq).norm();
    ee_vel = std::copysign(std::max(fabs(ee_vel), 1E-8), ee_vel);
    double ee_vel_error = ee_vel - param.desired_ee_velocity;

    double cost_input = cost_param.r_dq*dq.norm() + cost_param.r_dVs*uk_vec(PANDA_DOF) + cost_param.r_ee*pow(ee_vel_error,2);

    double cost0 = cost_contouring + cost_input;

    // caluculate real cost at xk+d_xk, uk+duk
    xk1_vec = xk_vec + d_xk_vec;
    uk1_vec = uk_vec + d_uk_vec;

    const mpcc::JointVector q1 = xk1_vec.segment(0,PANDA_DOF);
    const Eigen::Vector3d pos1 = robot->getEEPosition(q1);
    const double s1 = xk1_vec(PANDA_DOF);
    const Eigen::Vector3d pos1_ref = track.getPostion(s1);
    const Eigen::Vector3d dpos1_ref = track.getDerivative(s1);

    const Eigen::Vector3d total_error1 = pos1 - pos1_ref;
    Eigen::Vector3d lag_error1 = (dpos1_ref.dot(total_error1)) * dpos1_ref;
    Eigen::Vector3d contouring_error1 = total_error1 - lag_error1;

    double cost1_contouring = cost_param.q_c*contouring_error1.squaredNorm() + cost_param.q_l*lag_error1.squaredNorm() - cost_param.q_vs*xk1_vec(PANDA_DOF+1);

    // control input cost
    Matrix<double, 3, PANDA_DOF> Jv1 = robot->getJacobianv(q1);
    const mpcc::JointVector dq1 = uk1_vec.segment(0,PANDA_DOF);
    double ee_vel1 = (Jv1 * dq1).norm();
    ee_vel1 = std::copysign(std::max(fabs(ee_vel1), 1E-8), ee_vel1);
    double ee_vel_error1 = ee_vel1 - param.desired_ee_velocity;

    double cost1_input = cost_param.r_dq*dq1.norm() + cost_param.r_dVs*uk1_vec(PANDA_DOF) + cost_param.r_ee*pow(ee_vel_error1,2);

    double cost1 = cost1_contouring + cost1_input;

    // caluculate linearized cost at xk+d_xk, uk+duk
    // calculate cost matrix at xk, uk
    mpcc::CostMatrix cost_mat = cost.getCost(track,mpcc::vectorToState(xk_vec),mpcc::vectorToInput(uk_vec),1);

    // calculate linearized cost at xk, uk
    double cost_lin = (0.5*xk_vec.transpose()*cost_mat.Q*xk_vec + 0.5*uk_vec.transpose()*cost_mat.R*uk_vec +  cost_mat.q.transpose()*xk_vec +  cost_mat.r.transpose()*uk_vec).value();
    
    double cost1_lin = cost0 - cost_lin + 
                       (0.5*xk1_vec.transpose()*cost_mat.Q*xk1_vec + 0.5*uk1_vec.transpose()*cost_mat.R*uk1_vec +  cost_mat.q.transpose()*xk1_vec +  cost_mat.r.transpose()*uk1_vec).value();

    
    std::cout << "real cost on X0     : " << cost0 << std::endl;
    std::cout << "real cost on X0 + dX: " << cost1 << std::endl;
    std::cout << "lin  cost on X0 + dX: " << cost1_lin << std::endl;

    EXPECT_TRUE(fabs((cost1 - cost1_lin) / cost1) <= 1e-2);
}

TEST(TestCost, TestCost)
{

    std::ifstream iConfig("Params/config.json");
    json jsonConfig;
    iConfig >> jsonConfig;

    mpcc::PathToJson json_paths {jsonConfig["model_path"],
                                 jsonConfig["cost_path"],
                                 jsonConfig["bounds_path"],
                                 jsonConfig["track_path"],
                                 jsonConfig["normalization_path"]};
    
    std::shared_ptr<mpcc::RobotModel> robot;
    robot = std::make_shared<mpcc::RobotModel>();

    mpcc::Cost cost = mpcc::Cost(json_paths, robot);
    //cost.setCosts(cost_param);
    mpcc::ArcLengthSpline track = mpcc::ArcLengthSpline(json_paths, robot);
    genRoundTrack(track);


    mpcc::StateVector xk0_vec, xk1_vec, xk2_vec, xk3_vec;
    xk0_vec << 0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4, 0,   0;   // default state
    xk1_vec << 0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4, 0.5, 0;   // (higher cost) reference point far away from EE
    xk2_vec << 0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4, 0,   0.5; // (lower cost)  moving reference point
    xk3_vec << 0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4, 0,   0;   // (lower cost)  EE moving similar to desired velocity

    mpcc::InputVector uk0_vec, uk1_vec, uk2_vec, uk3_vec;
    uk0_vec << 0, 0, 0, 0, 0, 0, 0, 0.05;
    uk1_vec << 0, 0, 0, 0, 0, 0, 0, 0.05;
    uk2_vec << 0, 0, 0, 0, 0, 0, 0, 0.05;
    uk3_vec << 0, 0, 0, 0, 0, 0, 0, 0.05;

    // calculate q_dot which EE velocity is desired_ee_vel at xk3_vec state
    double desired_ee_vel = mpcc::Param(json_paths.param_path).desired_ee_velocity;
    xk3_vec(8) = desired_ee_vel;
    Eigen::Vector3d tangent_vec = track.getDerivative(xk3_vec(7));
    Eigen::Matrix<double,3,PANDA_DOF> jv = robot->getJacobianv(xk3_vec.segment(0,PANDA_DOF));
    Eigen::Matrix<double,PANDA_DOF,3> j_pinv = jv.transpose() * (jv*jv.transpose()).inverse();
    uk3_vec.segment(0,PANDA_DOF) = j_pinv * (desired_ee_vel*tangent_vec);

    // calculate costs
    mpcc::CostMatrix cost_mat0 = cost.getCost(track,mpcc::vectorToState(xk0_vec),mpcc::vectorToInput(uk0_vec),1);
    mpcc::CostMatrix cost_mat1 = cost.getCost(track,mpcc::vectorToState(xk1_vec),mpcc::vectorToInput(uk1_vec),1);
    mpcc::CostMatrix cost_mat2 = cost.getCost(track,mpcc::vectorToState(xk2_vec),mpcc::vectorToInput(uk2_vec),1);
    mpcc::CostMatrix cost_mat3 = cost.getCost(track,mpcc::vectorToState(xk3_vec),mpcc::vectorToInput(uk3_vec),1);
    double cost0 = (0.5*xk0_vec.transpose()*cost_mat0.Q*xk0_vec + 0.5*uk0_vec.transpose()*cost_mat0.R*uk0_vec +  cost_mat0.q.transpose()*xk0_vec +  cost_mat0.r.transpose()*uk0_vec).value();
    double cost1 = (0.5*xk1_vec.transpose()*cost_mat1.Q*xk1_vec + 0.5*uk1_vec.transpose()*cost_mat1.R*uk1_vec +  cost_mat1.q.transpose()*xk1_vec +  cost_mat1.r.transpose()*uk1_vec).value();
    double cost2 = (0.5*xk2_vec.transpose()*cost_mat2.Q*xk2_vec + 0.5*uk2_vec.transpose()*cost_mat2.R*uk2_vec +  cost_mat2.q.transpose()*xk2_vec +  cost_mat2.r.transpose()*uk2_vec).value();
    double cost3 = (0.5*xk3_vec.transpose()*cost_mat3.Q*xk3_vec + 0.5*uk3_vec.transpose()*cost_mat3.R*uk3_vec +  cost_mat3.q.transpose()*xk3_vec +  cost_mat3.r.transpose()*uk3_vec).value();

    std::cout << "cost0: " << cost0 << std::endl;
    std::cout << "cost1: " << cost1 << std::endl;
    std::cout << "cost2: " << cost2 << std::endl;
    std::cout << "cost3: " << cost3 << std::endl;

    bool result;
    if((cost0 <= cost1) && (cost0 >= cost2) && (cost0 >= cost3))
    {
        result = true;
    }
    else
    {
        result = false;
    }

    EXPECT_TRUE(result);
}

#endif //MPCC_COST_TEST_H
