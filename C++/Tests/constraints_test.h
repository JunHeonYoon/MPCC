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

#ifndef MPCC_CONSTRATINS_TEST_H
#define MPCC_CONSTRATINS_TEST_H


#include "Spline/arc_length_spline.h"
#include "Constraints/constraints.h"
#include "Constraints/SelfCollision/SelfCollisionModel.h"
#include "Model/robot_model.h"
#include "gtest/gtest.h"
#include <nlohmann/json.hpp>
using json = nlohmann::json;

void genRoundTrack(mpcc::ArcLengthSpline &track)
{
    int NT = 100;    //number of track points
    double TrackRadius = 0.2; // track radius

    Eigen::VectorXd X;
    Eigen::VectorXd Y;
    Eigen::VectorXd Z;
    Eigen::VectorXd phiR;

    X.setZero(NT);
    Y.setZero(NT);
    Z.setZero(NT);
//    // randomly distribute training points around circle
//    // generate random agles between [0,2pi]
//    phiR.setRandom(NT);
//    phiR = (phiR/2.0+0.5*Eigen::VectorXd::Ones(NT))*2*M_PI;
//    // sort training points
//    std::sort(phiR.data(), phiR.data()+phiR.size());
//    // fix start and end point
//    phiR(0) = 0;
//    phiR(NT-1) = 2*M_PI;
    // generate equally spaced points around circle
    phiR.setLinSpaced(NT,0,2*M_PI);
    // compute circle points
    for(int i=0;i<NT;i++){
        Z(i) = 0;
        Y(i) = TrackRadius*std::cos(phiR(i));
        Z(i) = TrackRadius*std::sin(phiR(i));
    }

    // give points to arc length based 3-D curve fitting
    track.gen3DSpline(X,Y,Z);



//    std::vector<double> Xv(NT),Yv(NT),Xsv(NT*10),Ysv(NT*10);
//    for(int i=0;i<NT;i++){
//        Xv[i] = X(i);
//        Yv[i] = Y(i);
//    }
//    Eigen::VectorXd s;
//    s.setLinSpaced(NT*10,0,2*M_PI);
//    for(int i=0;i<10*NT;i++){
//        Xsv[i] = track.splineX.ppval(s(i));
//        Ysv[i] = track.splineY.ppval(s(i));
//    }
//
//    plt::plot(Xv,Yv,"--g");
//    plt::draw();
//    plt::plot(Xsv,Ysv,"--r");
//    plt::draw();
//    plt::show();

}

double getRBF(double delta, double h)
{
    // Grandia, Ruben, et al. 
    // "Feedback mpc for torque-controlled legged robots." 
    // 2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2019.
    double result;
    if (h >= delta) result = -log(h);
    else            result = ( pow( (h-2*delta) / delta ,2) - 1 ) / 2 - log(delta);
    return result;
}

TEST(TestConstraints, TestSelfCollision)
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

    std::shared_ptr<mpcc::SelCollNNmodel> selcol;
    selcol = std::make_shared<mpcc::SelCollNNmodel>();
    Eigen::Vector2d n_hidden;
    n_hidden << 128, 128;
    selcol->setNeuralNetwork(PANDA_DOF, 1, n_hidden, true);

    mpcc::Constraints constraints = mpcc::Constraints(0.02,json_paths, robot, selcol);
    mpcc::ArcLengthSpline track = mpcc::ArcLengthSpline(json_paths,robot);
    mpcc::Model model = mpcc::Model(0.02, json_paths);
    mpcc::Param param = mpcc::Param(json_paths.param_path);
    // track.setParam(param);
    
    genRoundTrack(track);

    mpcc::StateVector xk0_vec, xk1_vec;
    xk0_vec << -0.59924886,  0.33032777, -1.79500182, -1.63463425, -0.16669177,  2.592901, -1.4701306, 0.1, 0.3;      
    xk1_vec  << -2.58433695, -1.36542807,  0.9670449,  -2.6810509,  -2.76339922,  3.30783224, -1.67854509, 0.01, 0.5; 
    mpcc::InputVector uk0_vec, uk1_vec;
    uk0_vec << 0.01,0.04,0.4,0.5,0.8,0.1,-0.4,0.1;
    uk1_vec << -0.4,0.5,0.01,0.07,-0.34,0.14,0.1,0.1;

    mpcc::State xk0 = mpcc::vectorToState(xk0_vec);
    mpcc::Input uk0 = mpcc::vectorToInput(uk0_vec);
    mpcc::State xk1 = mpcc::vectorToState(xk1_vec);
    mpcc::Input uk1 = mpcc::vectorToInput(uk1_vec);

    bool result = false;

    // condition 1
    // real inequality constraint
    auto pred0 = selcol->calculateMlpOutput(xk0_vec.segment(0,PANDA_DOF),false);
    double r_l0 = -100000; // -inf
    double r_x0 = -(pred0.second*uk0_vec.segment(0,PANDA_DOF))(0);
    double r_u0 = getRBF(2, pred0.first(0)-5);

    // Linearization inequality constriant
    mpcc::ConstrainsMatrix constraints_0 = constraints.getConstraints(track,xk0,uk0);
    double l_l0 = constraints_0.dl(0);
    double l_x0 = (constraints_0.C.row(0)*xk0_vec + constraints_0.D.row(0)*uk0_vec)(0);
    double l_u0 = constraints_0.du(0);

    if (r_l0 <= r_x0 && r_x0 <= r_u0) // if real ineq is true
    {
        if(l_l0 <= l_x0 && l_x0 <= l_u0) // also Lin ineq is true
        {
            result = true;
        }
    }
    else // if real ineq is false
    {
        if(!(l_l0 <= l_x0 && l_x0 <= l_u0)) // also Lin ineq is false
        {
            result = true;
        }
    }

    // condition 2
    // real inequality constraint
    auto pred1 = selcol->calculateMlpOutput(xk1_vec.segment(0,PANDA_DOF),false);
    double r_l1 = -100000; // -inf
    double r_x1 = -(pred1.second*uk1_vec.segment(0,PANDA_DOF))(0);
    double r_u1 = getRBF(2, pred1.first(0)-5);

    // Linearization inequality constriant
    mpcc::ConstrainsMatrix constraints_1 = constraints.getConstraints(track,xk1,uk1);
    double l_l1 = constraints_1.dl(0);
    double l_x1 = (constraints_1.C.row(0)*xk1_vec + constraints_1.D.row(0)*uk1_vec)(0);
    double l_u1 = constraints_1.du(0);

    if (r_l1 <= r_x1 && r_x1 <= r_u1) // if real ineq is true
    {
        if(l_l1 <= l_x1 && l_x1 <= l_u1) // also Lin ineq is true
        {
            result = true;
        }
    }
    else // if real ineq is false
    {
        if(!(l_l1 <= l_x1 && l_x1 <= l_u1)) // also Lin ineq is false
        {
            result = true;
        }
    }


    EXPECT_TRUE(result);
}

TEST(TestConstraints, TestSingularity)
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

    std::shared_ptr<mpcc::SelCollNNmodel> selcol;
    selcol = std::make_shared<mpcc::SelCollNNmodel>();
    Eigen::Vector2d n_hidden;
    n_hidden << 128, 128;
    selcol->setNeuralNetwork(PANDA_DOF, 1, n_hidden, true);

    mpcc::Constraints constraints = mpcc::Constraints(0.02,json_paths, robot, selcol);
    mpcc::ArcLengthSpline track = mpcc::ArcLengthSpline(json_paths,robot);
    mpcc::Model model = mpcc::Model(0.02, json_paths);
    mpcc::Param param = mpcc::Param(json_paths.param_path);
    // track.setParam(param);
    
    genRoundTrack(track);

    mpcc::StateVector xk0_vec, xk1_vec;
    xk0_vec << -0.59924886,  0.33032777, -1.79500182, -1.63463425, -0.16669177,  2.592901, -1.4701306, 0.1, 0.3;      
    xk1_vec  << -2.58433695, -1.36542807,  0.9670449,  -2.6810509,  -2.76339922,  3.30783224, -1.67854509, 0.01, 0.5; 
    mpcc::InputVector uk0_vec, uk1_vec;
    uk0_vec << 0.01,0.04,0.4,0.5,0.8,0.1,-0.4,0.1;
    uk1_vec << -0.4,0.5,0.01,0.07,-0.34,0.14,0.1,0.1;

    mpcc::State xk0 = mpcc::vectorToState(xk0_vec);
    mpcc::Input uk0 = mpcc::vectorToInput(uk0_vec);
    mpcc::State xk1 = mpcc::vectorToState(xk1_vec);
    mpcc::Input uk1 = mpcc::vectorToInput(uk1_vec);

    bool result = false;

    // condition 1
    // real inequality constraint
    double r_l0 = -100000; // -inf
    double r_x0 = -((robot->getDManipulability(mpcc::stateToJointVector(xk0))).transpose() * mpcc::inputToJointVector(uk0))(0);
    double r_u0 = getRBF(0.05, robot->getManipulability(mpcc::stateToJointVector(xk0)) - 0.03);

    // Linearization inequality constriant
    mpcc::ConstrainsMatrix constraints_0 = constraints.getConstraints(track,xk0,uk0);
    double l_l0 = constraints_0.dl(1);
    double l_x0 = (constraints_0.C.row(1)*xk0_vec + constraints_0.D.row(1)*uk0_vec)(0);
    double l_u0 = constraints_0.du(1);

    if (r_l0 <= r_x0 && r_x0 <= r_u0) // if real ineq is true
    {
        if(l_l0 <= l_x0 && l_x0 <= l_u0) // also Lin ineq is true
        {
            result = true;
        }
    }
    else // if real ineq is false
    {
        if(!(l_l0 <= l_x0 && l_x0 <= l_u0)) // also Lin ineq is false
        {
            result = true;
        }
    }

    // condition 2
    // real inequality constraint
    double r_l1 = -100000; // -inf
    double r_x1 = -((robot->getDManipulability(mpcc::stateToJointVector(xk1))).transpose() * mpcc::inputToJointVector(uk1))(0);
    double r_u1 = getRBF(0.05, robot->getManipulability(mpcc::stateToJointVector(xk1)) - 0.03);

    // Linearization inequality constriant
    mpcc::ConstrainsMatrix constraints_1 = constraints.getConstraints(track,xk1,uk1);
    double l_l1 = constraints_1.dl(1);
    double l_x1 = (constraints_1.C.row(1)*xk1_vec + constraints_0.D.row(1)*uk1_vec)(0);
    double l_u1 = constraints_1.du(1);


    if (r_l1 <= r_x1 && r_x1 <= r_u1) // if real ineq is true
    {
        if(l_l1 <= l_x1 && l_x1 <= l_u1) // also Lin ineq is true
        {
            result = true;
        }
    }
    else // if real ineq is false
    {
        if(!(l_l1 <= l_x1 && l_x1 <= l_u1)) // also Lin ineq is false
        {
            result = true;
        }
    }


    EXPECT_TRUE(result);
}


#endif //MPCC_CONSTRATINS_TEST_H