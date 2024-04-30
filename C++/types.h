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

#ifndef MPCC_TYPES_H
#define MPCC_TYPES_H

#include "config.h"
namespace mpcc{
/// @brief State of manipulator system
/// @param q1  (double) joint angle 
/// @param q2  (double) joint angle 
/// @param q3  (double) joint angle 
/// @param q4  (double) joint angle 
/// @param q5  (double) joint angle 
/// @param q6  (double) joint angle 
/// @param q7  (double) joint angle 
/// @param s   (double) path parameter, arc length 
/// @param vs  (double) velocity of path parameter 
struct State{ 
    double q1; // joint angle
    double q2; // joint angle
    double q3; // joint angle 
    double q4; // joint angle
    double q5; // joint angle
    double q6; // joint angle
    double q7; // joint angle
    double s;  // path parameter (arc length)
    double vs; // velocity of path parameter

    void setZero()
    {
        q1 = 0.0;
        q2 = 0.0;
        q3 = 0.0;
        q4 = 0.0;
        q5 = 0.0;
        q6 = 0.0;
        q7 = 0.0;
        s = 0.0;
        vs = 0.0;
    }
    /// @brief mapping s to [0, track_length]
    void unwrap(double track_length)
    {
        if (s > track_length)
            s -= track_length;
        if (s < 0)
            s += track_length;
    }
    // /// @brief set minimum x velocity
    // void vxNonZero(double vx_zero)
    // {
    //     if(vx < vx_zero){
    //         vx = vx_zero;
    //         vy = 0.0;
    //         r = 0.0;
    //         delta = 0.0;
    //     }
    // }
};

/// @brief Control input of manipulator system
/// @param dq1 (double) velocity of joint angle
/// @param dq2 (double) velocity of joint angle
/// @param dq3 (double) velocity of joint angle
/// @param dq4 (double) velocity of joint angle
/// @param dq5 (double) velocity of joint angle
/// @param dq6 (double) velocity of joint angle
/// @param dq7 (double) velocity of joint angle
/// @param dVs (double) change of velocity of path parameter 
struct Input{
    double dq1;
    double dq2;
    double dq3;
    double dq4;
    double dq5;
    double dq6;
    double dq7;
    double dVs;

    void setZero()
    {
        dq1 = 0.0;
        dq2 = 0.0;
        dq3 = 0.0;
        dq4 = 0.0;
        dq5 = 0.0;
        dq6 = 0.0;
        dq7 = 0.0;
        dVs = 0.0;
    }
    /// @brief set minimum joint velocity
    void dqNonZero()
    {
        if(fabs(dq1) < 1e-9) dq1 = 1e-3;
        if(fabs(dq2) < 1e-9) dq2 = 1e-3;
        if(fabs(dq3) < 1e-9) dq3 = 1e-3;
        if(fabs(dq4) < 1e-9) dq4 = 1e-3;
        if(fabs(dq5) < 1e-9) dq5 = 1e-3;
        if(fabs(dq6) < 1e-9) dq6 = 1e-3;
        if(fabs(dq7) < 1e-9) dq7 = 1e-3;
    }
};

/// @brief  Slack variables wrt constraints
/// @param selcol (double) self collision constraint
/// @param sing (double) singularity constraint
struct Slack{
    double selcol;
    double sing;

    void setZero()
    {
        selcol = 0.0;
        sing = 0.0;
    }
};

/// @brief path of JSON files
/// @param param_path         (std::string) path of model parameter 
/// @param cost_path          (std::string) path of cost parameter
/// @param bounds_path        (std::string) path of bound parameter 
/// @param track_path         (std::string) path of track 
/// @param normalization_path (std::string) path of normalization parameter 
struct PathToJson{
    const std::string param_path;
    const std::string cost_path;
    const std::string bounds_path;
    const std::string track_path;
    const std::string normalization_path;
};

typedef Eigen::Matrix<double,NX,1> StateVector;
typedef Eigen::Matrix<double,PANDA_DOF,1> JointVector;
typedef Eigen::Matrix<double,NU,1> InputVector;
typedef Eigen::Matrix<double,NS,1> SlackVector;

// x_(k+1) = Ax + Bu + g
typedef Eigen::Matrix<double,NX,NX> A_MPC; 
typedef Eigen::Matrix<double,NX,NU> B_MPC;
typedef Eigen::Matrix<double,NX,1> g_MPC;

typedef Eigen::Matrix<double,NX,NX> Q_MPC;
typedef Eigen::Matrix<double,NU,NU> R_MPC;
typedef Eigen::Matrix<double,NX,NU> S_MPC;

typedef Eigen::Matrix<double,NX,1> q_MPC;
typedef Eigen::Matrix<double,NU,1> r_MPC;

typedef Eigen::Matrix<double,NPC,NX> C_MPC;
typedef Eigen::Matrix<double,1,NX> C_i_MPC;
typedef Eigen::Matrix<double,NPC,NU> D_MPC;
typedef Eigen::Matrix<double,1,NU> D_i_MPC;
typedef Eigen::Matrix<double,NPC,1> d_MPC;

typedef Eigen::Matrix<double,NS,NS> Z_MPC;
typedef Eigen::Matrix<double,NS,1> z_MPC;

typedef Eigen::Matrix<double,NX,NX> TX_MPC;
typedef Eigen::Matrix<double,NU,NU> TU_MPC;
typedef Eigen::Matrix<double,NS,NS> TS_MPC;

typedef Eigen::Matrix<double,NX,1> Bounds_x;
typedef Eigen::Matrix<double,NU,1> Bounds_u;
typedef Eigen::Matrix<double,NS,1> Bounds_s;

StateVector stateToVector(const State &x);
JointVector stateToJointVector(const State &x);
InputVector inputToVector(const Input &u);
JointVector inputToJointVector(const Input &u);
SlackVector slackToVector(const Slack &s);

State vectorToState(const StateVector &xk);
Input vectorToInput(const InputVector &uk);
Slack vectorToSlack(const SlackVector &sk);

State arrayToState(double *xk);
Input arrayToInput(double *uk);
Slack arrayToSlack(double *sk);
}
#endif //MPCC_TYPES_H
