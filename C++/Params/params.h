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

#ifndef MPCC_PARAMS_H
#define MPCC_PARAMS_H

// #include <iostream>
// #include <fstream>
#include <vector>
#include <nlohmann/json.hpp>
#include "config.h"
#include "types.h"

namespace mpcc{
//used namespace
using json = nlohmann::json;

// // dynamic model parameter class 
class Param{
public:
//     double Cm1;
//     double Cm2;
//     double Cr0;
//     double Cr2;

//     double Br;
//     double Cr;
//     double Dr;

//     double Bf;
//     double Cf;
//     double Df;

//     double m;
//     double Iz;
//     double lf;
//     double lr;

//     double car_l;
//     double car_w;

//     double g;

//     double r_in; // length from track center to inner boundary 
//     double r_out; // length from track center to outter boundary

    double max_dist_proj;

//     double e_long; // tire specific ellipse parameters (tire contraints)
//     double e_eps;

//     double max_alpha; // maximum slip angle for front tire

    double desired_ee_velocity; // desired end-effector velocity
    double s_trust_region;

//     double vx_zero;

    Param();
    Param(std::string file);

};

class CostParam{
public:
    // Contouring cost
    double q_c;        // weight for contouring error
    double q_c_N_mult; // weight multiplication for terminal 
    double q_l;        // weight for lag error
    double q_vs;       // weight for velocity of path parameter

    // Heading cost
    double q_ori; // weight for heading cost

    // Input cost
    double r_dq;     // weight for joint velocity
    double r_dVs;    // weight for accelerate of path parameter
    double r_ee;     // weight for EE velocity error

    // Soft constraint cost
    double sc_quad_selcol;
    double sc_quad_sing;

    double sc_lin_selcol;
    double sc_lin_sing;

    CostParam();
    CostParam(std::string file);

};

class BoundsParam{
public:
    /// @brief  Lower bound of state
    /// @param q1_l (double) lower bound of q1
    /// @param q2_l (double) lower bound of q2
    /// @param q3_l (double) lower bound of q3
    /// @param q4_l (double) lower bound of q4
    /// @param q5_l (double) lower bound of q5
    /// @param q6_l (double) lower bound of q6
    /// @param q7_l (double) lower bound of q7
    /// @param s_l  (double) lower bound of s
    /// @param vs_l (double) lower bound of vs
    struct LowerStateBounds{
        double q1_l;
        double q2_l;
        double q3_l;
        double q4_l;
        double q5_l;
        double q6_l;
        double q7_l;
        double s_l;
        double vs_l;
    };

    /// @brief  Upper bound of state
    /// @param q1_u (double) upper bound of q1
    /// @param q2_u (double) upper bound of q2
    /// @param q3_u (double) upper bound of q3
    /// @param q4_u (double) upper bound of q4
    /// @param q5_u (double) upper bound of q5
    /// @param q6_u (double) upper bound of q6
    /// @param q7_u (double) upper bound of q7
    /// @param s_u  (double) upper bound of s
    /// @param vs_u (double) upper bound of vs
    struct UpperStateBounds{
        double q1_u;
        double q2_u;
        double q3_u;
        double q4_u;
        double q5_u;
        double q6_u;
        double q7_u;
        double s_u;
        double vs_u;
    };

    /// @brief  Lower bound of control input
    /// @param dq1_l (double) lower bound of dq1
    /// @param dq2_l (double) lower bound of dq2
    /// @param dq3_l (double) lower bound of dq3
    /// @param dq4_l (double) lower bound of dq4
    /// @param dq5_l (double) lower bound of dq5
    /// @param dq6_l (double) lower bound of dq6
    /// @param dq7_l (double) lower bound of dq7
    /// @param dVs_l (double) lower bound of dVs
    struct LowerInputBounds{
        double dq1_l;
        double dq2_l;
        double dq3_l;
        double dq4_l;
        double dq5_l;
        double dq6_l;
        double dq7_l;
        double dVs_l;
    };

    /// @brief  Upper bound of control input
    /// @param dq1_u (double) upper bound of dq1
    /// @param dq2_u (double) upper bound of dq2
    /// @param dq3_u (double) upper bound of dq3
    /// @param dq4_u (double) upper bound of dq4
    /// @param dq5_u (double) upper bound of dq5
    /// @param dq6_u (double) upper bound of dq6
    /// @param dq7_u (double) upper bound of dq7
    /// @param dVs_u (double) upper bound of dVs
    struct UpperInputBounds{
        double dq1_u;
        double dq2_u;
        double dq3_u;
        double dq4_u;
        double dq5_u;
        double dq6_u;
        double dq7_u;
        double dVs_u;
    };

    LowerStateBounds lower_state_bounds;
    UpperStateBounds upper_state_bounds;

    LowerInputBounds lower_input_bounds;
    UpperInputBounds upper_input_bounds;

    BoundsParam();
    BoundsParam(std::string file);

};

class NormalizationParam{
public:
    TX_MPC T_x;
    TX_MPC T_x_inv;

    TU_MPC T_u;
    TU_MPC T_u_inv;

    TS_MPC T_s;
    TS_MPC T_s_inv;

    NormalizationParam();
    NormalizationParam(std::string file);
};
}
#endif //MPCC_PARAMS_H
