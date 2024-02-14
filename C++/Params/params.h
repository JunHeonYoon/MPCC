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

// dynamic model parameter class 
class Param{
public:
    double Cm1;
    double Cm2;
    double Cr0;
    double Cr2;

    double Br;
    double Cr;
    double Dr;

    double Bf;
    double Cf;
    double Df;

    double m;
    double Iz;
    double lf;
    double lr;

    double car_l;
    double car_w;

    double g;

    double r_in; // length from track center to inner boundary 
    double r_out; // length from track center to outter boundary

    double max_dist_proj;

    double e_long; // tire specific ellipse parameters (tire contraints)
    double e_eps;

    double max_alpha; // maximum slip angle for front tire

    double initial_velocity; // control input is delta_velocity, so we ahve to define initial value
    double s_trust_region;

    double vx_zero;

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
    double q_r;        // weight for yaw rate (angular velocity)
    double q_r_N_mult; // weight multiplication for terminal

    // Heading cost
    double q_mu; // weight for heading cost


    double q_beta;     // weight for slip
    int beta_kin_cost; // whether to use beta_kin_cost(=1) or beta_cost

    // Input cost (real input and input(delta))
    double r_D;     // weight for d(driving comment)
    double r_delta; // weight for steering change
    double r_vs;    // weight for 
    double r_dD;
    double r_dDelta;
    double r_dVs;


    double sc_quad_track;
    double sc_quad_tire;
    double sc_quad_alpha;

    double sc_lin_track;
    double sc_lin_tire;
    double sc_lin_alpha;

    CostParam();
    CostParam(std::string file);

};

class BoundsParam{
public:
    /// @brief  Lower bound of state
    /// @param X_l (double) lower bound of X
    /// @param Y_l (double) lower bound of Y
    /// @param phi_l (double) lower bound of phi (yaw)
    /// @param vx_l (double) lower bound of vx 
    /// @param vy_l (double) lower bound of vy 
    /// @param r_l (double) lower bound of r (yaw rate) 
    /// @param s_l (double) lower bound of s (path param) 
    /// @param D_l (double) lower bound of D (driving command) 
    /// @param delta_l (double) lower bound of delta (steering) 
    /// @param vs_l (double) lower bound of vs
    struct LowerStateBounds{
        double X_l;
        double Y_l;
        double phi_l;
        double vx_l;
        double vy_l;
        double r_l;
        double s_l;
        double D_l;
        double delta_l;
        double vs_l;
    };

    /// @brief  Upper bound of state
    /// @param X_l (double) upper bound of X
    /// @param Y_l (double) upper bound of Y
    /// @param phi_l (double) upper bound of phi (yaw)
    /// @param vx_l (double) upper bound of vx 
    /// @param vy_l (double) upper bound of vy 
    /// @param r_l (double) upper bound of r (yaw rate) 
    /// @param s_l (double) upper bound of s (path param) 
    /// @param D_l (double) upper bound of D (driving command) 
    /// @param delta_l (double) upper bound of delta (steering angle) 
    /// @param vs_l (double) upper bound of vs
    struct UpperStateBounds{
        double X_u;
        double Y_u;
        double phi_u;
        double vx_u;
        double vy_u;
        double r_u;
        double s_u;
        double D_u;
        double delta_u;
        double vs_u;
    };

    /// @brief  Lower bound of control input
    /// @param dD_l (double) lower bound of dD (driving command) 
    /// @param dDelta_l (double) lower bound of dDelta (steering angle) 
    /// @param dVs_l (double) lower bound of dVs
    struct LowerInputBounds{
        double dD_l;
        double dDelta_l;
        double dVs_l;
    };

    /// @brief  Upper bound of control input
    /// @param dD_l (double) upper bound of dD (driving command) 
    /// @param dDelta_l (double) upper bound of dDelta (steering angle) 
    /// @param dVs_l (double) upper bound of dVs
    struct UpperInputBounds{
        double dD_u;
        double dDelta_u;
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
