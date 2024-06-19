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
    double max_dist_proj;
    double desired_ee_velocity; // desired end-effector velocity
    double s_trust_region;
    double deacc_ratio; // starting ratio to deaccelerate movement of s(path parameter)

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
    double r_ddq;    // weight for joint acceleration
    double r_dVs;    // weight for accelerate of path parameter


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
    /// @param dq1_l (double) lower bound of dq1
    /// @param dq2_l (double) lower bound of dq2
    /// @param dq3_l (double) lower bound of dq3
    /// @param dq4_l (double) lower bound of dq4
    /// @param dq5_l (double) lower bound of dq5
    /// @param dq6_l (double) lower bound of dq6
    /// @param dq7_l (double) lower bound of dq7
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
        double dq1_l;
        double dq2_l;
        double dq3_l;
        double dq4_l;
        double dq5_l;
        double dq6_l;
        double dq7_l;
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
    /// @param dq1_u (double) upper bound of dq1
    /// @param dq2_u (double) upper bound of dq2
    /// @param dq3_u (double) upper bound of dq3
    /// @param dq4_u (double) upper bound of dq4
    /// @param dq5_u (double) upper bound of dq5
    /// @param dq6_u (double) upper bound of dq6
    /// @param dq7_u (double) upper bound of dq7
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
        double dq1_u;
        double dq2_u;
        double dq3_u;
        double dq4_u;
        double dq5_u;
        double dq6_u;
        double dq7_u;
        double s_u;
        double vs_u;
    };

    /// @brief  Lower bound of control input
    /// @param ddq1_l (double) lower bound of ddq1
    /// @param ddq2_l (double) lower bound of ddq2
    /// @param ddq3_l (double) lower bound of ddq3
    /// @param ddq4_l (double) lower bound of ddq4
    /// @param ddq5_l (double) lower bound of ddq5
    /// @param ddq6_l (double) lower bound of ddq6
    /// @param ddq7_l (double) lower bound of ddq7
    /// @param dVs_l  (double) lower bound of dVs
    struct LowerInputBounds{
        double ddq1_l;
        double ddq2_l;
        double ddq3_l;
        double ddq4_l;
        double ddq5_l;
        double ddq6_l;
        double ddq7_l;
        double dVs_l;
    };

    /// @brief  Upper bound of control input
    /// @param ddq1_u (double) upper bound of ddq1
    /// @param ddq2_u (double) upper bound of ddq2
    /// @param ddq3_u (double) upper bound of ddq3
    /// @param ddq4_u (double) upper bound of ddq4
    /// @param ddq5_u (double) upper bound of ddq5
    /// @param ddq6_u (double) upper bound of ddq6
    /// @param ddq7_u (double) upper bound of ddq7
    /// @param dVs_u  (double) upper bound of dVs
    struct UpperInputBounds{
        double ddq1_u;
        double ddq2_u;
        double ddq3_u;
        double ddq4_u;
        double ddq5_u;
        double ddq6_u;
        double ddq7_u;
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

    NormalizationParam();
    NormalizationParam(std::string file);
};

class SQPParam{
    public:
        double eps_prim;
        double eps_dual;
        unsigned int max_iter;
        unsigned int line_search_max_iter;
        bool do_SOC;
        bool use_BFGS;

        double line_search_tau;
        double line_search_eta;
        double line_search_rho;

        SQPParam();
        SQPParam(std::string file);
};
}
#endif //MPCC_PARAMS_H
