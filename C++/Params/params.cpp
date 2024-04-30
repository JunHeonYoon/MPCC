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

#include "params.h"
namespace mpcc{
    
Param::Param(){
    std::cout << "Default initialization of model params" << std::endl;
}

Param::Param(std::string file){
    /////////////////////////////////////////////////////
    // Loading Model and Constraint Parameters //////////
    /////////////////////////////////////////////////////
    // std::cout << "model" << std::endl;

    std::ifstream iModel(file);
    json jsonModel;
    iModel >> jsonModel;
    
//     // Model Parameters
//     Cm1 = jsonModel["Cm1"];
//     Cm2 = jsonModel["Cm2"];
//     Cr0 = jsonModel["Cr0"];
//     Cr2 = jsonModel["Cr2"];

//     Br 	= jsonModel["Br"];
//     Cr 	= jsonModel["Cr"];
//     Dr 	= jsonModel["Dr"];

//     Bf 	= jsonModel["Bf"];
//     Cf 	= jsonModel["Cf"];
//     Df 	= jsonModel["Df"];

//     m 	= jsonModel["m"];
//     Iz 	= jsonModel["Iz"];
//     lf 	= jsonModel["lf"];
//     lr 	= jsonModel["lr"];

//     car_l = jsonModel["car_l"];
//     car_w = jsonModel["car_w"];
    
//     g = jsonModel["g"];

//     //Constraint Parameters
//     r_in = jsonModel["R_in"];
//     r_out = jsonModel["R_out"];

    max_dist_proj = jsonModel["max_dist_proj"];

//     e_long = jsonModel["E_long"];
//     e_eps = jsonModel["E_eps"];

//     max_alpha = jsonModel["maxAlpha"];

//     // initial warm start and trust region (model dependent)
    desired_ee_velocity = jsonModel["desired_ee_velocity"];
    s_trust_region = jsonModel["s_trust_region"];

//     vx_zero = jsonModel["vx_zero"];
}

CostParam::CostParam(){
    std::cout << "Default initialization of cost" << std::endl;
}

CostParam::CostParam(std::string file){
    /////////////////////////////////////////////////////
    // Loading Cost Parameters //////////////////////////
    /////////////////////////////////////////////////////
    // std::cout << "cost" << std::endl;

    std::ifstream iCost(file);
    json jsonCost;
    iCost >> jsonCost;

    q_c = jsonCost["qC"];
    q_c_N_mult = jsonCost["qCNmult"];
    q_l = jsonCost["qL"];
    q_vs = jsonCost["qVs"];

    q_ori = jsonCost["qOri"];

    r_dq = jsonCost["rdq"];
    r_dVs = jsonCost["rdVs"];
    r_ee = jsonCost["rEE"];


    sc_quad_selcol = jsonCost["sc_quad_selcol"];
    sc_quad_sing= jsonCost["sc_quad_sing"];

    sc_lin_selcol = jsonCost["sc_lin_selcol"];
    sc_lin_sing = jsonCost["sc_lin_sing"];
}

BoundsParam::BoundsParam() {
    std::cout << "Default initialization of bounds" << std::endl;
}

BoundsParam::BoundsParam(std::string file) {

    /////////////////////////////////////////////////////
    // Loading Cost Parameters //////////////////////////
    /////////////////////////////////////////////////////
    // std::cout << "bounds" << std::endl;

    std::ifstream iBounds(file);
    json jsonBounds;
    iBounds >> jsonBounds;

    lower_state_bounds.q1_l = jsonBounds["q1l"];
    lower_state_bounds.q2_l = jsonBounds["q2l"];
    lower_state_bounds.q3_l = jsonBounds["q3l"];
    lower_state_bounds.q4_l = jsonBounds["q4l"];
    lower_state_bounds.q5_l = jsonBounds["q5l"];
    lower_state_bounds.q6_l = jsonBounds["q6l"];
    lower_state_bounds.q7_l = jsonBounds["q7l"];
    lower_state_bounds.s_l  = jsonBounds["sl"];
    lower_state_bounds.vs_l = jsonBounds["vsl"];

    upper_state_bounds.q1_u = jsonBounds["q1u"];
    upper_state_bounds.q2_u = jsonBounds["q2u"];
    upper_state_bounds.q3_u = jsonBounds["q3u"];
    upper_state_bounds.q4_u = jsonBounds["q4u"];
    upper_state_bounds.q5_u = jsonBounds["q5u"];
    upper_state_bounds.q6_u = jsonBounds["q6u"];
    upper_state_bounds.q7_u = jsonBounds["q7u"];
    upper_state_bounds.s_u  = jsonBounds["su"];
    upper_state_bounds.vs_u = jsonBounds["vsu"];

    lower_input_bounds.dq1_l = jsonBounds["dq1l"];
    lower_input_bounds.dq2_l = jsonBounds["dq2l"];
    lower_input_bounds.dq3_l = jsonBounds["dq3l"];
    lower_input_bounds.dq4_l = jsonBounds["dq4l"];
    lower_input_bounds.dq5_l = jsonBounds["dq5l"];
    lower_input_bounds.dq6_l = jsonBounds["dq6l"];
    lower_input_bounds.dq7_l = jsonBounds["dq7l"];
    lower_input_bounds.dVs_l = jsonBounds["dVsl"];

    upper_input_bounds.dq1_u = jsonBounds["dq1u"];
    upper_input_bounds.dq2_u = jsonBounds["dq2u"];
    upper_input_bounds.dq3_u = jsonBounds["dq3u"];
    upper_input_bounds.dq4_u = jsonBounds["dq4u"];
    upper_input_bounds.dq5_u = jsonBounds["dq5u"];
    upper_input_bounds.dq6_u = jsonBounds["dq6u"];
    upper_input_bounds.dq7_u = jsonBounds["dq7u"];
    upper_input_bounds.dVs_u = jsonBounds["dVsu"];
}

NormalizationParam::NormalizationParam(){
    std::cout << "Default initialization of normalization" << std::endl;
}

NormalizationParam::NormalizationParam(std::string file)
{
    /////////////////////////////////////////////////////
    // Loading Normalization Parameters /////////////////
    /////////////////////////////////////////////////////
    // std::cout << "norm" << std::endl;

    std::ifstream iNorm(file);
    json jsonNorm;
    iNorm >> jsonNorm;

    T_x.setIdentity();
    T_x(si_index.q1,si_index.q1) = jsonNorm["q1"];
    T_x(si_index.q2,si_index.q2) = jsonNorm["q2"];
    T_x(si_index.q3,si_index.q3) = jsonNorm["q3"];
    T_x(si_index.q4,si_index.q4) = jsonNorm["q4"];
    T_x(si_index.q5,si_index.q5) = jsonNorm["q5"];
    T_x(si_index.q6,si_index.q6) = jsonNorm["q6"];
    T_x(si_index.q7,si_index.q7) = jsonNorm["q7"];
    T_x(si_index.s,si_index.s) = jsonNorm["s"];
    T_x(si_index.vs,si_index.vs) = jsonNorm["vs"];


    T_x_inv.setIdentity();
    for(int i = 0;i<NX;i++)
    {
        T_x_inv(i,i) = 1.0/T_x(i,i);
    }

    T_u.setIdentity();
    T_u(si_index.dq1,si_index.q1) = jsonNorm["dq1"];
    T_u(si_index.dq2,si_index.q2) = jsonNorm["dq2"];
    T_u(si_index.dq3,si_index.q3) = jsonNorm["dq3"];
    T_u(si_index.dq4,si_index.q4) = jsonNorm["dq4"];
    T_u(si_index.dq5,si_index.q5) = jsonNorm["dq5"];
    T_u(si_index.dq6,si_index.q6) = jsonNorm["dq6"];
    T_u(si_index.dq7,si_index.q7) = jsonNorm["dq7"];
    T_u(si_index.dVs,si_index.dVs) = jsonNorm["dVs"];

    T_u_inv.setIdentity();
    for(int i = 0;i<NU;i++)
    {
        T_u_inv(i,i) = 1.0/T_u(i,i);
    }

    T_s.setIdentity();
    T_s_inv.setIdentity();
}

}
