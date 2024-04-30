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

#include "cost.h"
namespace mpcc{
Cost::Cost() 
{
    std::cout << "default constructor, not everything is initialized properly" << std::endl;
}

Cost::Cost(const PathToJson &path, std::shared_ptr<RobotModel> robot) 
:cost_param_(CostParam(path.cost_path)),
param_(Param(path.param_path)),
robot_(robot)
{
}

TrackPoint Cost::getRefPoint(const ArcLengthSpline &track,const State &x) const
{
    // compute all the geometry information of the track at a given arc length
    const double s = x.s;

    // X-Y-Z postion of the reference at s
    const Eigen::Vector3d pos_ref = track.getPostion(s);
    const double x_ref = pos_ref(0);
    const double y_ref = pos_ref(1);
    const double z_ref = pos_ref(2);
    // reference path derivatives
    const Eigen::Vector3d dpos_ref = track.getDerivative(s);
    const double dx_ref = dpos_ref(0);
    const double dy_ref = dpos_ref(1);
    const double dz_ref = dpos_ref(2);
    // // angle of the reference path
    // const double theta_ref = atan2(dy_ref,dx_ref);
    // second order derivatives
    Eigen::Vector3d ddpos_ref = track.getSecondDerivative(s);
    const double ddx_ref = ddpos_ref(0);
    const double ddy_ref = ddpos_ref(1);
    const double ddz_ref = ddpos_ref(1);
    // // curvature
    // double dtheta_ref_nom = (dx_ref*ddy_ref - dy_ref*ddx_ref);
    // double dtheta_ref_denom = (dx_ref*dx_ref + dy_ref*dy_ref);
    // if(std::fabs(dtheta_ref_nom) < 1e-7)
    //     dtheta_ref_nom = 0;
    // if(std::fabs(dtheta_ref_denom) < 1e-7)
    //     dtheta_ref_denom = 1e-7;
    // double dtheta_ref = dtheta_ref_nom/dtheta_ref_denom;

    return {x_ref,y_ref,z_ref,dx_ref,dy_ref,dz_ref,ddx_ref,ddy_ref,ddz_ref};
}

ErrorInfo Cost::getErrorInfo(const ArcLengthSpline &track,const State &x) const
{
    ErrorInfo error_info;
    // compute error between reference and X-Y-Z position of the robot EE
    const JointVector q = stateToJointVector(x);
    const Eigen::Vector3d pos = robot_->getEEPosition(q);
    const TrackPoint track_point = getRefPoint(track,x);
    const Eigen::Vector3d total_error = pos - Eigen::Vector3d(track_point.x_ref,track_point.y_ref, track_point.z_ref);
    
    // lag error
    Eigen::Vector3d Tangent = Eigen::Vector3d(track_point.dx_ref, track_point.dy_ref, track_point.dz_ref); // tangent vector for ref point
    Eigen::Vector3d lag_error = (Tangent.dot(total_error)) * Tangent; // e_l = <T, e> * T

    // contouring error
    Eigen::Vector3d contouring_error = total_error - lag_error;

    // jacobian of the total error with respect to state X
    Eigen::Matrix<double,3,NX> d_total_error;
    d_total_error.setZero();
    d_total_error.block(0,0,3,PANDA_DOF) = robot_->getJacobianv(q);
    d_total_error.block(0,PANDA_DOF,3,1) = -Tangent;

    // jacobian of the lag error with respect to state X
    Eigen::Matrix<double,3,NX> d_Tangent;
    d_Tangent.setZero();
    d_Tangent.block(0,PANDA_DOF,3,1) = Eigen::Vector3d(track_point.ddx_ref, track_point.ddy_ref, track_point.ddz_ref); // normal vector for ref point
    Eigen::Matrix<double,3,NX> d_lag_error;
    d_lag_error.setZero();
    d_lag_error = (Tangent*Tangent.transpose()) * d_total_error + (Tangent*total_error.transpose() + lag_error.norm()*Eigen::MatrixXd::Identity(3,3)) * d_Tangent;
    
    // jacobian of the contouring error with respect to state X
    Eigen::Matrix<double,3,NX> d_contouring_error;
    d_contouring_error.setZero();
    d_contouring_error = d_total_error - d_lag_error;

    return {contouring_error,lag_error,d_contouring_error,d_lag_error};
}

CostMatrix Cost::getContouringCost(const ArcLengthSpline &track, const State &x,const int k) const
{
    // compute state cost, formed by contouring error cost + cost on "real" inputs
    // compute reference information
    const StateVector x_vec = stateToVector(x);
    // compute error and jacobian of error
    const ErrorInfo error_info = getErrorInfo(track,x);
    // contouring cost matrix
    Eigen::Vector2d ContouringCost;
    ContouringCost.setZero(2);
    ContouringCost(0) = k < N ? cost_param_.q_c : cost_param_.q_c_N_mult * cost_param_.q_c; // for contouring error
    ContouringCost(1) = cost_param_.q_l; // for lag error
    // contouring and lag error part
    Q_MPC Q_contouring_cost = Q_MPC::Zero();
    q_MPC q_contouring_cost = q_MPC::Zero();

    // Linearize contouring error function by jacobian. 
    // Error = d_error * State + error_zero
    Eigen::Matrix<double,3,NX> d_contouring_error = error_info.d_contouring_error;
    Eigen::Matrix<double,3,NX> d_lag_error = error_info.d_lag_error;

    Eigen::Vector3d contouring_error_zero = error_info.contouring_error - d_contouring_error*stateToVector(x);
    Eigen::Vector3d lag_error_zero = error_info.lag_error - d_lag_error*stateToVector(x);

    Q_contouring_cost = 2.0*ContouringCost(0)*d_contouring_error.transpose()*d_contouring_error +
                        2.0*ContouringCost(1)*d_lag_error.transpose()*d_lag_error;

    q_contouring_cost = 2.0*ContouringCost(0)*d_contouring_error.transpose()*contouring_error_zero +
                        2.0*ContouringCost(1)*d_lag_error.transpose()*lag_error_zero;
    
    // progress maximization part
    q_contouring_cost(si_index.vs) -= cost_param_.q_vs;

    // solver interface expects 0.5 x^T Q x + q^T x
    return {Q_contouring_cost,R_MPC::Zero(),S_MPC::Zero(),q_contouring_cost,r_MPC::Zero(),Z_MPC::Zero(),z_MPC::Zero()};
}

CostMatrix Cost::getInputCost(const ArcLengthSpline &track,const State &x, const Input &u) const
{
    // compute control input cost, formed by joint velocity, acceleration of path parameter 
    // and error between EE velocity and desired EE velocity
    Q_MPC Q_input_cost = Q_MPC::Zero();
    q_MPC q_input_cost = q_MPC::Zero();
    R_MPC R_input_cost = R_MPC::Zero();
    r_MPC r_input_cost = r_MPC::Zero();

    // // compute current EE linear velocity
    const JointVector q = stateToJointVector(x);
    Matrix<double, 3, PANDA_DOF> Jv = robot_->getJacobianv(q);
    const JointVector dq = inputToJointVector(u);
    // double ee_vel = (Jv * dq).norm();
    
    // // Linearization EE velocity error
    // double ee_vel_error = ee_vel - param_.desired_ee_velocity;
    // Matrix<double, PANDA_DOF, 1> d_ee_vel_error = (1 / ee_vel) * (Jv.transpose() * Jv * dq);
    // double ee_vel_error_zero = ee_vel_error - (d_ee_vel_error.transpose() * dq).value();

    // R_input_cost.block(si_index.dq1,si_index.dq1,PANDA_DOF,PANDA_DOF) = 2.0 * cost_param_.r_ee * d_ee_vel_error * d_ee_vel_error.transpose();
    // r_input_cost.segment(si_index.dq1,PANDA_DOF) = 2.0 * cost_param_.r_ee * d_ee_vel_error * ee_vel_error_zero;

    Matrix<double,3,1> ee_vel = Jv * dq;
    
    // Linearization EE velocity error
    Matrix<double,3,1> ee_vel_error = ee_vel - param_.desired_ee_velocity*track.getDerivative(x.s);
    double ee_vel_error_sqnorm = ee_vel_error.squaredNorm();
    double ds_ee_vel_error_sqnorm = -2*param_.desired_ee_velocity*ee_vel_error.transpose()*track.getSecondDerivative(x.s);
    double dss_ee_vel_error_sqnorm = 2*pow(param_.desired_ee_velocity,2)*track.getSecondDerivative(x.s).squaredNorm();
    Matrix<double, PANDA_DOF, 1> dq_ee_vel_error_sqnorm = 2*Jv.transpose()*ee_vel_error;
    Matrix<double, PANDA_DOF, PANDA_DOF> dqq_ee_vel_error_sqnorm = 2*Jv.transpose()*Jv;

    double ee_vel_error_sqnorm_s_zero = ds_ee_vel_error_sqnorm - dss_ee_vel_error_sqnorm*x.s;
    Matrix<double, PANDA_DOF, 1> ee_vel_error_sqnorm_dq_zero = dq_ee_vel_error_sqnorm - dqq_ee_vel_error_sqnorm*dq;

    Q_input_cost(si_index.s,si_index.s) = cost_param_.r_ee * dss_ee_vel_error_sqnorm;
    q_input_cost(si_index.s) = cost_param_.r_ee * ee_vel_error_sqnorm_s_zero;
    R_input_cost.block(si_index.dq1,si_index.dq1,PANDA_DOF,PANDA_DOF) = cost_param_.r_ee * dqq_ee_vel_error_sqnorm;
    r_input_cost.segment(si_index.dq1,PANDA_DOF) = cost_param_.r_ee * ee_vel_error_sqnorm_dq_zero;

    // for acceleration of path parameter
    r_input_cost(si_index.dVs) += cost_param_.r_dVs;

    // for joint velocity
    R_input_cost(si_index.dq1,si_index.dq1) += 2.0 * cost_param_.r_dq;
    R_input_cost(si_index.dq2,si_index.dq2) += 2.0 * cost_param_.r_dq;
    R_input_cost(si_index.dq3,si_index.dq3) += 2.0 * cost_param_.r_dq;
    R_input_cost(si_index.dq4,si_index.dq4) += 2.0 * cost_param_.r_dq;
    R_input_cost(si_index.dq5,si_index.dq5) += 2.0 * cost_param_.r_dq;
    R_input_cost(si_index.dq6,si_index.dq6) += 2.0 * cost_param_.r_dq;
    R_input_cost(si_index.dq7,si_index.dq7) += 2.0 * cost_param_.r_dq;

    // solver interface expects 0.5 u^T R u + r^T u
    return {Q_MPC::Zero(),R_input_cost,S_MPC::Zero(),q_MPC::Zero(),r_input_cost,Z_MPC::Zero(),z_MPC::Zero()};
}

CostMatrix Cost::getSoftConstraintCost() const
{
    // input cost and rate of chage of real inputs
    Z_MPC Z_cost = Z_MPC::Identity();
    z_MPC z_cost = z_MPC::Ones();

    Z_cost(si_index.con_sing,si_index.con_sing) = cost_param_.sc_quad_sing;
    Z_cost(si_index.con_selcol,si_index.con_selcol) = cost_param_.sc_quad_selcol;

    z_cost(si_index.con_sing) = cost_param_.sc_lin_sing;
    z_cost(si_index.con_selcol) = cost_param_.sc_lin_selcol;

    return {Q_MPC::Zero(),R_MPC::Zero(),S_MPC::Zero(),q_MPC::Zero(),r_MPC::Zero(),Z_cost,z_cost};
}

CostMatrix Cost::getCost(const ArcLengthSpline &track,const State &x,const Input &u,const int k) const
{
    // generate quadratic cost function
    const CostMatrix contouring_cost = getContouringCost(track,x,k);
    // const CostMatrix heading_cost = getHeadingCost(track,x,k);
    const CostMatrix input_cost = getInputCost(track,x,u);

    const CostMatrix soft_con_cost = getSoftConstraintCost();

    Q_MPC Q_not_sym = contouring_cost.Q + input_cost.Q;
    // Q_MPC Q_reg = 1e-9*Q_MPC::Identity();
    R_MPC R_not_sym = contouring_cost.R + input_cost.R;
    // R_MPC R_reg = 1e-9*R_MPC::Identity();

    // const Q_MPC Q = Q_not_sym;
    const Q_MPC Q = 0.5*(Q_not_sym.transpose()+Q_not_sym); //+ Q_reg;//contouring_cost.Q + input_cost.Q + beta_cost.Q;
    // const R_MPC R = contouring_cost.R + input_cost.R;
    const R_MPC R = 0.5*(R_not_sym.transpose()+R_not_sym); // + R_reg;
    const q_MPC q = contouring_cost.q + input_cost.q;
    const r_MPC r = contouring_cost.r + input_cost.r;
    const Z_MPC Z = soft_con_cost.Z;
    const z_MPC z = soft_con_cost.z;


    return {Q,R,S_MPC::Zero(),q,r,Z,z};
}
}