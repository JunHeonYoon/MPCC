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

#include "constraints.h"
namespace mpcc{
Constraints::Constraints()
{   
    std::cout << "default constructor, not everything is initialized properly" << std::endl;
}

Constraints::Constraints(double Ts,const PathToJson &path,std::shared_ptr<RobotModel> robot,std::shared_ptr<SelCollNNmodel> selcolNN) 
:model_(Ts,path),
param_(Param(path.param_path)),
robot_(robot),
selcolNN_(selcolNN)
{
}

OneDConstraint Constraints::getSelcollConstraint(const State &x,const Input &u) const
{
    // compute self-collision constraints
    // -∇_q Γ(q) * q_dot <= -RBF(Γ(q) - r), where r is buffer
    const JointVector q = stateToJointVector(x);
    const JointVector dq = inputToJointVector(u);

    // compute minimum distance between each links and its derivative
    auto y_pred = selcolNN_->calculateMlpOutput(q, false); // first: min_dist, second: derivative wrt q
    double min_dist = y_pred.first[0]; // unit: [cm]
    Eigen::VectorXd d_min_dist = y_pred.second.transpose();
    Eigen::Matrix<double, PANDA_DOF, PANDA_DOF> dd_min_dist = d_min_dist * d_min_dist.transpose(); // hessian matrix (approximation)

    // compute RBF value of minimum distance and its derivative
    double r = 3; // buffer [cm]
    double delta = 5; // switching point of RBF
    double RBF = 5*getRBF(delta, min_dist - r);
    double d_RBF = 5*getDRBF(delta, min_dist - r);

    C_i_MPC C_selcol_matrix = C_i_MPC::Zero();;
    D_i_MPC D_selcol_matrix = D_i_MPC::Zero();;
    double selcol_constrint_lower = 0, selcol_constrint_upper = 0;

    C_selcol_matrix.block(si_index.q1,si_index.q1,1,PANDA_DOF) = (-dd_min_dist*dq + d_RBF*d_min_dist).transpose();
    D_selcol_matrix.block(si_index.dq1,si_index.dq1,1,PANDA_DOF) = -d_min_dist.transpose();
    selcol_constrint_lower = -INF;
    selcol_constrint_upper = (d_min_dist.transpose()*dq).value() - RBF + (C_selcol_matrix*stateToVector(x)).value() + (D_selcol_matrix*inputToVector(u)).value();

    return {C_selcol_matrix,D_selcol_matrix,selcol_constrint_lower,selcol_constrint_upper};
}

OneDConstraint Constraints::getSingularConstraint(const State &x,const Input &u) const
{
    // compute singularity constraints
    // -∇_q μ(q) * q_dot <= -RBF(μ(q) - ɛ), where ɛ is buffer
    const JointVector q = stateToJointVector(x);
    const JointVector dq = inputToJointVector(u);

    // compute manipulability and its derivative
    double manipulability = 100*robot_->getManipulability(q);
    Eigen::VectorXd d_manipulability = 100*robot_->getDManipulability(q);
    Eigen::Matrix<double, PANDA_DOF, PANDA_DOF> dd_manipulability = d_manipulability * d_manipulability.transpose(); // hessian matrix (approximation)
    // cout<<"mani               : "<<manipulability<<std::endl;
    // cout<<"d_mani*q_dot       : "<<d_manipulability.dot(dq)<<std::endl;
    // cout<<"mani - d_mani*q_dot: "<<manipulability-d_manipulability.dot(dq) <<std::endl;

    // compute RBF value of manipulability and its derivative
    double eps = 100*0.03;    // buffer 
    double delta = 100*0.05;  // switching point of RBF
    double RBF = getRBF(delta, manipulability - eps);
    double d_RBF = getDRBF(delta, manipulability - eps);
    // std::cout<< -d_manipulability.dot(dq) << " <= " << -RBF << std::endl;


    C_i_MPC C_sing_matrix = C_i_MPC::Zero();
    D_i_MPC D_sing_matrix = D_i_MPC::Zero();
    D_sing_matrix.setZero();
    double sing_constrint_lower = 0, sing_constrint_upper = 0;

    C_sing_matrix.block(si_index.q1,si_index.q1,1,PANDA_DOF) = (-dd_manipulability*dq + d_RBF*d_manipulability).transpose();
    D_sing_matrix.block(si_index.dq1,si_index.dq1,1,PANDA_DOF) = -d_manipulability.transpose();
    sing_constrint_lower = -INF;
    sing_constrint_upper = (d_manipulability.transpose()*dq).value() - RBF + (C_sing_matrix*stateToVector(x)).value() + (D_sing_matrix*inputToVector(u)).value();


    return {C_sing_matrix,D_sing_matrix,sing_constrint_lower,sing_constrint_upper};
}


double Constraints::getRBF(double delta, double h) const
{
    // Grandia, Ruben, et al. 
    // "Feedback mpc for torque-controlled legged robots." 
    // 2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2019.
    double result;
    if (h >= delta) result = -log(h);
    else            result = ( pow( (h-2*delta) / delta ,2) - 1 ) / 2 - log(delta);
    return result;
}

double Constraints::getDRBF(double delta, double h) const
{
    // Grandia, Ruben, et al. 
    // "Feedback mpc for torque-controlled legged robots." 
    // 2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2019.
    double result;
    if (h >= delta) result = -1/h;
    else            result = (h-2*delta) / delta;
    return result;
}

ConstrainsMatrix Constraints::getConstraints(const ArcLengthSpline &track,const State &x,const Input &u) const
{
    // compute all the polytopic state constraints
    // compute the three constraints

    ConstrainsMatrix constrains_matrix;
    const OneDConstraint selcol_constraints = getSelcollConstraint(x,u);
    const OneDConstraint sing_constraints = getSingularConstraint(x,u);

    C_MPC C_constrains_matrix;
    D_MPC D_constrains_matrix;
    d_MPC dl_constrains_matrix;
    d_MPC du_constrains_matrix;

    C_constrains_matrix.row(si_index.con_selcol) = selcol_constraints.C_i;
    D_constrains_matrix.row(si_index.con_selcol) = selcol_constraints.D_i;
    dl_constrains_matrix(si_index.con_selcol) = selcol_constraints.dl_i;
    du_constrains_matrix(si_index.con_selcol) = selcol_constraints.du_i;
    C_constrains_matrix.row(si_index.con_sing) = sing_constraints.C_i;
    D_constrains_matrix.row(si_index.con_sing) = sing_constraints.D_i;
    dl_constrains_matrix(si_index.con_sing) = sing_constraints.dl_i;
    du_constrains_matrix(si_index.con_sing) = sing_constraints.du_i;

    return {C_constrains_matrix,D_constrains_matrix,dl_constrains_matrix,du_constrains_matrix};
}
}