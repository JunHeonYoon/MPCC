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

#ifndef MPCC_CONSTRAINTS_H
#define MPCC_CONSTRAINTS_H

#include "config.h"
#include "Spline/arc_length_spline.h"
#include "Model/model.h"
#include "SelfCollision/SelfCollisionModel.h"
namespace mpcc{

/// @brief polytopic inequality constraint matrix and bound vectors:
// dl <= C xk + D uk <= du
/// @param C (Eigen::MatrixXd) polytopic state constraints
/// @param D (Eigen::MatrixXd) polytopic input constraints
/// @param dl (Eigen::MatrixXd) lower bounds
/// @param du (Eigen::MatrixXd) upper bounds
struct ConstrainsMatrix {
    // dl <= C xk + D uk <= du
    C_MPC C;    //polytopic state constraints
    D_MPC D;    //polytopic input constraints
    d_MPC dl;   //lower bounds
    d_MPC du;   //upper bounds
};

/// @brief 1-D inequality constraint wrt state and input:
// dl_i <= C_i xk + D_i uk<= du_i
/// @param C_i (Eigen::MatrixXd) polytopic state constraints
/// @param D_i (Eigen::MatrixXd) polytopic input constraints
/// @param dl (double) lower bounds
/// @param du (double) upper bounds
struct OneDConstraint {
    const C_i_MPC C_i;
    const D_i_MPC D_i;
    const double dl_i;
    const double du_i;
};

class Constraints {
public:
    Constraints();
    Constraints(double Ts,const PathToJson &path,std::shared_ptr<RobotModel> robot,std::shared_ptr<SelCollNNmodel> selcolNN);
    
    /// @brief compute all the polytopic state constraints given current state
    /// @param track (ArcLengthSpline) reference track
    /// @param x (State) current state
    /// @param u (ControlInput) current control input
    /// @return (ConstrainsMatrix) polytopic inequality constraint matrix and bound vectors
    ConstrainsMatrix getConstraints(const ArcLengthSpline &track,const State &x,const Input &u) const;

private:
    /// @brief compute self-collision inequality constraint given current state and input
    /// @param x (State) current state
    /// @param u (ControlInput) current control input
    /// @return (ConstrainsMatrix) constraint matrix and bound wrt state and input
    OneDConstraint getSelcollConstraint(const State &x,const Input &u) const;


    /// @brief compute Singularity inequality constraint given current state and input
    /// @param x (State) current state
    /// @param u (ControlInput) current control input
    /// @return (Eigen::MatrixXd) constraint matrix and bound wrt state and input
    OneDConstraint getSingularConstraint(const State &x,const Input &u) const;


    /// @brief compute Relaxed Barrier Function of h
    /// @param delta (double) switching point from logarithm to quadratic function
    /// @param h(double) input value
    /// @return (double) RBF value
    double getRBF(double delta, double h) const;

    /// @brief compute derivative ofRelaxed Barrier Function wrt h
    /// @param delta (double) switching point from logarithm to quadratic function
    /// @param h(double) input value
    /// @return (double) derivation RBF value
    double getDRBF(double delta, double h) const;

    Model model_;
    Param param_;
    std::shared_ptr<RobotModel> robot_;
    std::shared_ptr<SelCollNNmodel> selcolNN_;
};
}

#endif //MPCC_CONSTRAINTS_H
