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

/// @brief 1-D inequality constraint wrt state:
// dl_i <= C_i xk<= du_i
/// @param C_i (Eigen::MatrixXd) polytopic input constraints
/// @param dl (double) lower bounds
/// @param du (double) upper bounds
struct OneDConstraint {
    const C_i_MPC C_i;
    const double dl_i;
    const double du_i;
};

class Constraints {
public:
    Constraints();
    Constraints(double Ts,const PathToJson &path);
    
    /// @brief compute all the polytopic state constraints given current state
    /// @param track (ArcLengthSpline) reference track
    /// @param x (State) current state
    /// @param u (ControlInput) current control input
    /// @return (ConstrainsMatrix) polytopic inequality constraint matrix and bound vectors
    ConstrainsMatrix getConstraints(const ArcLengthSpline &track,const State &x,const Input &u) const;

private:
    /// @brief compute 1-D track inequality constraint given current state
    /// @param track (ArcLengthSpline) reference track
    /// @param x (State) current state
    /// @return (OneDConstraint) constraint matrix and bound wrt state
    OneDConstraint getTrackConstraints(const ArcLengthSpline &track,const State &x) const;

    /// @brief compute 1-D tire friction elipse inequality constraint given current state
    /// @param x (State) current state
    /// @return (OneDConstraint) constraint matrix and bound wrt state
    OneDConstraint getTireConstraintRear(const State &x) const;

    /// @brief compute tire friction constraint jacobian wrt current state
    /// @param x (State) current state
    /// @return (Eigen::MatrixXd) tire friction constraint jacobian
    C_i_MPC getTireConstraintRearJac(const State &x) const;

    /// @brief compute 1-D slip angle inequality constraint given current state
    /// @param x (State) current state
    /// @return (Eigen::MatrixXd) tire friction constraint jacobian
    OneDConstraint getAlphaConstraintFront(const State &x) const;

    /// @brief compute slip angle constraint jacobian wrt current state
    /// @param x (State) current state
    /// @return (Eigen::MatrixXd) slip angle constraint jacobian
    C_i_MPC getAlphaConstraintFrontJac(const State &x) const;

    Model model_;
    Param param_;
};
}

#endif //MPCC_CONSTRAINTS_H
