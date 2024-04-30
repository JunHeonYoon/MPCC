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

#ifndef MPCC_COST_H
#define MPCC_COST_H

#include "config.h"
#include "types.h"
#include "Spline/arc_length_spline.h"

namespace mpcc{

/// @brief 
/*
  second and first order term of cost in QP:
  min 0.5 x^T Q x + 0.5 u^T R u + 0.5 x^T S u + q^T x + r^T u + 0.5 s^T Z s + z^T s
  where x is state(NX)
        u is control input(NU)
        s is slack variable (NS)
*/ 
/// @param Q (Eigen::MatrixXd) second order term for state
/// @param R (Eigen::MatrixXd) second order term for control inut
/// @param S (Eigen::MatrixXd) second order term for state and control input
/// @param q (Eigen::MatrixXd) first order term for state
/// @param r (Eigen::MatrixXd) first order term for control inut
/// @param Z (Eigen::MatrixXd) second order term for slack variable
/// @param z (Eigen::MatrixXd) first order term for slack variable
struct CostMatrix{
    Q_MPC Q; // for stadyrodcon
    R_MPC R; // for control input
    S_MPC S; // for state and control input
    q_MPC q; // for state 
    r_MPC r; // for control input
    Z_MPC Z;
    z_MPC z;
};

/// @brief refernce X-Y-Z path position and its derivates wrt arc length (s)
/// @param x_ref (double) reference X position data
/// @param y_ref (double) reference Y position data
/// @param z_ref (double) reference Z position data
/// @param dx_ref (double) reference X'(s) position data
/// @param dy_ref (double) reference Y'(s) position data
/// @param dz_ref (double) reference Z'(s) position data
/// @param ddx_ref (double) reference X''(s) position data
/// @param ddy_ref (double) reference Y''(s) position data
/// @param ddz_ref (double) reference Z''(s) position data
struct TrackPoint{
    const double x_ref;
    const double y_ref;
    const double z_ref;
    const double dx_ref;
    const double dy_ref;
    const double dz_ref;
    const double ddx_ref;
    const double ddy_ref;
    const double ddz_ref;
    // const double theta_ref;
    // const double dtheta_ref;
};

/// @brief error between reference and X-Y-Z position of the end-effector
/// @param contouring_error (Eigen::Vector3d) contouring error
/// @param lag_error (Eigen::Vector3d) lag error
/// @param d_contouring_error (Eigen::Matrix<double,3,NX>) derivatives of the contouring error with respect to state
/// @param d_lag_error (Eigen::Matrix<double,3,NX>) derivatives of the lag error with respect to state
struct ErrorInfo{
    const Eigen::Vector3d contouring_error;
    const Eigen::Vector3d lag_error;
    const Eigen::Matrix<double,3,NX> d_contouring_error;
    const Eigen::Matrix<double,3,NX> d_lag_error;
};

class Cost {
public:
    Cost(const PathToJson &path, std::shared_ptr<RobotModel> robot);
    Cost();
    
    /// @brief compute cost for contouring error, heading error, control input, beta (side slip angle), soft constraint given current state
    /// @param track (ArcLengthSpline) reference track
    /// @param x (State) current state
    /// @param u (Input) current control input
    /// @param k (int) receding horizon index
    /// @return (CostMatrix) second order approximation matrix of cost
    CostMatrix getCost(const ArcLengthSpline &track,const State &x,const Input &u,int k) const;

private:
    /// @brief compute all the geometry information of the track at a given current arc length
    /// @param track (ArcLengthSpline) reference track
    /// @param x (State) current state
    /// @return (TrackPoint) reference X-Y-theta path position and its derivates wrt arc length (s)
    TrackPoint getRefPoint(const ArcLengthSpline &track,const State &x) const;

    /// @brief compute error between reference track and X-Y position of the car
    /// @param track (ArcLengthSpline) reference track
    /// @param x (State) current state
    /// @return (ErrorInfo) contouring and lag error and its derivatives wrt state
    ErrorInfo  getErrorInfo(const ArcLengthSpline &track,const State &x) const;

    /// @brief compute contouring cost given current state
    /// @param track (ArcLengthSpline) reference track
    /// @param x (State) current state
    /// @param k (int) receding horizon index
    /// @return (CostMatrix) second order approximation (wrt current state) matrix of contouring cost
    CostMatrix getContouringCost(const ArcLengthSpline &track, const State &x,int k) const;

    // /// @brief compute heading angle (yaw) cost given current state
    // /// @param track (ArcLengthSpline) reference track
    // /// @param x (State) current state
    // /// @param k (int) receding horizon index
    // /// @return (CostMatrix) second order approximation (wrt current state) matrix of heading cost
    // CostMatrix getHeadingCost(const ArcLengthSpline &track, const State &x,int k) const;

    /// @brief compute control input cost
    /// @param track (ArcLengthSpline) reference track
    /// @param x (State) current state
    /// @param u (Input) current control input
    /// @return (CostMatrix) cost matrix for control input
    CostMatrix getInputCost(const ArcLengthSpline &track,const State &x, const Input &u) const;

    /// @brief compute soft constraint cost
    /// @return (CostMatrix) cost matrix for slack variables
    CostMatrix getSoftConstraintCost() const;

    CostParam cost_param_;
    Param param_;
    std::shared_ptr<RobotModel> robot_;
};
}
#endif //MPCC_COST_H
