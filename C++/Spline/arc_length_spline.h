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

#ifndef MPCC_ARC_LENGTH_SPLINE_H
#define MPCC_ARC_LENGTH_SPLINE_H

#include "cubic_spline.h"
#include "types.h"
#include "Params/params.h"
#include <map>

namespace mpcc{

/// @brief raw path data
/// @param X (Eigen::VectorXd) X position data
/// @param Y (Eigen::VectorXd) Y position data
struct RawPath{
    Eigen::VectorXd X;
    Eigen::VectorXd Y;
};

/// @brief arc length path data
/// @param X (Eigen::VEctorXd) X position data
/// @param Y (Eigen::VEctorXd) Y position data
/// @param s (Eigen::VEctorXd) arc length
/// @param n_points (int) number of path points
struct PathData{
    Eigen::VectorXd X;
    Eigen::VectorXd Y;
    Eigen::VectorXd s;
    int n_points;
};

class ArcLengthSpline {
public:
    ArcLengthSpline();
    ArcLengthSpline(const PathToJson &path);

    /// @brief  generate 2-D arc length parametrized spline given X-Y position path data
    /// @param X (Eigen::VectorXd) X position data
    /// @param Y (Eigen::VectorXd) Y position data
    void gen2DSpline(const Eigen::VectorXd &X,const Eigen::VectorXd &Y);

    /// @brief get X-Y position data given arc length (s)
    /// @param s (double) arc length 
    /// @return (Eigen::Vector2d) X-Y position data
    Eigen::Vector2d getPostion(double) const;

    /// @brief get X'(s)-Y'(s) position data derivatived by arc length (s) given arc length (s)
    /// @param  (double) arc length 
    /// @return (Eigen::Vector2d) X'(s)-Y'(s) position data
    Eigen::Vector2d getDerivative(double) const;

    /// @brief get X''(s)-Y''(s) position data twice derivatived by arc length (s) given arc length (s)
    /// @param  (double) arc length 
    /// @return (Eigen::Vector2d) X''(s)-Y''(s) position data
    Eigen::Vector2d getSecondDerivative(double) const;

    /// @brief get total arc length of splined path 
    /// @return (double) total arc length
    double getLength() const;

    /// @brief compute arc length of projected on splined path which calculated by Newton-Euler method given current state
    /// @param x (State) current state
    /// @return (double) projected arc lenth
    double projectOnSpline(const State &x) const;

    // void setParam(const Param &param) { param_ = param; };

private:
    /// @brief set irregular path point data to make path data (PathData)
    /// @param X_in (Eigen::VEctorXd) X position data
    /// @param Y_in (Eigen::VEctorXd) Y position data
    void setData(const Eigen::VectorXd &X_in,const Eigen::VectorXd &Y_in);

    /// @brief set regular path point data to make path data (PathData)
    /// @param X_in (Eigen::VEctorXd) X position data
    /// @param Y_in (Eigen::VEctorXd) Y position data
    /// @param s_in (Eigen::VEctorXd) arc length
    void setRegularData(const Eigen::VectorXd &X_in,const Eigen::VectorXd &Y_in,const Eigen::VectorXd &s_in);

    /// @brief compute arc length (s) given X, Y position points
    /// @param X_in (Eigen::VectorXd) X position data
    /// @param Y_in (Eigen::VectorXd) Y position data
    /// @return (Eigen::VectorXd) arc length data
    Eigen::VectorXd compArcLength(const Eigen::VectorXd &X_in,const Eigen::VectorXd &Y_in) const;

    /// @brief  re-sample arc length parametrized X-Y spline path with N_spline data points using equidistant arc length values
    /// @param initial_spline_x (CubicSpline) X position parameterized by arc length (S) 
    /// @param initial_spline_y (CubicSpline) Y position parameterized by arc length (S) 
    /// @param total_arc_length (double) total arc length
    /// @return (PathData) re-sampled x, y position path data with arc length (s)
    PathData resamplePath(const CubicSpline &initial_spline_x,const CubicSpline &initial_spline_y,double total_arc_length) const;

    /// @brief remove points which are not at all equally spaced, to avoid fitting problems
    /// @param X_original (Eigen::VectorXd) X position data
    /// @param Y_original (Eigen::VectorXd) Y position data
    /// @return (RawPath) x, y position path data 
    RawPath outlierRemoval(const Eigen::VectorXd &X_original,const Eigen::VectorXd &Y_original) const;

    /// @brief generate cubic-splined X, Y position path points parameterized by arc length (S)
    /// @param X (Eigen::VectorXd) X position data
    /// @param Y (Eigen::VectorXd) Y position data
    void fitSpline(const Eigen::VectorXd &X,const Eigen::VectorXd &Y);

    /// @brief if arc length (s) is larger than total arc length, then s become unwrapped. (like angle is -pi ~ pi)
    /// @param x (double) arc length
    /// @return (double) unwrapped arc length (s) data
    double unwrapInput(double x) const;

    PathData path_data_;      // initial data and data used for successive fitting
//    PathData pathDataFinal; // final data
    CubicSpline spline_x_;
    CubicSpline spline_y_;
    Param param_;
};
}
#endif //MPCC_ARC_LENGTH_SPLINE_H