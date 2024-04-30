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

#ifndef MPCC_TRACK_H
#define MPCC_TRACK_H

#include "config.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <nlohmann/json.hpp>

namespace mpcc {
//used namespace
using json = nlohmann::json;

/// @brief Tracking waypoints
/// @param X   (const Eigen::VectorXd) X waypoints
/// @param Y   (const Eigen::VectorXd) y waypoints
/// @param Z   (const Eigen::VectorXd) z waypoints
/// @param R11 (const Eigen::VectorXd) R_11 for rotation matrix
/// @param R12 (const Eigen::VectorXd) R_12 for rotation matrix
/// @param R13 (const Eigen::VectorXd) R_13 for rotation matrix
/// @param R21 (const Eigen::VectorXd) R_21 for rotation matrix
/// @param R22 (const Eigen::VectorXd) R_22 for rotation matrix
/// @param R23 (const Eigen::VectorXd) R_23 for rotation matrix
/// @param R31 (const Eigen::VectorXd) R_31 for rotation matrix
/// @param R32 (const Eigen::VectorXd) R_32 for rotation matrix
/// @param R33 (const Eigen::VectorXd) R_33 for rotation matrix
struct TrackPos {
    // for position
    const Eigen::VectorXd X;
    const Eigen::VectorXd Y;
    const Eigen::VectorXd Z;

    // // for orientation
    // const Eigen::VectorXd R11;
    // const Eigen::VectorXd R12;
    // const Eigen::VectorXd R13;
    // const Eigen::VectorXd R21;
    // const Eigen::VectorXd R22;
    // const Eigen::VectorXd R23;
    // const Eigen::VectorXd R31;
    // const Eigen::VectorXd R32;
    // const Eigen::VectorXd R33;
};

class Track {
public:
    Track(std::string file);

    /// @brief get Track waypoints 
    /// @param init_position (Eigen::Vector3d) initial End-Effector position 
    /// @return (TrackPos) Tracking waypoints about x, y, z-axis and rotation matrix(not yet)
    TrackPos getTrack(Eigen::Vector3d init_position = Eigen::Vector3d::Zero());

private:
    Eigen::VectorXd X;
    Eigen::VectorXd Y;
    Eigen::VectorXd Z;

    // Eigen::VectorXd R11;
    // Eigen::VectorXd R12;
    // Eigen::VectorXd R13;
    // Eigen::VectorXd R21;
    // Eigen::VectorXd R22;
    // Eigen::VectorXd R23;
    // Eigen::VectorXd R31;
    // Eigen::VectorXd R32;
    // Eigen::VectorXd R33;
};
};

#endif //MPCC_TRACK_H
