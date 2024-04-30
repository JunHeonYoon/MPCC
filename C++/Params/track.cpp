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

#include "track.h"
namespace mpcc{
Track::Track(std::string file) 
{
    /////////////////////////////////////////////////////
    // Loading Model and Constraint Parameters //////////
    /////////////////////////////////////////////////////
    std::ifstream iTrack(file);
    json jsonTrack;
    iTrack >> jsonTrack;
    // Model Parameters
    std::vector<double> x = jsonTrack["X"];
    X = Eigen::Map<Eigen::VectorXd>(x.data(), x.size());
    std::vector<double> y = jsonTrack["Y"];
    Y = Eigen::Map<Eigen::VectorXd>(y.data(), y.size());
    std::vector<double> z = jsonTrack["Z"];
    Z = Eigen::Map<Eigen::VectorXd>(z.data(), z.size());
    // std::vector<double> r11 = jsonTrack["R11"];
    // R11 = Eigen::Map<Eigen::VectorXd>(r11.data(), r11.size());
    // std::vector<double> r12 = jsonTrack["R12"];
    // R12 = Eigen::Map<Eigen::VectorXd>(r12.data(), r12.size());
    // std::vector<double> r13 = jsonTrack["R13"];
    // R13 = Eigen::Map<Eigen::VectorXd>(r13.data(), r13.size());
    // std::vector<double> r21 = jsonTrack["R21"];
    // R21 = Eigen::Map<Eigen::VectorXd>(r21.data(), r21.size());
    // std::vector<double> r22 = jsonTrack["R22"];
    // R22 = Eigen::Map<Eigen::VectorXd>(r22.data(), r22.size());
    // std::vector<double> r23 = jsonTrack["R23"];
    // R23 = Eigen::Map<Eigen::VectorXd>(r23.data(), r23.size());
    // std::vector<double> r31 = jsonTrack["R31"];
    // R31 = Eigen::Map<Eigen::VectorXd>(r31.data(), r31.size());
    // std::vector<double> r32 = jsonTrack["R32"];
    // R32 = Eigen::Map<Eigen::VectorXd>(r32.data(), r32.size());
    // std::vector<double> r33 = jsonTrack["R33"];
    // R33 = Eigen::Map<Eigen::VectorXd>(r33.data(), r33.size());
}

TrackPos Track::getTrack(Eigen::Vector3d init_position)
{
    X = X.array() - X(0) + init_position(0);
    Y = Y.array() - Y(0) + init_position(1);
    Z = Z.array() - Z(0) + init_position(2);
    return {Eigen::Map<Eigen::VectorXd>(X.data(), X.size()), 
            Eigen::Map<Eigen::VectorXd>(Y.data(), Y.size()),
            Eigen::Map<Eigen::VectorXd>(Z.data(), Z.size()),
            // Eigen::Map<Eigen::VectorXd>(R11.data(), R11.size()),
            // Eigen::Map<Eigen::VectorXd>(R12.data(), R12.size()),
            // Eigen::Map<Eigen::VectorXd>(R13.data(), R13.size()),
            // Eigen::Map<Eigen::VectorXd>(R21.data(), R21.size()),
            // Eigen::Map<Eigen::VectorXd>(R22.data(), R22.size()),
            // Eigen::Map<Eigen::VectorXd>(R23.data(), R23.size()),
            // Eigen::Map<Eigen::VectorXd>(R31.data(), R31.size()),
            // Eigen::Map<Eigen::VectorXd>(R32.data(), R32.size()),
            // Eigen::Map<Eigen::VectorXd>(R33.data(), R33.size())
            };
}
}
