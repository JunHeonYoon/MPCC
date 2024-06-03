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

// #include "Tests/spline_test.h"
// #include "Tests/model_integrator_test.h"
// #include "Tests/constratins_test.h"
// #include "Tests/cost_test.h"
#include <iostream>
#include <fstream>
#include "MPC/mpc.h"
#include "Model/integrator.h"
#include "Params/track.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

int main() {

    using namespace mpcc;
    std::ifstream iConfig("Params/config.json");
    json jsonConfig;
    iConfig >> jsonConfig;

    PathToJson json_paths {jsonConfig["model_path"],
                           jsonConfig["cost_path"],
                           jsonConfig["bounds_path"],
                           jsonConfig["track_path"],
                           jsonConfig["normalization_path"],
                           jsonConfig["sqp_path"]};

    Integrator integrator = Integrator(jsonConfig["Ts"],json_paths);

    std::shared_ptr<RobotModel> robot;
    robot = std::make_shared<mpcc::RobotModel>();

    std::shared_ptr<SelCollNNmodel> selcolNN;
    selcolNN = std::make_shared<SelCollNNmodel>();
    Eigen::Vector2d n_hidden;
    n_hidden << 128, 64;
    selcolNN->setNeuralNetwork(PANDA_DOF, 1, n_hidden, true);

    std::list<MPCReturn> log;
    MPC mpc(jsonConfig["Ts"],json_paths,robot,selcolNN);

    State x0 = {0., 0., 0., -M_PI/2, 0, M_PI/2, M_PI/4, 0.0, 0.0};
    Eigen::Vector3d ee_pos = robot->getEEPosition(stateToJointVector(x0));
    Eigen::Matrix3d ee_ori = robot->getEEOrientation(stateToJointVector(x0));

    Track track = Track(json_paths.track_path);
    TrackPos track_xyzr = track.getTrack(ee_pos);

    mpc.setTrack(track_xyzr.X,track_xyzr.Y,track_xyzr.Z,track_xyzr.R);

    Eigen::Vector3d end_point;
    Eigen::Matrix3d end_ori;
    end_point(0) = track_xyzr.X(track_xyzr.X.size() - 1); 
    end_point(1) = track_xyzr.Y(track_xyzr.Y.size() - 1);
    end_point(2) = track_xyzr.Z(track_xyzr.Z.size() - 1);
    end_ori = track_xyzr.R[track_xyzr.R.size() - 1];
    std::cout<<"end posi: "<<end_point.transpose()<<std::endl;

    ofstream debug_file;
    debug_file.open("debug.txt");

    std::cout << "============================ Init ============================"<<std::endl;
    std::cout << "q now           :\t";
    std::cout << std::fixed << std::setprecision(6) << stateToJointVector(x0).transpose() << std::endl;
    std::cout << "x               :\t";
    std::cout << std::fixed << std::setprecision(6) << ee_pos.transpose() << std::endl;
    std::cout << "R               :" << std::endl;
    std::cout << std::fixed << std::setprecision(6) << ee_ori << std::endl;
    std::cout << "manipulability  :\t";
    std::cout << std::fixed << std::setprecision(6) << robot->getManipulability(stateToJointVector(x0)) << std::endl;
    std::cout << "min distance[cm]:\t";
    std::cout << std::fixed << std::setprecision(6) << (selcolNN->calculateMlpOutput(stateToJointVector(x0),false)).first << std::endl;
    std::cout << "s               :";
    std::cout << std::fixed << std::setprecision(6) << x0.s << std::endl;
    std::cout << "vs              :";
    std::cout << std::fixed << std::setprecision(6) << x0.vs << std::endl;
    std::cout << "x_error         :";
    std::cout << std::fixed << std::setprecision(6) << (end_point - ee_pos).transpose() << std::endl;
    std::cout << "R error         :";
    std::cout << std::fixed << std::setprecision(6) << getInverseSkewVector(LogMatrix(end_ori.transpose()*ee_ori)).transpose() << std::endl;
    std::cout << "==============================================================="<<std::endl;
    debug_file << stateToJointVector(x0).transpose() << std::endl;

    for(int i=0;i<jsonConfig["n_sim"];i++)
    {
        MPCReturn mpc_sol = mpc.runMPC(x0);
        x0 = integrator.simTimeStep(x0,mpc_sol.u0,jsonConfig["Ts"]);
        ee_pos = robot->getEEPosition(stateToJointVector(x0));
        ee_ori = robot->getEEOrientation(stateToJointVector(x0));

        std::cout << "==============================================================="<<std::endl;
        std::cout << "time step: " << i << std::endl;;
        std::cout << "q now           :\t";
        std::cout << std::fixed << std::setprecision(6) << stateToJointVector(x0).transpose() << std::endl;
        std::cout << "q_dot now       :\t";
        std::cout << std::fixed << std::setprecision(6) << inputToJointVector(mpc_sol.u0).transpose()  << std::endl;
        std::cout << "x               :\t";
        std::cout << std::fixed << std::setprecision(6) << ee_pos.transpose() << std::endl;
        std::cout << "x_dot           :\t";
        std::cout << std::fixed << std::setprecision(6) << (robot->getJacobianv(stateToJointVector(x0))*inputToJointVector(mpc_sol.u0)).transpose() << std::endl;
        std::cout << std::fixed << std::setprecision(6) << (robot->getJacobianv(stateToJointVector(x0))*inputToJointVector(mpc_sol.u0)).norm() << std::endl;
        std::cout << "R               :" << std::endl;
        std::cout << std::fixed << std::setprecision(6) << ee_ori << std::endl;
        std::cout << "manipulability  :\t";
        std::cout << std::fixed << std::setprecision(6) << robot->getManipulability(stateToJointVector(x0)) << std::endl;
        std::cout << "min distance[cm]:\t";
        std::cout << std::fixed << std::setprecision(6) << (selcolNN->calculateMlpOutput(stateToJointVector(x0),false)).first << std::endl;
        std::cout << "s               :";
        std::cout << std::fixed << std::setprecision(6) << x0.s << std::endl;
        std::cout << "vs              :";
        std::cout << std::fixed << std::setprecision(6) << x0.vs << std::endl;
        std::cout << "dVs              :";
        std::cout << std::fixed << std::setprecision(6) << mpc_sol.u0.dVs << std::endl;
        std::cout << "x_error         :";
        std::cout << std::fixed << std::setprecision(6) << (end_point - ee_pos).transpose() << std::endl;
        std::cout << "R error         :";
        std::cout << std::fixed << std::setprecision(6) << getInverseSkewVector(LogMatrix(end_ori.transpose()*ee_ori)).transpose() << std::endl;
        std::cout << "==============================================================="<<std::endl;
        
        debug_file << stateToJointVector(x0).transpose() << std::endl;
        log.push_back(mpc_sol);

        if((end_point - ee_pos).norm() < 1e-2 && (getInverseSkewVector(LogMatrix(end_ori.transpose()*ee_ori))).norm() < 1e-3 && x0.s > 0.1)
        {
            std::cout << "End point reached!!!"<< std::endl;
            break;
        }
    }

    double mean_time = 0.0;
    double max_time = 0.0;
    for(MPCReturn log_i : log)
    {
        mean_time += log_i.time_total;
        if(log_i.time_total > max_time)
            max_time = log_i.time_total;
    }
    std::cout << "mean nmpc time " << mean_time/double(jsonConfig["n_sim"]) << std::endl;
    std::cout << "max nmpc time " << max_time << std::endl;
    return 0;
}


