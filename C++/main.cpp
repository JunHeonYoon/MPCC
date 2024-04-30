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
#include "Plotting/plotting.h"

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
                           jsonConfig["normalization_path"]};

    Integrator integrator = Integrator(jsonConfig["Ts"],json_paths);
    // Plotting plotter = Plotting(jsonConfig["Ts"],json_paths);

    std::shared_ptr<RobotModel> robot;
    robot = std::make_shared<mpcc::RobotModel>();
    std::shared_ptr<SelCollNNmodel> selcolNN;
    selcolNN = std::make_shared<SelCollNNmodel>();
    Eigen::Vector2d n_hidden;
    n_hidden << 128, 128;
    selcolNN->setNeuralNetwork(PANDA_DOF, 1, n_hidden, true);

    std::list<MPCReturn> log;
    MPC mpc(jsonConfig["max_n_sqp"],jsonConfig["n_reset"],jsonConfig["max_n_sqp_linesearch"],jsonConfig["Ts"],json_paths,robot,selcolNN);

    State x0 = {0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4, 0, 0};
    JointVector q0 = stateToJointVector(x0);
    Eigen::Vector3d ee_pos0 = robot->getEEPosition(q0);

    Track track = Track(json_paths.track_path);
    TrackPos track_xyz = track.getTrack(ee_pos0);

    mpc.setTrack(track_xyz.X,track_xyz.Y,track_xyz.Z);

    ofstream debug_file;
    debug_file.open("debug.txt");

    std::cout << "============================ Init ============================"<<std::endl;
    std::cout << "q now    :\t";
    std::cout << std::fixed << std::setprecision(3) << q0.transpose() << std::endl;
    std::cout << "x        :\t";
    std::cout << std::fixed << std::setprecision(3) << robot->getEEPosition(q0).transpose() << std::endl;
    std::cout << "R        :\t" << std::endl;
    std::cout << std::fixed << std::setprecision(3) << robot->getEEOrientation(q0) << std::endl;
    std::cout << "J        :\t" << std::endl;
    std::cout << std::fixed << std::setprecision(3) << robot->getJacobianv(q0) << std::endl;
    std::cout << "==============================================================="<<std::endl;
    debug_file << q0.transpose() << std::endl;

    // for(int i=0;i<jsonConfig["n_sim"];i++)
    for(int i=0;i<jsonConfig["n_sim"];i++)
    {
        std::cout<<"sim step: "<< i << std::endl;
        MPCReturn mpc_sol = mpc.runMPC(x0);
        x0 = integrator.simTimeStep(x0,mpc_sol.u0,jsonConfig["Ts"]);

        std::cout << "==============================================================="<<std::endl;
        std::cout << "time step: " << i << std::endl;;
        std::cout << "q now    :\t";
        std::cout << std::fixed << std::setprecision(3) << stateToJointVector(x0).transpose() << std::endl;
        std::cout << "q_dot now    :\t";
        std::cout << std::fixed << std::setprecision(3) << inputToJointVector(mpc_sol.u0).transpose()  << std::endl;
        std::cout << "x        :\t";
        std::cout << std::fixed << std::setprecision(3) << robot->getEEPosition(stateToJointVector(x0)).transpose() << std::endl;
        std::cout << "x_dot    :\t";
        std::cout << std::fixed << std::setprecision(3) << (robot->getJacobianv(stateToJointVector(x0))*inputToJointVector(mpc_sol.u0)).transpose() << std::endl;
        std::cout << std::fixed << std::setprecision(3) << (robot->getJacobianv(stateToJointVector(x0))*inputToJointVector(mpc_sol.u0)).norm() << std::endl;
        // std::cout << "R        :\t" << std::endl;
        // std::cout << std::fixed << std::setprecision(3) << robot->getEEOrientation(stateToJointVector(x0)) << std::endl;
        // std::cout << "J        :\t" << std::endl;
        // std::cout << std::fixed << std::setprecision(3) << robot->getJacobianv(stateToJointVector(x0)) << std::endl;
        std::cout << "==============================================================="<<std::endl;
        debug_file << stateToJointVector(x0).transpose() << std::endl;
        log.push_back(mpc_sol);
    }
    // plotter.plotRun(log,track_xyz);
    // plotter.plotSim(log,track_xyz);

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


