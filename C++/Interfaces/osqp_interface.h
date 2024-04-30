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

#ifndef MPCC_OSQP_INTERFACE_H
#define MPCC_OSQP_INTERFACE_H

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <sys/time.h>

#include "OsqpEigen/OsqpEigen.h"

#include "config.h"
#include "types.h"
#include "Model/model.h"
#include "Cost/cost.h"
#include "Constraints/constraints.h"
#include "Constraints/bounds.h"
#include "solver_interface.h"

#include <array>
#include <vector>

namespace mpcc{
struct OptVariables;
struct Stage;


class OsqpInterface : public SolverInterface {
public:
    // std::array<OptVariables,N+1> solveMPC(std::array<Stage,N+1> &stages,const State &x0, int *status);
std::array<OptVariables,N+1> solveMPC(std::array<Stage,N+1> &stages,const std::array<OptVariables,N+1> &initial_guess, int *status);
    ~OsqpInterface(){
        std::cout << "Deleting Osqp Interface" << std::endl;
    }
private:
    MatrixXd P_;
    MatrixXd q_;
    MatrixXd A_eq_;
    MatrixXd A_ineqb_;
    MatrixXd A_ineqp_;
    MatrixXd A_;
    MatrixXd l_eq_;
    MatrixXd l_ineqb_;
    MatrixXd l_ineqp_;
    MatrixXd l_;
    MatrixXd u_eq_;
    MatrixXd u_ineqb_;
    MatrixXd u_ineqp_;
    MatrixXd u_;

    MatrixXd initial_x_;

    OsqpEigen::Solver solver_;
    VectorXd qp_sol_;

    bool is_solved_ = false;

    void setDynamics(std::array<Stage,N+1> &stages,const State &x0);
    void setCost(std::array<Stage,N+1> &stages);
    void setBounds(std::array<Stage,N+1> &stages,const State &x0);
    void setPolytopicConstraints(std::array<Stage,N+1> &stages);
    void setInitialGuess(const std::array<OptVariables,N+1> &initial_guess);


    bool Solve(bool verbose);
    std::array<OptVariables,N+1> getSolution();

    void print_data();
};
}
#endif //MPCC_HPIPM_INTERFACE_H