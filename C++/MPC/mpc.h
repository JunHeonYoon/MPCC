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

#ifndef MPCC_MPC_H
#define MPCC_MPC_H

#include "config.h"
#include "types.h"
#include "Params/params.h"
#include "Spline/arc_length_spline.h"
#include "Model/model.h"
#include "Model/integrator.h"
#include "Cost/cost.h"
#include "Constraints/constraints.h"
#include "Constraints/bounds.h"

#include "Interfaces/solver_interface.h"
#include "Interfaces/hpipm_interface.h"
#include "Interfaces/osqp_interface.h"

#include <array>
#include <memory>
#include <ctime>
#include <ratio>
#include <chrono>

namespace mpcc{

/// @brief parameters for optimization, state and control input of MPC
/// @param xk  (State) state
/// @param uk  (Input) control input
/// @param slk (Slack) lower slack variable 
/// @param suk (Slack) upper slack variable 
struct OptVariables {
    State xk;
    Input uk;
    Slack suk;
    Slack slk;
};

/// @brief dynamic model, cost, constraint matrix in a stage for MPC 
/// @param lin_model (LinModelMatrix) linear matrix of robot model
/// @param cost_mat (CostMatrix) cost matrix 
/// @param constrains_mat (ConstrainsMatrix) constraint matrix
/// @param u_bounds_x (Eigen::VectorXd) upper bound of state
/// @param l_bounds_x (Eigen::VectorXd) lower bound of state
/// @param u_bounds_u (Eigen::VectorXd) upper bound of control input
/// @param l_bounds_u (Eigen::VectorXd) lower bound of control input
/// @param u_bounds_s (Eigen::VectorXd) upper bound of slack variables
/// @param l_bounds_s (Eigen::VectorXd) lower bound of slack variables
/// @param nx,nu,nbx,nbu,ng,ns (int) number of each parameters
struct Stage {
    LinModelMatrix lin_model;
    CostMatrix cost_mat;
    ConstrainsMatrix constrains_mat;

    Bounds_x u_bounds_x;
    Bounds_x l_bounds_x;

    Bounds_u u_bounds_u;
    Bounds_u l_bounds_u;

    Bounds_s u_bounds_s;
    Bounds_s l_bounds_s;

    //nx    -> number of states
    //nu    -> number of inputs
    //nbx   -> number of bounds on x
    //nbu   -> number of bounds on u
    //ng    -> number of polytopic constratins
    //ns    -> number of soft constraints
    int nx, nu, nbx, nbu, ng, ns;
};

/// @brief output of MPC
/// @param u0 (Input) optimal control input
/// @param mpc_horizon (std::array<OptVariables,N+1>) total horizon results (state and control input)
/// @param time_total (double) time to run MPC
struct MPCReturn {
    const Input u0;
    const std::array<OptVariables,N+1> mpc_horizon;
    const double time_total;
};

class MPC {
public:
    MPC();
    MPC(int n_sqp, int n_reset, double sqp_mixing, double Ts,const PathToJson &path,std::shared_ptr<RobotModel> robot, std::shared_ptr<SelCollNNmodel> selcolNN);

    /// @brief run MPC by sqp given current state
    /// @param x0 (State) current state
    /// @return (MPCReturn) log for MPC: optimal control input, total horizon results, time to run MPC
    MPCReturn runMPC(State &x0);

    /// @brief set track given X-Y path data
    /// @param X (Eigen::VectorXd) X path data
    /// @param Y (Eigen::VectorXd) Y path data
    /// @param Z (Eigen::VectorXd) Z path data
    void setTrack(const Eigen::VectorXd &X, const Eigen::VectorXd &Y,const Eigen::VectorXd &Z);

private:
    /// @brief MPC whole stage data (model, cost constraint) by initial variables
    void setMPCProblem();

    /// @brief set MPC single stage data (model, cost constraint) given state and control input
    /// @param xk (State) state
    /// @param uk (Input) control input
    /// @param time_step (int) time_step
    void setStage(const State &xk, const Input &uk, int time_step);

    /// @brief normalizing cost by normalize parameters
    /// @param cost_mat (CostMatrix) cost matrix before normalization
    /// @return (CostMatrix) normalized cost matrix
    CostMatrix normalizeCost(const CostMatrix &cost_mat);

    /// @brief normalizing dynamic model by normalize parameters
    /// @param cost_mat (LinModelMatrix) dynamic model matrix before normalization
    /// @return (LinModelMatrix) normalized dynamic model matrix
    LinModelMatrix normalizeDynamics(const LinModelMatrix &lin_model);

    /// @brief normalizing constraints by normalize parameters
    /// @param cost_mat (ConstrainsMatrix) dconstraints matrix before normalization
    /// @return (ConstrainsMatrix) normalized constraints matrix
    ConstrainsMatrix normalizeCon(const ConstrainsMatrix &con_mat);

    /// @brief denormalize MPC solution 
    /// @param solution (std::array<OptVariables,N+1>) raw MPC solution
    /// @return std::array<OptVariables,N+1> denormalized MPC solution
    std::array<OptVariables,N+1> deNormalizeSolution(const std::array<OptVariables,N+1> &solution);

    /// @brief normalize MPC solution 
    /// @param solution (std::array<OptVariables,N+1>) raw MPC solution
    /// @return std::array<OptVariables,N+1> normalized MPC solution
    std::array<OptVariables,N+1> normalizeSolution(const std::array<OptVariables,N+1> &solution);

    /// @brief to be warmstart, update initial variables for MPC
    /// @param x0 (State) solution of MPC before time step
    void updateInitialGuess(const State &x0);

    /// @brief generate new initial variables for MPC for the first
    /// @param x0 (State) current state
    void generateNewInitialGuess(const State &x0);

    /// @brief unwrapping for initial variables which have phi(yaw) and arc length(s) 
    void unwrapInitialGuess();

    /// @brief mixing current and last solution of MPC
    /// @param last_solution (std::array<OptVariables, N + 1>) last solution
    /// @param current_solution (std::array<OptVariables, N + 1>) current solution
    /// @return (std::array<OptVariables, N + 1>) mixed solution
    std::array<OptVariables, N + 1> sqpSolutionUpdate(const std::array<OptVariables, N + 1> &last_solution,
                                                      const std::array<OptVariables, N + 1> &current_solution);

    double constraint_norm(const VectorXd &constr, const VectorXd &l, const VectorXd &u);

    bool valid_initial_guess_;

    std::array<Stage, N + 1> stages_;
    // std::array<Stage, N + 1> stages_denor_;

    std::array<OptVariables, N + 1> initial_guess_;
    std::array<OptVariables, N + 1> initial_guess_nor_;
    std::array<OptVariables, N + 1> optimal_solution_;

    int max_n_sqp_; // maximum number of iteration for sqp
    double max_n_sqp_linesearch_; // maximum number of iteration for line search in sqp
    int n_non_solves_;
    int n_no_solves_sqp_;
    int n_reset_; // threshhold for reset the initial guess

    const double Ts_;

    Model model_;
    Integrator integrator_;
    Cost cost_;
    Constraints constraints_;
    ArcLengthSpline track_;

    Bounds bounds_;
    NormalizationParam normalization_param_;
    Param param_;

    std::shared_ptr<RobotModel> robot_;
    std::shared_ptr<SelCollNNmodel> selcolNN_;

    std::unique_ptr<SolverInterface> solver_interface_;
};

}

#endif //MPCC_MPC_H
