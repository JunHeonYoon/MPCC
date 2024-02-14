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

#ifndef MPCC_MODEL_H
#define MPCC_MODEL_H

#include "config.h"
#include "types.h"
#include "Params/params.h"
namespace mpcc{
//Return

/// @brief Linear model of car: x_(k+1) = A * x_k + B * u_k + g
/// @param A (Eigen::MatrixXd) Linear part of state
/// @param B (Eigen::MatrixXd) Linear part of control input
/// @param g (Eigen::MatrixXd) Constant part
struct LinModelMatrix {
    A_MPC A;
    B_MPC B;
    g_MPC g;
};

/// @brief Tire Forces
/// @param F_x (const double) Tire force along x-axis
/// @param F_y (const double) Tire force along y-axis
struct TireForces {
    const double F_y;
    const double F_x;
};

/// @brief Normal Forces
/// @param F_N_front (const double) Normal force acting front wheel
/// @param F_N_rear (const double) Normal force acting rear wheel
struct NormalForces {
    const double F_N_front;
    const double F_N_rear;
};

/// @brief Derivative of Tire Forces
/// @param dF_x_vx (const double) derivative of F_x wrt vx
/// @param dF_x_vy (const double) derivative of F_x wrt vy
/// @param dF_x_r (const double) derivative of F_x wrt r
/// @param dF_x_D (const double) derivative of F_x wrt D
/// @param dF_x_delta (const double) derivative of F_x wrt delta
/// @param dF_y_vx (const double) derivative of F_y wrt vx
/// @param dF_y_vy (const double) derivative of F_y wrt vy
/// @param dF_y_r (const double) derivative of F_y wrt r
/// @param dF_y_D (const double) derivative of F_y wrt D
/// @param dF_y_delta (const double) derivative of F_y wrt delta
struct TireForcesDerivatives{
    const double dF_y_vx;
    const double dF_y_vy;
    const double dF_y_r;
    const double dF_y_D;
    const double dF_y_delta;

    const double dF_x_vx;
    const double dF_x_vy;
    const double dF_x_r;
    const double dF_x_D;
    const double dF_x_delta;
};


/// @brief Derivative of Friction Force
/// @param dF_f_vx (const double) derivative of F_f wrt vx
/// @param dF_f_vy (const double) derivative of F_f wrt vy
/// @param dF_f_r (const double) derivative of F_f wrt r
/// @param dF_f_D (const double) derivative of F_f wrt D
/// @param dF_f_delta (const double) derivative of F_f wrt delta
struct FrictionForceDerivatives {
    const double dF_f_vx;
    const double dF_f_vy;
    const double dF_f_r;
    const double dF_f_D;
    const double dF_f_delta;
};

class Model {
public:
    Model();
    Model(double Ts,const PathToJson &path);

    /// @brief  compute slip angles given current state
    /// @param x (State) current state
    /// @return (double) Slip angle of front wheel
    double getSlipAngleFront(const State &x) const;

    /// @brief  compute slip angles given current state
    /// @param x (State) current state
    /// @return (double) Slip angle of rear wheel
    double getSlipAngleRear(const State &x) const;

    /// @brief compute Front Tire Force given current state
    /// @param x (State) current state
    /// @return (TireForces) Front Tire Force along x, y-axis
    TireForces getForceFront(const State &x) const;

    /// @brief compute Rear Tire Force given current state
    /// @param x (State) current state
    /// @return (TireForces) Rear Tire Force along x, y-axis
    TireForces getForceRear(const State &x) const;

    /// @brief compute Friction Force given current state
    /// @param x (State) current state
    /// @return (double) Friction Force
    double getForceFriction(const State &x) const;

    /// @brief compute Normal Force given current state
    /// @param x (State) current state
    /// @return (NormalForces) Normal Force of front and rear wheels
    NormalForces getForceNormal(const State &x) const;


    /// @brief compute Derivative of Front Tire Force given current state
    /// @param x (State) current state
    /// @return (TireForcesDerivatives) Derivative of Front Tire Force along x, y-axis wrt vx, xy, r, D, delta 
    TireForcesDerivatives getForceFrontDerivatives(const State &x) const;

    /// @brief compute Derivative of Rear Tire Force given current state
    /// @param x (State) current state
    /// @return (TireForcesDerivatives) Derivative of Rear Tire Force along x, y-axis wrt vx, xy, r, D, delta 
    TireForcesDerivatives getForceRearDerivatives(const State &x) const;

    /// @brief compute Derivative of Friction Force given current state
    /// @param x (State) current state
    /// @return (FrictionForceDerivatives) Derivative of Friction Force wrt vx, xy, r, D, delta 
    FrictionForceDerivatives getForceFrictionDerivatives(const State &x) const;

    /// @brief compute derivative of State wrt time (x_dot) given current state and control input
    /// @param x (State) current state
    /// @param u (Input) current control input
    /// @return (Eigen::VectorXd) derivative of State
    StateVector getF(const State &x,const Input &u) const;

    /// @brief compute Discretized Linear model given current state and control input
    /// @param x (State) Current state
    /// @param u (Input) Current control input
    /// @return (LinModelMatrix) Discrete Linear model (A, B, g)
    LinModelMatrix getLinModel(const State &x, const Input &u) const;

private:
    /// @brief compute Linear Model given current state and control input
    /// @param x (State) current state
    /// @param u (Input) current control input
    /// @return (LinModelMatrix) Linear model (A, B, g)
    LinModelMatrix getModelJacobian(const State &x, const Input &u) const;

    /// @brief compute Discretized Linear model from continuous Linear model
    /// @param lin_model_c (LinModelMatrix) Continuous  Linear model
    /// @return (LinModelMatrix) Discrete Linear model (A, B, g)
    LinModelMatrix discretizeModel(const LinModelMatrix &lin_model_c) const;

    Param param_;
    const double Ts_;
};
}
#endif //MPCC_MODEL_H
