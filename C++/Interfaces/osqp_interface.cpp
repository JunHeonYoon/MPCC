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

#include "osqp_interface.h"
#include "MPC/mpc.h"
namespace mpcc{
void OsqpInterface::setDynamics(std::array<Stage,N+1> &stages,const State &x0)
{
    A_eq_.resize((N+1)*NX,(N+1)*(NX+NU)); A_eq_.setZero();
    l_eq_.resize((N+1)*NX,1);             l_eq_.setOnes(); l_eq_ *= -OsqpEigen::INFTY; 
    u_eq_.resize((N+1)*NX,1);             u_eq_.setOnes(); u_eq_ *= OsqpEigen::INFTY;
    
      
    for(size_t i=0;i<N+1;i++)
    {
        if(i==0)
        {
            A_eq_.block(0,0,NX,NX) = MatrixXd::Identity(NX,NX);
            l_eq_.block(0,0,NX,1) = stateToVector(x0);
            u_eq_.block(0,0,NX,1) = stateToVector(x0);
        }
        else
        {
            A_eq_.block(NX*i, NX*(i-1), NX, NX) = -stages[i].lin_model.A;
            A_eq_.block(NX*i, NX*i, NX, NX) = MatrixXd::Identity(NX,NX);
            A_eq_.block(NX*i, NX*(N+1)+NU*(i-1), NX, NU) = -stages[i].lin_model.B;
            l_eq_.block(NX*i, 0, NX, 1) = stages[i].lin_model.g;
            u_eq_.block(NX*i, 0, NX, 1) = stages[i].lin_model.g;
        }
    }
}

void OsqpInterface::setCost(std::array<Stage,N+1> &stages)
{
    // P_.resize((N+1)*(NX+NU+2*NS),(N+1)*(NX+NU+2*NS)); P_.setZero();
    // q_.resize((N+1)*(NX+NU+2*NS),1);                  q_.setZero();

    // for(size_t i=0;i<N+1;i++)
    // {
    //     P_.block(NX*i, NX*i, NX, NX) = stages[i].cost_mat.Q;
    //     P_.block(NX*(N+1)+NU*i, NX*(N+1)+NU*i, NU, NU) = stages[i].cost_mat.R;
    //     q_.block(NX*i, 0, NX, 1) = stages[i].cost_mat.q;
    //     q_.block(NX*(N+1)+NU*i, 0, NU, 1) = stages[i].cost_mat.r;

    //     if(stages[i].ns != 0)
    //     {
    //         P_.block((NX+NU)*(N+1)+NS*i, (NX+NU)*(N+1)+NS*i, NS, NS) = stages[i].cost_mat.Z;
    //         P_.block((NX+NU+NS)*(N+1)+NS*i, (NX+NU+NS)*(N+1)+NS*i, NS, NS) = stages[i].cost_mat.Z;
    //         q_.block((NX+NU)*(N+1)+NS*i, 0, NS, 1) = stages[i].cost_mat.z;
    //         q_.block((NX+NU+NS)*(N+1)+NS*i, 0, NS, 1) = stages[i].cost_mat.z;
    //     }
    // }
    P_.resize((N+1)*(NX+NU),(N+1)*(NX+NU)); P_.setZero();
    q_.resize((N+1)*(NX+NU),1);                  q_.setZero();

    for(size_t i=0;i<N+1;i++)
    {
        P_.block(NX*i, NX*i, NX, NX) = stages[i].cost_mat.Q;
        P_.block(NX*(N+1)+NU*i, NX*(N+1)+NU*i, NU, NU) = stages[i].cost_mat.R;
        P_.block(NX*i, NX*(N+1)+NU*i, NX, NU) = stages[i].cost_mat.S;
        P_.block(NX*(N+1)+NU*i, NX*i, NU, NX) = stages[i].cost_mat.S.transpose();
        q_.block(NX*i, 0, NX, 1) = stages[i].cost_mat.q;
        q_.block(NX*(N+1)+NU*i, 0, NU, 1) = stages[i].cost_mat.r;
    }
}

void OsqpInterface::setBounds(std::array<Stage,N+1> &stages,const State &x0)
{
    // A_ineqb_.resize((N+1)*(NX+NU+2*NS),(N+1)*(NX+NU+2*NS)); A_ineqb_.setZero();
    // l_ineqb_.resize((N+1)*(NX+NU+2*NS),1);                  l_ineqb_.setOnes(); l_ineqb_ *= -OsqpEigen::INFTY; 
    // u_ineqb_.resize((N+1)*(NX+NU+2*NS),1);                  u_ineqb_.setOnes(); u_ineqb_ *= OsqpEigen::INFTY;

    // for(size_t i=0;i<N+1;i++)
    // {
    //     A_ineqb_.block(NX*i, NX*i, NX, NX) = MatrixXd::Identity(NX, NX);
    //     A_ineqb_.block(NX*(N+1)+NU*i, NX*(N+1)+NU*i, NU, NU) = MatrixXd::Identity(NU, NU);
    //     l_ineqb_.block(NX*i, 0, NX, 1) = stages[i].l_bounds_x;
    //     u_ineqb_.block(NX*i, 0, NX, 1) = stages[i].u_bounds_x;
    //     l_ineqb_.block(NX*(N+1)+NU*i, 0, NU, 1) = stages[i].l_bounds_u;
    //     u_ineqb_.block(NX*(N+1)+NU*i, 0, NU, 1) = stages[i].u_bounds_u;

    //     if(stages[i].ns != 0)
    //     {
    //         A_ineqb_.block((NX+NU)*(N+1)+NS*i, (NX+NU)*(N+1)+NS*i, NS, NS) = MatrixXd::Identity(NS, NS);
    //         A_ineqb_.block((NX+NU+NS)*(N+1)+NS*i, (NX+NU+NS)*(N+1)+NS*i, NS, NS) = MatrixXd::Identity(NS, NS);
    //         l_ineqb_.block((NX+NU)*(N+1)+NS*i, 0, NS, 1) = stages[i].l_bounds_s;
    //         l_ineqb_.block((NX+NU+NS)*(N+1)+NS*i, 0, NS, 1) = stages[i].u_bounds_s;
    //     }
    // }

    A_ineqb_.resize((N+1)*(NX+NU),(N+1)*(NX+NU)); A_ineqb_.setZero();
    l_ineqb_.resize((N+1)*(NX+NU),1);                  l_ineqb_.setOnes(); l_ineqb_ *= -OsqpEigen::INFTY; 
    u_ineqb_.resize((N+1)*(NX+NU),1);                  u_ineqb_.setOnes(); u_ineqb_ *= OsqpEigen::INFTY;

    for(size_t i=0;i<N+1;i++)
    {
        A_ineqb_.block(NX*i, NX*i, NX, NX) = MatrixXd::Identity(NX, NX);
        A_ineqb_.block(NX*(N+1)+NU*i, NX*(N+1)+NU*i, NU, NU) = MatrixXd::Identity(NU, NU);
        l_ineqb_.block(NX*i, 0, NX, 1) = stages[i].l_bounds_x;
        u_ineqb_.block(NX*i, 0, NX, 1) = stages[i].u_bounds_x;
        l_ineqb_.block(NX*(N+1)+NU*i, 0, NU, 1) = stages[i].l_bounds_u;
        u_ineqb_.block(NX*(N+1)+NU*i, 0, NU, 1) = stages[i].u_bounds_u;

    }
}

void OsqpInterface::setPolytopicConstraints(std::array<Stage,N+1> &stages)
{
    // A_ineqp_.resize((N+1)*(2*NPC),(N+1)*(NX+NU+2*NS)); A_ineqp_.setZero();
    // l_ineqp_.resize((N+1)*(2*NPC),1);                  l_ineqp_.setOnes(); l_ineqp_ *= -OsqpEigen::INFTY; 
    // u_ineqp_.resize((N+1)*(2*NPC),1);                  u_ineqp_.setOnes(); u_ineqp_ *= OsqpEigen::INFTY;
    // for(size_t i;i<N+1;i++)
    // {
    //     if(stages[i].ng > 0)
    //     {
    //         A_ineqp_.block(NPC*i, NX*i, NPC, NX) = stages[i].constrains_mat.C;
    //         A_ineqp_.block(NPC*i, NX*(N+1)+NU*i, NPC, NU) = stages[i].constrains_mat.D;
    //         A_ineqp_.block(NPC*i, (NX+NU)*(N+1)+NS*i, NPC, NS) = MatrixXd::Identity(NPC, NS);
    //         l_ineqp_.block(NPC*i, 0, NPC, 1) = stages[i].constrains_mat.dl;

    //         A_ineqp_.block(NPC*(N+1)+NPC*i, NX*i, NPC, NX) = stages[i].constrains_mat.C;
    //         A_ineqp_.block(NPC*(N+1)+NPC*i, NX*(N+1)+NU*i, NPC, NU) = stages[i].constrains_mat.D;
    //         A_ineqp_.block(NPC*(N+1)+NPC*i, (NX+NU)*(N+1)+NS*i, NPC, NS) = -MatrixXd::Identity(NPC, NS);
    //         u_ineqp_.block(NPC*(N+1)+NPC*i, 0, NPC, 1) = stages[i].constrains_mat.du;
    //     }
    // }
    A_ineqp_.resize((N+1)*NPC,(N+1)*(NX+NU)); A_ineqp_.setZero();
    l_ineqp_.resize((N+1)*NPC,1);                  l_ineqp_.setOnes(); l_ineqp_ *= -OsqpEigen::INFTY; 
    u_ineqp_.resize((N+1)*NPC,1);                  u_ineqp_.setOnes(); u_ineqp_ *= OsqpEigen::INFTY;
    for(size_t i;i<N+1;i++)
    {
        if(stages[i].ng > 0)
        {
            A_ineqp_.block(NPC*i, NX*i, NPC, NX) = stages[i].constrains_mat.C;
            A_ineqp_.block(NPC*i, NX*(N+1)+NU*i, NPC, NU) = stages[i].constrains_mat.D;
            l_ineqp_.block(NPC*i, 0, NPC, 1) = stages[i].constrains_mat.dl;
            u_ineqp_.block(NPC*i, 0, NPC, 1) = stages[i].constrains_mat.du;
        }
    }
}

void OsqpInterface::setInitialGuess(const std::array<OptVariables,N+1> &initial_guess)
{
    initial_x_.resize((N+1)*(NX+NU),1); initial_x_.setZero();
    for(size_t i=0;i<N+1;i++)
    {
        initial_x_.block(NX*i,0,NX,1) = stateToVector(initial_guess[i].xk);
        initial_x_.block(NX*(N+1)+NU*i,0,NU,1) = inputToVector(initial_guess[i].uk);
    }
}



std::array<OptVariables,N+1> OsqpInterface::solveMPC(std::array<Stage,N+1> &stages,const std::array<OptVariables,N+1> &initial_guess, int *status)
{
    // setDynamics(stages,x0);
    setDynamics(stages,initial_guess[0].xk);
    setCost(stages);
    // setBounds(stages,x0);
    setBounds(stages,initial_guess[0].xk);
    setPolytopicConstraints(stages);
    setInitialGuess(initial_guess);
//    print_data();

    is_solved_ = Solve(false);
    solver_.data()->clearHessianMatrix();
    solver_.data()->clearLinearConstraintsMatrix();
    solver_.clearSolverVariables();
    solver_.clearSolver();
    if(is_solved_)
    {
        *status = 0;
        is_solved_ = false;
        return getSolution();
    }
    else
    {
        *status = 1;
        std::array<OptVariables,N+1> zero_solution;
        for(size_t i=0;i<N+1;i++)
        {
            // zero_solution[i].xk = x0;
            zero_solution[i].xk = initial_guess[0].xk;
            zero_solution[i].uk.setZero();
            zero_solution[i].slk.setZero();
            zero_solution[i].suk.setZero();
        }
        return zero_solution;
    }
}

// std::array<OptVariables,N+1> OsqpInterface::Solve(bool verbose)
bool OsqpInterface::Solve(bool verbose)
{
    /* 
    min   1/2 x' P x + q' x
     x

    subject to
    l <= A x <= u

    with :
    P sparse (n x n) positive definite
    q dense  (n x 1)
    A sparse (nc x n)
    l dense (nc x 1)
    u dense (nc x 1)
    */

    // const int n = (N+1)*(NX+NU+2*NS); // size of qp state
    // const int nc = (N+1)*NX + (N+1)*(NX+NU+2*NS) + (N+1)*(2*NPC); // size of qp constraints
    const int n = (N+1)*(NX+NU); // size of qp state
    const int nc = (N+1)*NX + (N+1)*(NX+NU) + (N+1)*(NPC); // size of qp constraints

    // Constraint matrix
    A_.resize(nc,n); A_.setZero();
    l_.resize(nc,1); l_.setOnes(); l_ *= -OsqpEigen::INFTY;
    u_.resize(nc,1); u_.setOnes(); u_ *= OsqpEigen::INFTY;
    
    // equality constraints
    A_.block(0, 0, A_eq_.rows(), A_eq_.cols()) = A_eq_; 
    l_.block(0, 0, l_eq_.rows(), 1) = l_eq_;
    u_.block(0, 0, u_eq_.rows(), 1) = u_eq_;
    // bound inequality constraints
    A_.block(A_eq_.rows(), 0, A_ineqb_.rows(), A_ineqb_.cols()) = A_ineqb_; 
    l_.block(l_eq_.rows(), 0, l_ineqb_.rows(), 1) = l_ineqb_;
    u_.block(u_eq_.rows(), 0, u_ineqb_.rows(), 1) = u_ineqb_;
    // polytopic inequality constraints
    A_.block(A_eq_.rows()+A_ineqb_.rows(), 0, A_ineqp_.rows(), A_ineqp_.cols()) = A_ineqp_; 
    l_.block(l_eq_.rows()+l_ineqb_.rows(), 0, l_ineqp_.rows(), 1) = l_ineqp_;
    u_.block(u_eq_.rows()+u_ineqb_.rows(), 0, u_ineqp_.rows(), 1) = u_ineqp_;

    SparseMatrix<double> P(n, n);
    Matrix<double, n, 1> q;
    SparseMatrix<double> A(nc, n);
    Matrix<double, nc, 1> l, u; 
    Matrix<double, n, 1> initial_x;

    P = P_.sparseView();
    q = q_;
    A = A_.sparseView();
    l = l_; 
    u = u_;
    initial_x = initial_x_;

    // settings
    if (!verbose) solver_.settings()->setVerbosity(true);

    // set the initial data of the QP solver
    solver_.data()->setNumberOfVariables(n);
    solver_.data()->setNumberOfConstraints(nc);
    if (!solver_.data()->setHessianMatrix(P))           return false;
    if (!solver_.data()->setGradient(q))                return false;
    if (!solver_.data()->setLinearConstraintsMatrix(A)) return false;
    if (!solver_.data()->setLowerBound(l))              return false;
    if (!solver_.data()->setUpperBound(u))              return false;

    // instantiate the solver
    if (!solver_.initSolver()) return false;
    if (!solver_.setPrimalVariable(initial_x)) return false;

    // solve the QP problem
    if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return false;
    if (solver_.getStatus() != OsqpEigen::Status::Solved) return false;

    // get the controller input
    qp_sol_ = solver_.getSolution();
    // std::cout<<QPSolution<<std::endl;

    return true; 
}

std::array<OptVariables, N + 1> OsqpInterface::getSolution()
{
    std::array<OptVariables,N+1> optimal_solution;
    for(size_t i=0;i<N+1;i++)
    {
        optimal_solution[i].xk = vectorToState(qp_sol_.block(NX*i,0,NX,1));
        optimal_solution[i].uk = vectorToInput(qp_sol_.block(NX*(N+1)+NU*i,0,NU,1));
        // optimal_solution[i].slk = vectorToSlack(qp_sol_.block((NX+NU)*(N+1)+NS*i,0,NS,1));
        // optimal_solution[i].suk = vectorToSlack(qp_sol_.block((NX+NU+NS)*(N+1)+NS*i,0,NS,1));
        optimal_solution[i].slk.setZero();
        optimal_solution[i].suk.setZero();
    }
    optimal_solution[N].uk.setZero();
    return optimal_solution;
}

void OsqpInterface::print_data()
{

}
}