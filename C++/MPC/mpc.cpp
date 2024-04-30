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

#include "mpc.h"

namespace mpcc{
MPC::MPC()
:Ts_(1.0)
{
    std::cout << "default constructor, not everything is initialized properly" << std::endl;
}

MPC::MPC(int max_n_sqp, int n_reset,double max_n_sqp_linesearch, double Ts,const PathToJson &path,std::shared_ptr<RobotModel> robot, std::shared_ptr<SelCollNNmodel> selcolNN)
:Ts_(Ts),
valid_initial_guess_(false),
// solver_interface_(new HpipmInterface()),
solver_interface_(new OsqpInterface()),
param_(Param(path.param_path)),
normalization_param_(NormalizationParam(path.normalization_path)),
bounds_(BoundsParam(path.bounds_path)),
constraints_(Constraints(Ts,path, robot,selcolNN)),
cost_(Cost(path,robot)),
integrator_(Integrator(Ts,path)),
model_(Model(Ts,path)),
track_(ArcLengthSpline(path,robot)),
robot_(robot),
selcolNN_(selcolNN)
{
    max_n_sqp_ = max_n_sqp;
    max_n_sqp_linesearch_ = max_n_sqp_linesearch;
    n_non_solves_ = 0;
    n_no_solves_sqp_ = 0;
    n_reset_ = n_reset;
}

void MPC::setMPCProblem()
{
    for(int i=0;i<=N;i++)
    {
        initial_guess_[i].uk.dqNonZero();
        setStage(initial_guess_[i].xk,initial_guess_[i].uk,i);
    }
}

void MPC::setStage(const State &xk, const Input &uk, const int time_step)
{
    stages_[time_step].nx = NX;
    stages_[time_step].nu = NU;

    if(time_step == 0)
    {
        stages_[time_step].ng = 0;
        stages_[time_step].ns = 0;
    }
    else
    {
        stages_[time_step].ng = NPC;
        stages_[time_step].ns = NS;
    }

    State xk_nz = xk;
    // xk/_nz.vxNonZero(param_.vx_zero);
    stages_[time_step].cost_mat = normalizeCost(cost_.getCost(track_,xk_nz,uk,time_step));
    stages_[time_step].lin_model = normalizeDynamics(model_.getLinModel(xk_nz,uk));
    stages_[time_step].constrains_mat = normalizeCon(constraints_.getConstraints(track_,xk_nz,uk));
    stages_[time_step].l_bounds_x = normalization_param_.T_x_inv*bounds_.getBoundsLX();
    stages_[time_step].u_bounds_x = normalization_param_.T_x_inv*bounds_.getBoundsUX();
    stages_[time_step].l_bounds_u = normalization_param_.T_u_inv*bounds_.getBoundsLU();
    stages_[time_step].u_bounds_u = normalization_param_.T_u_inv*bounds_.getBoundsUU();
    stages_[time_step].l_bounds_s = normalization_param_.T_s_inv*bounds_.getBoundsLS();
    stages_[time_step].u_bounds_s = normalization_param_.T_s_inv*bounds_.getBoundsUS();

    stages_[time_step].l_bounds_x(si_index.s) = normalization_param_.T_x_inv(si_index.s,si_index.s)*
                                                (initial_guess_[time_step].xk.s - param_.s_trust_region);//*initial_guess_[time_step].xk.vs;
    stages_[time_step].u_bounds_x(si_index.s) = normalization_param_.T_x_inv(si_index.s,si_index.s)*
                                                (initial_guess_[time_step].xk.s + param_.s_trust_region);//*initial_guess_[time_step].xk.vs;

}

CostMatrix MPC::normalizeCost(const CostMatrix &cost_mat)
{
    const Q_MPC Q = normalization_param_.T_x*cost_mat.Q*normalization_param_.T_x;
    const R_MPC R = normalization_param_.T_u*cost_mat.R*normalization_param_.T_u;
    const S_MPC S = normalization_param_.T_x*cost_mat.S*normalization_param_.T_u;
    const q_MPC q = normalization_param_.T_x*cost_mat.q;
    const r_MPC r = normalization_param_.T_u*cost_mat.r;
    const Z_MPC Z = normalization_param_.T_s*cost_mat.Z*normalization_param_.T_s;
    const z_MPC z = normalization_param_.T_s*cost_mat.z;
    return {Q,R,S,q,r,Z,z};
}

LinModelMatrix MPC::normalizeDynamics(const LinModelMatrix &lin_model)
{
    const A_MPC A = normalization_param_.T_x_inv*lin_model.A*normalization_param_.T_x;
    const B_MPC B = normalization_param_.T_x_inv*lin_model.B*normalization_param_.T_u;
    const g_MPC g = normalization_param_.T_x_inv*lin_model.g;
    return {A,B,g};
}



ConstrainsMatrix MPC::normalizeCon(const ConstrainsMatrix &con_mat)
{
    const C_MPC C = con_mat.C*normalization_param_.T_x;
    const D_MPC D =  con_mat.D*normalization_param_.T_u;
    const d_MPC dl = con_mat.dl;
    const d_MPC du = con_mat.du;
    return {C,D,dl,du};
}

std::array<OptVariables,N+1> MPC::deNormalizeSolution(const std::array<OptVariables,N+1> &solution)
{
    std::array<OptVariables, N + 1> denormalized_solution;
    StateVector updated_x_vec;
    InputVector updated_u_vec;
    for (int i = 0; i <= N; i++) {
        updated_x_vec = normalization_param_.T_x*stateToVector(solution[i].xk);
        updated_u_vec = normalization_param_.T_u*inputToVector(solution[i].uk);

        denormalized_solution[i].xk = vectorToState(updated_x_vec);
        denormalized_solution[i].uk = vectorToInput(updated_u_vec);
    }
    return denormalized_solution;
}

std::array<OptVariables,N+1> MPC::normalizeSolution(const std::array<OptVariables,N+1> &solution)
{
    std::array<OptVariables, N + 1> normalized_solution;
    StateVector updated_x_vec;
    InputVector updated_u_vec;
    for (int i = 0; i <= N; i++) {
        updated_x_vec = normalization_param_.T_x_inv*stateToVector(solution[i].xk);
        updated_u_vec = normalization_param_.T_u_inv*inputToVector(solution[i].uk);

        normalized_solution[i].xk = vectorToState(updated_x_vec);
        normalized_solution[i].uk = vectorToInput(updated_u_vec);
    }
    return normalized_solution;
}


void MPC::updateInitialGuess(const State &x0)
{
    for(int i=1;i<N;i++) initial_guess_[i-1] = initial_guess_[i];

    initial_guess_[0].xk = x0;
    initial_guess_[0].uk.setZero();

    initial_guess_[N-1].xk = initial_guess_[N-2].xk;
    initial_guess_[N-1].uk.setZero();// = initial_guess_[N-2].uk;

    initial_guess_[N].xk = integrator_.RK4(initial_guess_[N-1].xk,initial_guess_[N-1].uk,Ts_);
    initial_guess_[N].uk.setZero();

    unwrapInitialGuess();
}

// alternatively OptVariables MPC::unwrapInitialGuess(const OptVariables &initial_guess)
void MPC::unwrapInitialGuess()
{
    double L = track_.getLength();
    for(int i=1;i<=N;i++)
    {
        if((initial_guess_[i].xk.s - initial_guess_[i-1].xk.s) > L/2.)
        {
            initial_guess_[i].xk.s -= L;
        }
    }

}

void MPC::generateNewInitialGuess(const State &x0)
{
    std::cout<< "generate new initial guess!!"<<std::endl;
    for(int i = 0;i<=N;i++)
    {
        initial_guess_[i].xk.setZero();
        initial_guess_[i].uk.setZero();

        if (i == 0)
        { 
            initial_guess_[0].xk = x0;
        }
        else
        {
            initial_guess_[i].xk.q1 = initial_guess_[i-1].xk.q1 + Ts_*initial_guess_[i-1].uk.dq1;
            initial_guess_[i].xk.q2 = initial_guess_[i-1].xk.q2 + Ts_*initial_guess_[i-1].uk.dq2;
            initial_guess_[i].xk.q3 = initial_guess_[i-1].xk.q3 + Ts_*initial_guess_[i-1].uk.dq3;
            initial_guess_[i].xk.q4 = initial_guess_[i-1].xk.q4 + Ts_*initial_guess_[i-1].uk.dq4;
            initial_guess_[i].xk.q5 = initial_guess_[i-1].xk.q5 + Ts_*initial_guess_[i-1].uk.dq5;
            initial_guess_[i].xk.q6 = initial_guess_[i-1].xk.q6 + Ts_*initial_guess_[i-1].uk.dq6;
            initial_guess_[i].xk.q7 = initial_guess_[i-1].xk.q7 + Ts_*initial_guess_[i-1].uk.dq7;
            initial_guess_[i].xk.vs = param_.desired_ee_velocity;
            initial_guess_[i].xk.s = initial_guess_[i-1].xk.s + Ts_*initial_guess_[i].xk.vs;
        }


        Eigen::Vector3d track_pos_i = track_.getPostion(initial_guess_[i].xk.s);
        Eigen::Vector3d track_dpos_i = track_.getDerivative(initial_guess_[i].xk.s);
        Eigen::Vector3d track_vel_i = track_dpos_i * initial_guess_[i].xk.vs;

        // Using CLIK to initialize joint and joint velocity
        JointVector q_i = stateToJointVector(initial_guess_[i].xk);
        Eigen::Vector3d ee_posi_i = robot_->getEEPosition(q_i);
        Matrix<double,3,PANDA_DOF> Jv_i = robot_->getJacobianv(q_i);
        Matrix<double,PANDA_DOF,3> pseudo_Jv_inv_i = Jv_i.transpose() * (Jv_i * Jv_i.transpose()).inverse();
        double kp = 100;

        JointVector desired_dq_i = pseudo_Jv_inv_i * (track_vel_i + kp * (track_pos_i - ee_posi_i));

        initial_guess_[i].uk.dq1 = desired_dq_i(0);
        initial_guess_[i].uk.dq2 = desired_dq_i(1);
        initial_guess_[i].uk.dq3 = desired_dq_i(2);
        initial_guess_[i].uk.dq4 = desired_dq_i(3);
        initial_guess_[i].uk.dq5 = desired_dq_i(4);
        initial_guess_[i].uk.dq6 = desired_dq_i(5);
        initial_guess_[i].uk.dq7 = desired_dq_i(6);

        // std::cout << "state: \n" << stateToVector(initial_guess_[i].xk).transpose() <<std::endl; 
        // std::cout << "input: \n" << inputToVector(initial_guess_[i].uk).transpose() <<std::endl; 
    }
    unwrapInitialGuess();
    valid_initial_guess_ = true;
}

std::array<OptVariables,N+1> MPC::sqpSolutionUpdate(const std::array<OptVariables,N+1> &last_solution,
                                                    const std::array<OptVariables,N+1> &current_solution)
{
    // line search
    double d_fk_p=0, dd_fk_p=0;
    double c_k1 = 0;
    std::array<VectorXd,N+1> px, pu, psl, psu;
    for(size_t i=0;i<=N; i++)
    {
        px[i]  = stateToVector(current_solution[i].xk) - stateToVector(last_solution[i].xk);
        pu[i]  = inputToVector(current_solution[i].uk) - inputToVector(last_solution[i].uk);
        psl[i] = slackToVector(current_solution[i].slk) - slackToVector(last_solution[i].slk);
        psu[i] = slackToVector(current_solution[i].suk) - slackToVector(last_solution[i].suk);
    }
    for(size_t i=0;i<=N; i++)
    {
        dd_fk_p += (1/2 * px[i].transpose()  * stages_[i].cost_mat.Q * px[i]  +
                    1/2 * pu[i].transpose()  * stages_[i].cost_mat.R * pu[i]  +
                    1/2 * psl[i].transpose() * stages_[i].cost_mat.Z * psl[i] +
                    1/2 * psu[i].transpose() * stages_[i].cost_mat.Z * psu[i]).value();

        d_fk_p += ((stages_[i].cost_mat.q + stages_[i].cost_mat.Q * stateToVector(last_solution[i].xk)).transpose() * px[i]  +
                   (stages_[i].cost_mat.r + stages_[i].cost_mat.R * inputToVector(last_solution[i].uk)).transpose() * pu[i]  +
                   (stages_[i].cost_mat.z + stages_[i].cost_mat.Z * slackToVector(last_solution[i].slk)).transpose() * psl[i] +
                   (stages_[i].cost_mat.z + stages_[i].cost_mat.Z * slackToVector(last_solution[i].suk)).transpose() * psu[i]).value();

        c_k1 += constraint_norm(stages_[i].constrains_mat.C * stateToVector(last_solution[i].xk) +  stages_[i].constrains_mat.D * inputToVector(last_solution[i].uk),
                                stages_[i].constrains_mat.dl, 
                                stages_[i].constrains_mat.du);
        c_k1 += constraint_norm(stateToVector(last_solution[i].xk), stages_[i].l_bounds_x, stages_[i].u_bounds_x);
        c_k1 += constraint_norm(inputToVector(last_solution[i].uk), stages_[i].l_bounds_u, stages_[i].u_bounds_u);
        c_k1 += constraint_norm(slackToVector(last_solution[i].slk), stages_[i].l_bounds_s, stages_[i].u_bounds_s);
        c_k1 += constraint_norm(slackToVector(last_solution[i].suk), stages_[i].l_bounds_s, stages_[i].u_bounds_s);
    }

    double rho = 0.5; // line search parameter
    double eta = 0.25; // line search parameter
    double tau = 0.5; // line search parameter
    double mu = (d_fk_p + dd_fk_p) / ((1-rho) * c_k1); // penalty parameter
    double alpha = 1; // step parameter
    
    int iter_lin = 0;
    double c_k2 = 0;
    for(size_t i=0;i<=N; i++)
    {
        VectorXd x_step = stateToVector(last_solution[i].xk) + alpha * px[i];
        VectorXd u_step = inputToVector(last_solution[i].uk) + alpha * pu[i];
        VectorXd sl_step = slackToVector(last_solution[i].slk) + alpha * psl[i];
        VectorXd su_step = slackToVector(last_solution[i].suk) + alpha * psu[i];

        ConstrainsMatrix constrains_mat_step = constraints_.getConstraints(track_,vectorToState(x_step),vectorToInput(u_step));

        c_k2 += constraint_norm(constrains_mat_step.C * x_step +  constrains_mat_step.D * u_step,
                                constrains_mat_step.dl, 
                                constrains_mat_step.du);
        c_k2 += constraint_norm(x_step,  bounds_.getBoundsLX(), bounds_.getBoundsUX());
        c_k2 += constraint_norm(u_step,  bounds_.getBoundsLU(), bounds_.getBoundsUU());
        c_k2 += constraint_norm(sl_step, bounds_.getBoundsLS(), bounds_.getBoundsUS());
        c_k2 += constraint_norm(su_step, bounds_.getBoundsLS(), bounds_.getBoundsUS());
    }
    if(fabs(c_k1) > 1E-6) std::cout<<"c_1: "<< c_k1 <<std::endl;
    if(fabs(c_k2) > 1E-6) std::cout<<"c_2: "<< c_k2 <<std::endl;

    while(iter_lin < max_n_sqp_linesearch_)
    {
        if( (alpha*d_fk_p + pow(alpha,2)*dd_fk_p + mu*c_k2 - mu*c_k1) > eta*(alpha*d_fk_p - mu*c_k1) )
        {
            alpha *= tau;
        }
        else break;
        iter_lin++;
    }
    std::cout<<"alpha: "<<alpha << std::endl;

    // update solution by line search alpha
    std::array<OptVariables,N+1> updated_solution;
    StateVector updated_x_vec;
    InputVector updated_u_vec;
    for(int i = 0;i<=N;i++)
    {
        updated_x_vec = alpha*stateToVector(current_solution[i].xk)
                        +(1.0-alpha)*stateToVector(last_solution[i].xk);
        updated_u_vec = alpha*inputToVector(current_solution[i].uk)
                        +(1.0-alpha)*inputToVector(last_solution[i].uk);

        // updated_x_vec = 0.8*stateToVector(current_solution[i].xk)
        //                 +(1.0-0.8)*stateToVector(last_solution[i].xk);
        // updated_u_vec = 0.8*inputToVector(current_solution[i].uk)
        //                 +(1.0-0.8)*inputToVector(last_solution[i].uk);


        updated_solution[i].xk = vectorToState(updated_x_vec);
        updated_solution[i].uk = vectorToInput(updated_u_vec);
    }

    return updated_solution;
}

double MPC::constraint_norm(const VectorXd &constr, const VectorXd &l, const VectorXd &u)
{
    double c_l1 = 0;
    // l <= c(x) <= u
    c_l1 += (l - constr).cwiseMax(0.0).sum();
    c_l1 += (constr - u).cwiseMax(0.0).sum();

    return c_l1;
}

MPCReturn MPC::runMPC(State &x0)
{
    auto t1 = std::chrono::high_resolution_clock::now();
    int solver_status = -1;
    x0.s = track_.projectOnSpline(x0);
    x0.unwrap(track_.getLength());
    if(valid_initial_guess_)
        updateInitialGuess(x0);
    else
        generateNewInitialGuess(x0);

    initial_guess_nor_ = normalizeSolution(initial_guess_);

    //TODO: this is one approach to handle solver errors, works well in simulation
    n_no_solves_sqp_ = 0;
    for(int i=0;i<max_n_sqp_;i++)
    {
        std::cout << "sqp iter: "<<i <<std::endl;
        // std::cout << "initial_guess: " <<std::endl;
        // for(int ii=0; ii<=N;ii++){
        //     std::cout << "state: \n" << stateToVector(initial_guess_[ii].xk).transpose() <<std::endl; 
        //     std::cout << "input: \n" << inputToVector(initial_guess_[ii].uk).transpose() <<std::endl; 
        // }

        setMPCProblem();
        // State x0_normalized = vectorToState(normalization_param_.T_x_inv*stateToVector(x0));
        // optimal_solution_ = solver_interface_->solveMPC(stages_,x0_normalized, &solver_status);
        optimal_solution_ = solver_interface_->solveMPC(stages_,initial_guess_nor_, &solver_status);

        // optimal_solution_ = deNormalizeSolution(optimal_solution_);
        // std::cout << "optimal_solution: " <<std::endl;
        // for(int ii=0; ii<=N;ii++){
        //     std::cout << "state: \n" << stateToVector(optimal_solution_[ii].xk).transpose() <<std::endl; 
        //     std::cout << "input: \n" << inputToVector(optimal_solution_[ii].uk).transpose() <<std::endl; 
        // }
        if(solver_status != 0)
            n_no_solves_sqp_++;
        if(solver_status <= 1)
            initial_guess_nor_ = sqpSolutionUpdate(initial_guess_nor_,optimal_solution_);
    }

    initial_guess_ = deNormalizeSolution(initial_guess_nor_);

    const int max_error = std::max(max_n_sqp_-1,1);
    if(n_no_solves_sqp_ >= max_error)
        n_non_solves_++;
    else
        n_non_solves_ = 0;

    if(n_non_solves_ >= n_reset_){
        valid_initial_guess_ = false;
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    double time_nmpc = time_span.count();

    return {initial_guess_[0].uk,initial_guess_,time_nmpc};
}

void MPC::setTrack(const Eigen::VectorXd &X, const Eigen::VectorXd &Y,const Eigen::VectorXd &Z){
    track_.gen3DSpline(X,Y,Z);
}


}