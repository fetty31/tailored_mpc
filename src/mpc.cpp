#include "mpc.hh"

// Constructor
MPC::MPC(const Params& params){

    // NLOP params
    this->n_states = params.mpc.nlop.n_states;
    this->n_controls = params.mpc.nlop.n_controls;
    this->N = params.mpc.nlop.N;
    this->Npar = params.mpc.nlop.Npar + this->n_states + this->N; // [ 23 (MPC parameters) + (initial state) + n (curvature points == N) ]

    // MPC
    this->Hz = params.mpc.Hz;
    this->rk4_t = params.mpc.rk4_t;
    this->nPlanning = params.mpc.nPlanning;

    // Vehicle params
    this->m = params.vehicle.m;
    this->longue = params.vehicle.longue;
    this->width = params.vehicle.width;
    this->Lr = params.vehicle.Lr;
    this->Lf = params.vehicle.Lf;
    this->d_IMU = params.vehicle.d_IMU;
    this->I = params.vehicle.I;
    this->Ar = params.vehicle.Ar;
    this->rho = params.vehicle.rho;
    this->gravity = params.vehicle.gravity;

    this->FORCES = params.FORCES;

    planner = Eigen::MatrixXd::Zero(nPlanning,7);
    carState = Eigen::VectorXd::Zero(8);
    predicted_s = Eigen::VectorXd::Zero(N);

    lastState = Eigen::MatrixXd::Zero(N,6);
    lastCommands = Eigen::MatrixXd::Zero(N,4);
    solStates = Eigen::MatrixXd::Zero(N,5);
    solCommands = Eigen::MatrixXd::Zero(N,4);

    this->paramFlag = true;

}

///////////////////////////////////////////////////////////////////////////////////////////////
//------------------------Callback functions--------------------------------------------------

void MPC::stateCallback(const as_msgs::CarState::ConstPtr& msg){

    carState << msg->odom.position.x, msg->odom.position.y, msg->odom.heading, msg->odom.velocity.x, msg->odom.velocity.y + msg->odom.velocity.w*d_IMU, msg->odom.velocity.w, msg->steering, msg->odom.acceleration.x;

    stateFlag = true;

}


void MPC::plannerCallback(const as_msgs::ObjectiveArrayCurv::ConstPtr& msg){

    if (msg->objectives.size() < nPlanning){
			ROS_WARN("MPC: Planner is too short!");
			return;
    }

    // Fill planner matrix
    for (unsigned int i = 0; i < nPlanning ; i++)
    {	
        planner(i, 0) = msg->objectives[i].x;
        planner(i, 1) = msg->objectives[i].y;
        planner(i, 2) = msg->objectives[i].s; 
        planner(i, 3) = msg->objectives[i].k; 
        planner(i, 4) = msg->objectives[i].vx;
        planner(i, 5) = msg->objectives[i].L;
        planner(i, 6) = msg->objectives[i].R;
    }

    smax = 20000;
    plannerFlag = true;

}


///////////////////////////////////////////////////////////////////////////////////////////////
//------------------------Principal functions--------------------------------------------------

void MPC::solve(){

    if(paramFlag && dynParamFlag && plannerFlag && stateFlag){
        
        auto start_time = chrono::system_clock::now();

        if(!this->FORCES){
            solve_IPOPT();
        }else{
            solve_FORCES();
        }

        get_solution();
        s_prediction();

        auto finish_time = chrono::system_clock::now();

        chrono::duration<double> elapsed_seconds = finish_time - start_time;

        ROS_WARN("TAILORED MPC elapsed time: %f ms", elapsed_seconds.count()*1000);
        
    }else{

        if(!paramFlag || !dynParamFlag){
            ROS_ERROR("MPC: Parameters aren't properly defined");
            ROS_ERROR_STREAM("Static params: " << paramFlag);
            ROS_ERROR_STREAM("Dyn params: " << dynParamFlag);
        }else{
            ROS_ERROR("MPC: No data from state car or planner");
            ROS_ERROR_STREAM("Planner: " << plannerFlag);
            ROS_ERROR_STREAM("State: " << stateFlag);
        }
    }
}

void MPC::solve_FORCES(){

    set_FORCES();

    /* Get i-th memory buffer */
    int i = 0;
    forces.mem_handle = TailoredSolver_internal_mem(i);
    /* Note: number of available memory buffers is controlled by code option max_num_mem */

    // Solve
    forces.exit_flag = TailoredSolver_solve(&forces.params, &forces.solution, &forces.info, forces.mem_handle, NULL, forces.ext_func);

    ROS_WARN_STREAM("MPC exit flag: " << forces.exit_flag);

    if(forces.exit_flag == 1) this->firstIter = false;
    else this->firstIter = true;

}


void MPC::solve_IPOPT(){

    set_boundaries_IPOPT();
    set_parameters_IPOPT();

    // Bounds and initial guess
    std::map<std::string, casadi::DM> arg, sol;
    arg["lbx"] = ipopt.lbx;
    arg["ubx"] = ipopt.ubx;
    arg["lbg"] = vconcat(vconcat(ipopt.lbg_next,ipopt.lbg_track),ipopt.lbg_elipse);
    arg["ubg"] = vconcat(vconcat(ipopt.ubg_next,ipopt.ubg_track),ipopt.ubg_elipse);
    arg["x0"] = ipopt.x0;
    arg["p"] = ipopt.p;

    // Solve the NLOP
    if(ipopt.solver_ptr){
        sol = (*ipopt.solver_ptr)(arg);

        casadi::Dict stats;
        stats = (*ipopt.solver_ptr).stats();

        ipopt.exit_flag = stats["return_status"].get_str(); // Maximum_Iterations_Exceeded
        ipopt.solution = vector<double>(sol.at("x"));

        // Exit Flag:
        cout << "IPOPT EXIT FLAG = " << ipopt.exit_flag << endl;

        cout << "SOLUTION: " << endl;
        printVec(ipopt.solution, 15);

    }else{
        ROS_ERROR("IPOPT SOLVER POINTER IS NULL");
    }
    
}

///////////////////////////////////////////////////////////////////////////////////////////////
//------------------------Auxiliar functions--------------------------------------------------

vector<double> MPC::initial_conditions(){

    // Calculate tangent vector of the trajectory
    Eigen::Vector2d tangent;
    tangent << planner(1,0) - planner(0,0), planner(1,1) - planner(0,1);

    // Calculate vector from car to first point of the trajectory
    Eigen::Vector2d s0_to_car;
    s0_to_car << carState(0) - planner(0,0), carState(1) - planner(0,1);

    // Calculate cross product to get angle
    double cross = s0_to_car(0)*tangent(1) - s0_to_car(1)*tangent(0);
    double angle = asin(cross/(s0_to_car.norm()*tangent.norm())); // angle between car position and heading of the trajectory

    // Get sign
    int sign = angle/fabs(angle);

    if(fabs(angle) > M_PI/2) angle = fabs(angle) - M_PI/2;
    else angle = M_PI/2 - fabs(angle);

    double n0 = s0_to_car.norm()*cos(angle)*sign; // normal distance from car to track
    double t_angle = atan2(tangent(1), tangent(0)); // heading of the trajectory
    double mu0 = -(t_angle - carState(2));

    ROS_WARN_STREAM("mu0: " << mu0);
    ROS_WARN_STREAM("n0: " << n0);

    vector<double> xinit(this->n_states); // x = [delta, acc, n, mu, Vx, Vy, w]
    xinit[0] = carState(6);
    xinit[1] = carState(7);
    xinit[2] = n0;
    xinit[3] = mu0;
    xinit[4] = carState(3);
    xinit[5] = carState(4);
    xinit[6] = carState(5);

    return xinit;

}

void MPC::s_prediction(){

    predicted_s.setZero();
    cout << "S_PRED planner(0,2): " << planner(0,2) << endl;
    predicted_s(0) = fmod(planner(0,2), smax);

    lastState(0,0) = carState(0);
    lastState(0,1) = carState(1);
    lastState(0,2) = carState(2);
    lastState(0,3) = solStates(0,2);
    lastState(0,4) = solStates(0,3);
    lastState(0,5) = solStates(0,4);

    for(int i=1; i < N; i++){

        double theta = lastState(i-1, 2);
        double n = solStates(i-1, 0);
        double mu = solStates(i-1, 1);
        double vx = solStates(i-1, 2);
        double vy = solStates(i-1, 3);
        double w = solStates(i-1, 4);
        double k = forces.params.all_parameters[23 + (i-1)*this->Npar];

        double sdot = (vx*cos(mu) - vy*sin(mu))/(1 - n*k);

        predicted_s(i) = fmod( predicted_s(i-1) + sdot*this->rk4_t, this->smax );

        // Ensure sdot > 0
        if(sdot < 0){
            ROS_ERROR("MPC FATAL ERROR, negative sdot");
            this->firstIter = true;
            break;
        }

        // Predict states evolution with Euler
        lastState(i,0) = lastState(i-1,0) + (vx*cos(theta) - vy*sin(theta))*this->rk4_t;
        lastState(i,1) = lastState(i-1,1) + (vx*sin(theta) + vy*cos(theta))*this->rk4_t;
        lastState(i,2) = lastState(i-1,2) + w*this->rk4_t;
        lastState(i,3) = vx;
        lastState(i,4) = vy;
        lastState(i,5) = w;

        lastCommands(i-1,0) = solCommands(i-1,0);
        lastCommands(i-1,1) = solCommands(i-1,1);
        lastCommands(i-1,2) = solCommands(i-1,2);
        lastCommands(i-1,3) = solCommands(i-1,3);

    }

    // If predicted s is too small set initial s again
    // double totalLength = predicted_s(this->N-1) - predicted_s(0);
    // if(totalLength < 0) totalLength += this->smax;    
    // ...
}

void MPC::get_solution(){

    if(!this->FORCES){
        // Concatenate optimized stages in Xopt ( 9*(N+1) x 1) --> (9 x (N+1) )
        Eigen::MatrixXd optimized = vector2eigen(ipopt.solution);
        Eigen::Map<Eigen::MatrixXd> stages(optimized.topRows(n_states*(N+1)).data(),n_states,N+1);
        Eigen::Map<Eigen::MatrixXd> controls(optimized.bottomRows(n_controls*(N+1)).data(),n_controls,N+1);
        Eigen::MatrixXd Xopt(stages.rows()+controls.rows(), stages.cols());
        Xopt << controls, 
                stages;
        Eigen::MatrixXd solStatesT, solCommandsT;
        solStatesT = Xopt.bottomRows(5);
        solCommandsT = Xopt.topRows(4);

        solStates = solStatesT.transpose();     // [n, mu, vx, vy, w] 
        solCommands = solCommandsT.transpose(); // [diff_delta, diff_acc, delta, acc]

    }else{
        // Change vec dimensions from x(5*N x 1) u(4*N x 1) to x(N x 5) u(N x 4)
        Eigen::MatrixXd u = array2eigen(forces.solution.U);
        Eigen::MatrixXd x = array2eigen(forces.solution.X);

        Eigen::Map<Eigen::MatrixXd> controlsT(u.data(), 4, N);
        Eigen::Map<Eigen::MatrixXd> statesT(u.data(), 5, N);

        solStates = statesT.transpose();
        solCommands = controlsT.transpose();
    }

}

void MPC::set_FORCES(){

    int id_k = 0;       // curvature's index
    int id_sinit = 0;   // initial s index
    double mean_s = 0;  // mean s discretization

    int size = sizeof(forces.params.hu)/sizeof(forces.params.hu[0]);
    int nh = int(size/this->N);
    int Nvar = this->n_states + this->n_controls;

    vector<double> xinit = initial_conditions(); // initial conditions evaluation
    
    if(!firstIter){

        // Loop to find minimum distance between s0 and predicted s
        double min_dist = 10;
        for(int index = 0; index < predicted_s.size(); index++){

            double diff = fabs(planner(0,2) - predicted_s(index));

            if(diff < 0.1){
                id_sinit = index;
                break;

            }else if(diff < min_dist){
                min_dist = diff;
                id_sinit = index;
            }
        }
    }

    for(int k = 0; k < this->N; ++k){

        this->forces.params.all_parameters[ 0 + k*this->Npar] = this->dRd; 
        this->forces.params.all_parameters[ 1 + k*this->Npar] = this->dRa;
        this->forces.params.all_parameters[ 2 + k*this->Npar] = this->m;
        this->forces.params.all_parameters[ 3 + k*this->Npar] = this->I;
        this->forces.params.all_parameters[ 4 + k*this->Npar] = this->Lf;
        this->forces.params.all_parameters[ 5 + k*this->Npar] = this->Lr;
        this->forces.params.all_parameters[ 6 + k*this->Npar] = this->Dr;
        this->forces.params.all_parameters[ 7 + k*this->Npar] = this->Df;
        this->forces.params.all_parameters[ 8 + k*this->Npar] = this->Cr;
        this->forces.params.all_parameters[ 9 + k*this->Npar] = this->Cf;
        this->forces.params.all_parameters[10 + k*this->Npar] = this->Br;
        this->forces.params.all_parameters[11 + k*this->Npar] = this->Bf;
        this->forces.params.all_parameters[12 + k*this->Npar] = this->u_r;   
        this->forces.params.all_parameters[13 + k*this->Npar] = this->gravity;
        this->forces.params.all_parameters[14 + k*this->Npar] = this->Cd;
        this->forces.params.all_parameters[15 + k*this->Npar] = this->rho;   
        this->forces.params.all_parameters[16 + k*this->Npar] = this->Ar; 
        this->forces.params.all_parameters[17 + k*this->Npar] = this->q_slip;
        this->forces.params.all_parameters[18 + k*this->Npar] = this->p_long;
        this->forces.params.all_parameters[19 + k*this->Npar] = this->q_n;
        this->forces.params.all_parameters[20 + k*this->Npar] = this->q_mu;
        this->forces.params.all_parameters[21 + k*this->Npar] = this->lambda;
        this->forces.params.all_parameters[22 + k*this->Npar] = this->q_s;


        int plannerIdx;
        if(firstIter){
            plannerIdx = this->samplingS*k;
        }else{
            if(id_sinit < this->N){

                // Set k(s), L(s), R(s) with the prediction from last MPC iteration 
                double diff_s = predicted_s(id_sinit) - this->planner(0, 2);
                id_k = int(round(diff_s/this->delta_s));

                if(diff_s < 0) id_k = 0;

                // Average of last 5 delta_s 
                // TO DO: 
                //      -make longer
                if(k != 0 && id_sinit > this->N - 5){
                    diff_s = predicted_s(id_sinit) - this->forces.params.all_parameters[23 + (k-1)*this->Npar];

                    if(diff_s < 0) diff_s += this->smax; // If we are passing the start line, reset diff
                    mean_s += diff_s;
                }

                if(id_sinit == this->N) {
                    mean_s = round(mean_s/5/delta_s);
                }

                id_sinit++;
                plannerIdx = id_k;

            }else{
                // Set k(s), L(s), R(s) with planner actual discretization
                // TO DO:
                //      - increase discretization (more than the planner)
                //      - maybe rk4 time from matlab would have to be real time param 
                id_k += int(mean_s);
                plannerIdx = id_k;
            }
        }

        this->forces.params.all_parameters[23 + k*this->Npar] = planner(plannerIdx, 2); // curvature 
        // L(s), R(s) --> planner(plannerIdx, 5:6)

        // Inequality constraints bounds:
        this->forces.params.hu[k*nh] = this->lambda;
        this->forces.params.hu[k*nh + 1] = this->lambda;
        this->forces.params.hu[k*nh + 2] = planner(plannerIdx, 5); // L(s) ortogonal left dist from the path to the track limits
        this->forces.params.hu[k*nh + 3] = planner(plannerIdx, 6); // R(s) ortogonal right dist from the path to the track limits

        // Equality constraints bounds:
        this->forces.params.lb[k*Nvar] = this->bounds.u_min[0];
        this->forces.params.lb[k*Nvar + 1] = this->bounds.u_min[1];
        this->forces.params.lb[k*Nvar + 2] = this->bounds.x_min[0];
        this->forces.params.lb[k*Nvar + 3] = this->bounds.x_min[1];
        this->forces.params.lb[k*Nvar + 4] = this->bounds.x_min[2];
        this->forces.params.lb[k*Nvar + 5] = this->bounds.x_min[3];
        this->forces.params.lb[k*Nvar + 6] = this->bounds.x_min[4];
        this->forces.params.lb[k*Nvar + 7] = this->bounds.x_min[5];
        this->forces.params.lb[k*Nvar + 8] = this->bounds.x_min[6];

        this->forces.params.ub[k*Nvar] = this->bounds.u_max[0];
        this->forces.params.ub[k*Nvar + 1] = this->bounds.u_max[1];
        this->forces.params.ub[k*Nvar + 2] = this->bounds.x_max[0];
        this->forces.params.ub[k*Nvar + 3] = this->bounds.x_max[1];
        this->forces.params.ub[k*Nvar + 4] = this->bounds.x_max[2];
        this->forces.params.ub[k*Nvar + 5] = this->bounds.x_max[3];
        this->forces.params.ub[k*Nvar + 6] = this->bounds.x_max[4];
        this->forces.params.ub[k*Nvar + 7] = this->bounds.x_max[5];
        this->forces.params.ub[k*Nvar + 8] = this->bounds.x_max[6];

        if(!firstIter){
            if((latency + k) < this->N - 1){
                if(this->forces.exit_flag == 1){
                    this->forces.params.x0[k*Nvar] = solCommands(latency+k, 0);
                    this->forces.params.x0[k*Nvar + 1] = solCommands(latency+k, 1);
                    this->forces.params.x0[k*Nvar + 2] = solCommands(latency+k, 2);
                    this->forces.params.x0[k*Nvar + 3] = solCommands(latency+k, 3);
                    this->forces.params.x0[k*Nvar + 4] = solStates(latency+k, 0);
                    this->forces.params.x0[k*Nvar + 5] = solStates(latency+k, 1);
                    this->forces.params.x0[k*Nvar + 6] = solStates(latency+k, 2);
                    this->forces.params.x0[k*Nvar + 7] = solStates(latency+k, 3);
                    this->forces.params.x0[k*Nvar + 8] = solStates(latency+k, 4);
                }else{
                    this->forces.params.x0[k*Nvar] = (this->forces.params.lb[k*Nvar] + this->forces.params.ub[k*Nvar])/2;
                    this->forces.params.x0[k*Nvar + 1] = (this->forces.params.lb[k*Nvar + 1] + this->forces.params.ub[k*Nvar + 1])/2;
                    this->forces.params.x0[k*Nvar + 2] = (this->forces.params.lb[k*Nvar + 2] + this->forces.params.ub[k*Nvar + 2])/2;
                    this->forces.params.x0[k*Nvar + 3] = (this->forces.params.lb[k*Nvar + 3] + this->forces.params.ub[k*Nvar + 3])/2;
                    this->forces.params.x0[k*Nvar + 4] = (this->forces.params.lb[k*Nvar + 4] + this->forces.params.ub[k*Nvar + 4])/2;
                    this->forces.params.x0[k*Nvar + 5] = (this->forces.params.lb[k*Nvar + 5] + this->forces.params.ub[k*Nvar + 5])/2;
                    this->forces.params.x0[k*Nvar + 6] = (this->forces.params.lb[k*Nvar + 6] + this->forces.params.ub[k*Nvar + 6])/2;
                    this->forces.params.x0[k*Nvar + 7] = (this->forces.params.lb[k*Nvar + 7] + this->forces.params.ub[k*Nvar + 7])/2;
                    this->forces.params.x0[k*Nvar + 8] = (this->forces.params.lb[k*Nvar + 8] + this->forces.params.ub[k*Nvar + 8])/2;
                }
            }else{
                if(this->forces.exit_flag == 1){
                    this->forces.params.x0[k*Nvar] = solCommands(N-1, 0);
                    this->forces.params.x0[k*Nvar + 1] = solCommands(N-1, 1);
                    this->forces.params.x0[k*Nvar + 2] = solCommands(N-1, 2);
                    this->forces.params.x0[k*Nvar + 3] = solCommands(N-1, 3);
                    this->forces.params.x0[k*Nvar + 4] = solStates(N-1, 0);
                    this->forces.params.x0[k*Nvar + 5] = solStates(N-1, 1);
                    this->forces.params.x0[k*Nvar + 6] = solStates(N-1, 2);
                    this->forces.params.x0[k*Nvar + 7] = solStates(N-1, 3);
                    this->forces.params.x0[k*Nvar + 8] = solStates(N-1, 4);
                }else{
                    this->forces.params.x0[k*Nvar] = (this->forces.params.lb[k*Nvar] + this->forces.params.ub[k*Nvar])/2;
                    this->forces.params.x0[k*Nvar + 1] = (this->forces.params.lb[k*Nvar + 1] + this->forces.params.ub[k*Nvar + 1])/2;
                    this->forces.params.x0[k*Nvar + 2] = (this->forces.params.lb[k*Nvar + 2] + this->forces.params.ub[k*Nvar + 2])/2;
                    this->forces.params.x0[k*Nvar + 3] = (this->forces.params.lb[k*Nvar + 3] + this->forces.params.ub[k*Nvar + 3])/2;
                    this->forces.params.x0[k*Nvar + 4] = (this->forces.params.lb[k*Nvar + 4] + this->forces.params.ub[k*Nvar + 4])/2;
                    this->forces.params.x0[k*Nvar + 5] = (this->forces.params.lb[k*Nvar + 5] + this->forces.params.ub[k*Nvar + 5])/2;
                    this->forces.params.x0[k*Nvar + 6] = (this->forces.params.lb[k*Nvar + 6] + this->forces.params.ub[k*Nvar + 6])/2;
                    this->forces.params.x0[k*Nvar + 7] = (this->forces.params.lb[k*Nvar + 7] + this->forces.params.ub[k*Nvar + 7])/2;
                    this->forces.params.x0[k*Nvar + 8] = (this->forces.params.lb[k*Nvar + 8] + this->forces.params.ub[k*Nvar + 8])/2;
                }
            }
        }else{

            if(k==0){
                this->forces.params.x0[k*Nvar] = 0;
                this->forces.params.x0[k*Nvar + 1] = 0;
                this->forces.params.x0[k*Nvar + 2] = xinit[0];
                this->forces.params.x0[k*Nvar + 3] = xinit[1];
                this->forces.params.x0[k*Nvar + 4] = xinit[2];
                this->forces.params.x0[k*Nvar + 5] = xinit[3];
                this->forces.params.x0[k*Nvar + 6] = xinit[4];
                this->forces.params.x0[k*Nvar + 7] = xinit[5];
                this->forces.params.x0[k*Nvar + 8] = xinit[6];
            }else{
                this->forces.params.x0[k*Nvar] = (this->forces.params.lb[k*Nvar] + this->forces.params.ub[k*Nvar])/2;
                this->forces.params.x0[k*Nvar + 1] = (this->forces.params.lb[k*Nvar + 1] + this->forces.params.ub[k*Nvar + 1])/2;
                this->forces.params.x0[k*Nvar + 2] = (this->forces.params.lb[k*Nvar + 2] + this->forces.params.ub[k*Nvar + 2])/2;
                this->forces.params.x0[k*Nvar + 3] = (this->forces.params.lb[k*Nvar + 3] + this->forces.params.ub[k*Nvar + 3])/2;
                this->forces.params.x0[k*Nvar + 4] = (this->forces.params.lb[k*Nvar + 4] + this->forces.params.ub[k*Nvar + 4])/2;
                this->forces.params.x0[k*Nvar + 5] = (this->forces.params.lb[k*Nvar + 5] + this->forces.params.ub[k*Nvar + 5])/2;
                this->forces.params.x0[k*Nvar + 6] = (this->forces.params.lb[k*Nvar + 6] + this->forces.params.ub[k*Nvar + 6])/2;
                this->forces.params.x0[k*Nvar + 7] = (this->forces.params.lb[k*Nvar + 7] + this->forces.params.ub[k*Nvar + 7])/2;
                this->forces.params.x0[k*Nvar + 8] = (this->forces.params.lb[k*Nvar + 8] + this->forces.params.ub[k*Nvar + 8])/2;
            }
        }
    }

}

// set_boundaries_IPOPT: Set boundaries for state, control variables and equality & inequality constraints
void MPC::set_boundaries_IPOPT(){

    vector<double> X_MIN;
    vector<double> X_MAX;
    vector<double> X0;
    vector<double> U_MIN;
    vector<double> U_MAX;
    vector<double> U0;

    // Reserve memory
    X_MIN.reserve(this->bounds.x_min.size()*(this->N+1));
    X_MAX.reserve(this->bounds.x_max.size()*(this->N+1));
    X0.reserve(this->bounds.x0.size()*(this->N+1));
    U_MIN.reserve(this->bounds.u_min.size()*this->N);
    U_MAX.reserve(this->bounds.u_max.size()*this->N);
    U0.reserve(this->bounds.u0.size()*this->N);

    // Include lambda in upper_ellipse
    this->bounds.upper_ellipse = this->lambda;

    // Bounds of constraints
    vector<double> lbg_next(n_states*(N+1), this->bounds.lower_continuity);  // lower boundaries continuity constraint
    vector<double> lbg_track(2*(N+1), this->bounds.lower_track);         // lower boundaries track constraint
    vector<double> lbg_elipse(2*(N+1), this->bounds.lower_ellipse);      // lower boundaries ellipse constraint

    vector<double> ubg_next(n_states*(N+1), this->bounds.upper_continuity);  // upper boundaries continuity constraint
    vector<double> ubg_elipse(2*(N+1), this->bounds.upper_ellipse);      // upper boundaries ellipse constraint

    vector<double> ubg_track; // upper boundaries track constraint
    for(unsigned i=0; i<N+1; i++){
        ubg_track.push_back(planner(i,7));
        ubg_track.push_back(planner(i,8));
    }

    ipopt.lbg_next = lbg_next;
    ipopt.lbg_track = lbg_track;
    ipopt.lbg_elipse = lbg_elipse;
    ipopt.ubg_next = ubg_next;
    ipopt.ubg_track = ubg_track;
    ipopt.ubg_elipse = ubg_elipse;

    for(int k=0; k<N+1; k++){

        X_MIN.insert(X_MIN.end(), this->bounds.x_min.begin(), this->bounds.x_min.end());
        X_MAX.insert(X_MAX.end(), this->bounds.x_max.begin(), this->bounds.x_max.end());

        X0.insert(X0.end(), this->bounds.x0.begin(), this->bounds.x0.end());

        if(k<N){
        
        U_MIN.insert(U_MIN.end(), this->bounds.u_min.begin(), this->bounds.u_min.end());
        U_MAX.insert(U_MAX.end(), this->bounds.u_max.begin(), this->bounds.u_max.end());

        U0.insert(U0.end(), this->bounds.u0.begin(), this->bounds.u0.end());
        }
    }

    ipopt.lbx = vconcat(X_MIN,U_MIN);
    ipopt.ubx = vconcat(X_MAX,U_MAX);
    ipopt.x0 = vconcat(X0,U0);


}

void MPC::set_parameters_IPOPT(){

    vector<double> param = {this->dRd, 
                        this->dRa, 
                        this->m, 
                        this->I, 
                        this->Lf, 
                        this->Lr, 
                        this->Dr, 
                        this->Df, 
                        this->Cr, 
                        this->Cf, 
                        this->Br, 
                        this->Bf, 
                        this->u_r, 
                        this->gravity, 
                        this->Cd, 
                        this->rho, 
                        this->Ar, 
                        this->q_slip, 
                        this->p_long, // not used
                        this->q_n, 
                        this->q_mu, 
                        this->lambda, 
                        this->q_s};

    vector<double> curvature(this->N);
    for (unsigned int i = 0; i < this->N; i++){
        curvature[i] = planner(i,3);
    }

    vector<double> xinit = initial_conditions(); // initial conditions evaluation

    param.reserve(xinit.size()+param.size()+curvature.size()); // reserve memory

    param.insert(param.end(), xinit.begin(), xinit.end()); // insert initial state vector

    param.insert(param.end(), curvature.begin(), curvature.end()); // insert midline path's curvature

    ipopt.p = param;
    this->Npar = param.size();

    cout << "XINIT: " << endl;
    printVec(xinit);

}

void MPC::msgCommands(as_msgs::CarCommands *msg){

    msg->header.stamp = ros::Time::now();
    msg->motor = solCommands(this->latency, 3);
    msg->steering = solCommands(this->latency, 2);
    msg->Mtv = 0.0;
    return;
}

vector<double> MPC::vconcat(const vector<double>& x, const vector<double>& y){
    vector<double> v(x.size() + y.size(),0.0);
	move(x.begin(), x.end(), v.begin());
	move(y.begin(), y.end(), v.begin() + x.size());
    return v;
}

void MPC::printVec(vector<double> &input, int firstElements){
    if(firstElements!=0){
      for (auto it = input.begin(); it != input.end()-input.size()+firstElements; it++) {
        cout << *it << "\n";
      }
    }else{
        for (auto it = input.begin(); it != input.end(); it++) {
        cout << *it << "\n";
      }
    }   
}

Eigen::MatrixXd MPC::vector2eigen(vector<double> vect){

    Eigen::MatrixXd result(vect.size(),1);
    int rows = 0;
    vector<double>::iterator row;
    for(row = vect.begin(); row != vect.end(); ++row){
            result(rows,0) = *row; 
            rows++;
    }   
    return result;
}

Eigen::MatrixXd MPC::array2eigen(double *arr){

    int size = sizeof(*arr)/sizeof(arr[0]);
    cout << "Array size is " << size << endl;
    Eigen::MatrixXd result(size,1);
    for(int i = 0; i < size; i++){
        result(i,0) = arr[i];
    }
    return result;
}


void MPC::reconfigure(tailored_mpc::dynamicConfig& config){

    try{ 
        
        this->dRd = config.dRd;
        this->dRa = config.dRa; 
        this->Dr = config.Dr;
        this->Df = config.Df;
        this->Cr = config.Cr;
        this->Cf = config.Cf;
        this->Br = config.Br;
        this->Bf = config.Bf;
        this->u_r = config.u_r;
        this->Cd = config.Cd;
        this->q_slip = config.q_slip;
        this->p_long = config.p_long;
        this->q_n = config.q_n;
        this->q_mu = config.q_mu;
        this->lambda = config.lambda;
        this->q_s = config.q_s;
        this->latency = config.latency;

        this->bounds.u_min[0] = config.diff_delta_min;
        this->bounds.u_min[1] = config.diff_acc_min;
        this->bounds.u_max[0] = config.diff_delta_max;
        this->bounds.u_max[1] = config.diff_acc_max;

        this->dynParamFlag = true;


    } catch (exception& e){
        ROS_ERROR_STREAM("MPC RECONFIGURE DIED" << e.what());
    }

}

