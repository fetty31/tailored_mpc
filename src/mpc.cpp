#include "mpc.hh"

// Constructor
MPC::MPC(const Params* params){

    // NLOP params
    this->N = params->mpc.nlop.N;
    this->Nslacks = params->mpc.nlop.Nslacks;

    // MPC
    this->Hz            = params->mpc.Hz;
    this->nPlanning     = params->mpc.nPlanning;
    this->Nthreads      = params->mpc.Nthreads;
    this->troProfile    = params->mpc.TroProfile;

    cout << "troProfile: " << troProfile << endl;

    // Vehicle params
    this->m         = params->vehicle.m;
    this->longue    = params->vehicle.longue;
    this->width     = params->vehicle.width;
    this->Lr        = params->vehicle.Lr;
    this->Lf        = params->vehicle.Lf;
    this->d_IMU     = params->vehicle.d_IMU;
    this->I         = params->vehicle.I;
    this->Ar        = params->vehicle.Ar;
    this->rho       = params->vehicle.rho;
    this->gravity   = params->vehicle.gravity;
    this->Rwheel    = params->vehicle.Rwheel;

    this->sizeU = sizeof(forces.solution.U)/sizeof(forces.solution.U[0]); // size of control FORCES array
    this->sizeX = sizeof(forces.solution.X)/sizeof(forces.solution.X[0]); // size of states FORCES array

    cout << "sizeU: "  << sizeU << endl;
    cout << "sizeX: "  << sizeX << endl;

    int sizeParam = sizeof(forces.params.all_parameters)/sizeof(forces.params.all_parameters[0]); // size of params FORCES array
    this->Npar = int(sizeParam/N);

    this->n_states = int(sizeX/N);      // number of car state variables
    this->n_controls = int(sizeU/N);    // number of commands variables    

    cout << "n_states: " << n_states << endl;
    cout << "n_controls: " << n_controls << endl;

    planner = Eigen::MatrixXd::Zero(nPlanning,7);
    carState = Eigen::VectorXd::Zero(9);
    predicted_t = Eigen::VectorXd::Zero(N);

    lastState = Eigen::MatrixXd::Zero(N,6);                     // [x, y, theta, vx, vy, w]
    lastCommands = Eigen::MatrixXd::Zero(N,n_controls-Nslacks); // [diff_delta, diff_Fm, Mtv, delta, acc]

    solStates = Eigen::MatrixXd::Zero(N,n_states);        // [n, mu, vx, vy, w]
    solCommands = Eigen::MatrixXd::Zero(N,n_controls);    // [slack_vx, slack_track, diff_delta, diff_acc, Mtv, delta, Fm]

    this->paramFlag = true;
}

///////////////////////////////////////////////////////////////////////////////////////////////
//------------------------Callback functions--------------------------------------------------

void MPC::stateCallback(const as_msgs::CarState::ConstPtr& msg){

    // carState << msg->odom.position.x, msg->odom.position.y, msg->odom.heading, msg->odom.velocity.x, msg->odom.velocity.y + msg->odom.velocity.w*d_IMU, msg->odom.velocity.w, msg->steering, msg->odom.acceleration.x, msg->Mtv;
    carState << msg->odom.position.x, msg->odom.position.y, msg->odom.heading, msg->odom.velocity.x, msg->odom.velocity.y, msg->odom.velocity.w, msg->steering, msg->odom.acceleration.x, 0.0;
    // carState << msg->odom.position.x, msg->odom.position.y, msg->odom.heading, msg->odom.velocity.x, msg->odom.velocity.y + msg->odom.velocity.w*d_IMU, msg->odom.velocity.w, 0.0, msg->odom.acceleration.x;

    stateFlag = true;

}


void MPC::plannerCallback(const as_msgs::ObjectiveArrayCurv::ConstPtr& msg){

    if(!troActive){
        if (msg->objectives.size() < nPlanning){
                ROS_WARN("MPC: Planner is too short!");
                return;
        }

        int size = min(int(msg->objectives.size()), nPlanning);

        // Fill planner matrix
        for (unsigned int i = 0; i < size ; i++){	
                planner(i, 0) = msg->objectives[i].x;
                planner(i, 1) = msg->objectives[i].y;
                planner(i, 2) = msg->objectives[i].s; 
                planner(i, 3) = msg->objectives[i].k; 
                planner(i, 4) = msg->objectives[i].vx;
                planner(i, 5) = msg->objectives[i].L;
                planner(i, 6) = msg->objectives[i].R;
        }

        rk4_s = min(max(size*delta_s, Dist_min), Dist_max)/N;
        ROS_WARN_STREAM("Delta_s: " << rk4_s);
        ROS_WARN_STREAM("size*delta_s: " << size*delta_s);

        smax = 2000;
        plannerFlag = true;
    }
}

void MPC::troCallback(const as_msgs::ObjectiveArrayCurv::ConstPtr& msg){

    if (msg->objectives.size() < nPlanning){
			ROS_WARN("MPC: TRO is too short!");
			return;
    }

    int size = min(int(msg->objectives.size()), nPlanning);

    // Fill planner matrix
    for (unsigned int i = 0; i < size ; i++){	
            planner(i, 0) = msg->objectives[i].x;
            planner(i, 1) = msg->objectives[i].y;
            planner(i, 2) = msg->objectives[i].s; 
            planner(i, 3) = msg->objectives[i].k; 
            planner(i, 4) = msg->objectives[i].vx;
            planner(i, 5) = msg->objectives[i].L;
            planner(i, 6) = msg->objectives[i].R;
    }

    rk4_s = min(max(size*delta_s, Dist_min), Dist_max)/N;
    ROS_WARN_STREAM("Delta_s: " << rk4_s);

    smax = msg->smax;
    cout << "SMAX: " << smax << endl;

    plannerFlag = troActive = true;
    ROS_WARN_ONCE("MPC: FOLLOWING TRO! :)");

}


///////////////////////////////////////////////////////////////////////////////////////////////
//------------------------Principal functions--------------------------------------------------

void MPC::solve(){

    if(paramFlag && dynParamFlag && plannerFlag && stateFlag){
        
        auto start_time = chrono::system_clock::now();

        // Set number of internal threads
        forces.params.num_of_threads = this->Nthreads;

        initial_conditions();
        set_params_bounds();

        /* Get i-th memory buffer */
        int i = 0;
        forces.mem_handle = TailoredSolver_internal_mem(i);
        /* Note: number of available memory buffers is controlled by code option max_num_mem */

        // Solve
        forces.exit_flag = TailoredSolver_solve(&forces.params, &forces.solution, &forces.info, forces.mem_handle, NULL, forces.ext_func);

        ROS_ERROR_STREAM("MPC exit flag: " << forces.exit_flag);
        ROS_ERROR_STREAM("MPC solve time: " << forces.info.solvetime*1000 << " ms");
        ROS_ERROR_STREAM("MPC iterations: " << forces.info.it);

        get_solution();
        t_prediction();

        if(forces.exit_flag == 1){

            // Update flags
            this->firstIter = false;

            // if(optCount > 100) this->firstOpt = true;
            // else optCount++;  

        }else this->firstIter = true;

        this->running = true; // MPC is running so we can start publishing commands

        auto finish_time = chrono::system_clock::now();

        elapsed_time = finish_time - start_time;

        ROS_WARN("TAILORED MPC elapsed time: %f ms", elapsed_time.count()*1000);

        // Save data
        // save<float>("/home/fetty/Desktop/control_ws2022/src/control/tailored_mpc/debug/", "solve_time.txt", elapsed_time.count()*1000, true);
        // save<int>("/home/fetty/Desktop/control_ws2022/src/control/tailored_mpc/debug/", "exit_flags.txt", forces.exit_flag, true);

        
    }else{

        if(!paramFlag || !dynParamFlag){
            ROS_ERROR("MPC: Parameters aren't properly defined");
            ROS_ERROR_STREAM("Static params: " << paramFlag);
            ROS_ERROR_STREAM("Dyn params: " << dynParamFlag);
        }else{
            ROS_ERROR("MPC: No data from state car or planner");
            ROS_ERROR_STREAM("Planner status: " << plannerFlag);
            ROS_ERROR_STREAM("State status: " << stateFlag);
        }
    }
}

void MPC::initial_conditions(){

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

    double n0 = -s0_to_car.norm()*cos(angle)*sign; // normal distance from car to track
    double t_angle = atan2(tangent(1), tangent(0)); // heading of the trajectory
    double mu0 = -(t_angle - carState(2));

    ROS_WARN_STREAM("mu0: " << mu0);
    ROS_WARN_STREAM("n0: " << n0);

    // xinit = [diff_delta, diff_Fm, Mtv, delta, Fm, n, mu, Vx, Vy, w]
    forces.params.xinit[0] = 0.0;
    forces.params.xinit[1] = 0.0;
    forces.params.xinit[2] = carState(8);
    forces.params.xinit[3] = carState(6);
    forces.params.xinit[4] = ax_to_throttle(carState(7));
    forces.params.xinit[5] = n0;
    forces.params.xinit[6] = mu0;
    forces.params.xinit[7] = carState(3);
    forces.params.xinit[8] = carState(4);
    forces.params.xinit[9] = carState(5);

    cout << "Xinit:\n";
    for(int i=0;i<10;i++){
        cout << forces.params.xinit[i] << endl;
    }

    cout << "Steering feedback: " << carState(6) << endl;

}

void MPC::set_params_bounds(){

    int size = sizeof(forces.params.hu)/sizeof(forces.params.hu[0]);
    int nh = int(size/this->N);
    int Nvar = this->n_states + this->n_controls;

    cout << "nh: " << nh << endl;
    cout << "Nvar: " << Nvar << endl;
    
    int plannerIdx = 0;
    for(int k = 0; k < this->N; k++){

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
        this->forces.params.all_parameters[19 + k*this->Npar] = (k == N-1) ? this->q_nN : this->q_n;
        this->forces.params.all_parameters[20 + k*this->Npar] = this->q_mu;
        this->forces.params.all_parameters[21 + k*this->Npar] = this->lambda;
        this->forces.params.all_parameters[22 + k*this->Npar] = (k == N-1) ? this->q_sN : this->q_s;
        this->forces.params.all_parameters[23 + k*this->Npar] = this->ax_max; 
        this->forces.params.all_parameters[24 + k*this->Npar] = this->ay_max;
        this->forces.params.all_parameters[25 + k*this->Npar] = this->Cm; 
        this->forces.params.all_parameters[26 + k*this->Npar] = this->dMtv; 

        this->forces.params.all_parameters[27 + k*this->Npar] = this->rk4_s; 
        this->forces.params.all_parameters[28 + k*this->Npar] = this->q_slack_vx; 

        plannerIdx = int(rk4_s/delta_s)*k;

        this->forces.params.all_parameters[29 + k*this->Npar] = planner(plannerIdx, 4); // planner velocity profile
        this->forces.params.all_parameters[30 + k*this->Npar] = this->q_slack_track;
        this->forces.params.all_parameters[31 + k*this->Npar] = planner(plannerIdx, 3); // curvature 

        // Inequality constraints bounds:
        this->forces.params.hu[k*nh]     = fabs(planner(plannerIdx, 5)); // L(s) ortogonal left dist from the path to the track limits
        this->forces.params.hu[k*nh + 1] = fabs(planner(plannerIdx, 6)); // R(s) ortogonal right dist from the path to the track limits
        this->forces.params.hu[k*nh + 2] = this->lambda; // forces ellipse contraints
        this->forces.params.hu[k*nh + 3] = this->lambda;
        this->forces.params.hu[k*nh + 4] = this->bounds.x_max[4]; // v_max

        // Variables bounds:
        this->forces.params.lb[k*Nvar] =     this->bounds.u_min[0];
        this->forces.params.lb[k*Nvar + 1] = this->bounds.u_min[1];
        this->forces.params.lb[k*Nvar + 2] = this->bounds.u_min[2];
        this->forces.params.lb[k*Nvar + 3] = this->bounds.u_min[3];
        this->forces.params.lb[k*Nvar + 4] = this->bounds.u_min[4];
        this->forces.params.lb[k*Nvar + 5] = this->bounds.x_min[0];
        this->forces.params.lb[k*Nvar + 6] = this->bounds.x_min[1];
        this->forces.params.lb[k*Nvar + 7] = this->bounds.x_min[2];
        this->forces.params.lb[k*Nvar + 8] = this->bounds.x_min[3];
        this->forces.params.lb[k*Nvar + 9] = this->bounds.x_min[4];
        this->forces.params.lb[k*Nvar + 10]= this->bounds.x_min[5];
        this->forces.params.lb[k*Nvar + 11]= this->bounds.x_min[6];

            // Slack variables doesn't have any upper bounds, so Nvar-2 is used here
        this->forces.params.ub[k*(Nvar-2)] =     this->bounds.u_max[0];
        this->forces.params.ub[k*(Nvar-2) + 1] = this->bounds.u_max[1];
        this->forces.params.ub[k*(Nvar-2) + 2] = this->bounds.u_max[2];
        this->forces.params.ub[k*(Nvar-2) + 3] = this->bounds.x_max[0];
        this->forces.params.ub[k*(Nvar-2) + 4] = this->bounds.x_max[1];
        this->forces.params.ub[k*(Nvar-2) + 5] = this->bounds.x_max[2];
        this->forces.params.ub[k*(Nvar-2) + 6] = this->bounds.x_max[3];
        if(troActive && troProfile){
            this->forces.params.ub[k*(Nvar-2) + 7] = planner(plannerIdx,4);
        }else{
            this->forces.params.ub[k*(Nvar-2) + 7] = (k == N-1) ? this->bounds.vx_N : this->bounds.x_max[4];
        }
        this->forces.params.ub[k*(Nvar-2) + 8] = this->bounds.x_max[5];
        this->forces.params.ub[k*(Nvar-2) + 9] = this->bounds.x_max[6];


        if(!firstIter){
            if((u_idx+k) < this->N - 1){
                if(this->forces.exit_flag == 1){
                    this->forces.params.x0[k*Nvar]     = 0.0;
                    this->forces.params.x0[k*Nvar + 1] = 0.0;
                    this->forces.params.x0[k*Nvar + 2] = solCommands(u_idx+k, 2);
                    this->forces.params.x0[k*Nvar + 3] = solCommands(u_idx+k, 3);
                    this->forces.params.x0[k*Nvar + 4] = solCommands(u_idx+k, 4);
                    this->forces.params.x0[k*Nvar + 5] = solCommands(u_idx+k, 5);
                    this->forces.params.x0[k*Nvar + 6] = solCommands(u_idx+k, 6);
                    this->forces.params.x0[k*Nvar + 7] = solStates(u_idx+k, 0);
                    this->forces.params.x0[k*Nvar + 8] = solStates(u_idx+k, 1);
                    this->forces.params.x0[k*Nvar + 9] = solStates(u_idx+k, 2);
                    this->forces.params.x0[k*Nvar + 10]= solStates(u_idx+k, 3);
                    this->forces.params.x0[k*Nvar + 11]= solStates(u_idx+k, 4);
                }else{
                    this->forces.params.x0[k*Nvar]     =  this->forces.params.lb[k*Nvar];
                    this->forces.params.x0[k*Nvar + 1] =  this->forces.params.lb[k*Nvar + 1];
                    this->forces.params.x0[k*Nvar + 2] = (this->forces.params.lb[k*Nvar + 2] + this->forces.params.ub[k*(Nvar-2)])/2;
                    this->forces.params.x0[k*Nvar + 3] = (this->forces.params.lb[k*Nvar + 3] + this->forces.params.ub[k*(Nvar-2) + 1])/2;
                    this->forces.params.x0[k*Nvar + 4] = (this->forces.params.lb[k*Nvar + 4] + this->forces.params.ub[k*(Nvar-2) + 2])/2;
                    this->forces.params.x0[k*Nvar + 5] = (this->forces.params.lb[k*Nvar + 5] + this->forces.params.ub[k*(Nvar-2) + 3])/2;
                    this->forces.params.x0[k*Nvar + 6] = (this->forces.params.lb[k*Nvar + 6] + this->forces.params.ub[k*(Nvar-2) + 4])/2;
                    this->forces.params.x0[k*Nvar + 7] = (this->forces.params.lb[k*Nvar + 7] + this->forces.params.ub[k*(Nvar-2) + 5])/2;
                    this->forces.params.x0[k*Nvar + 8] = (this->forces.params.lb[k*Nvar + 8] + this->forces.params.ub[k*(Nvar-2) + 6])/2;
                    this->forces.params.x0[k*Nvar + 9] = (this->forces.params.lb[k*Nvar + 9] + this->forces.params.ub[k*(Nvar-2) + 7])/2;
                    this->forces.params.x0[k*Nvar + 10]= (this->forces.params.lb[k*Nvar + 10] + this->forces.params.ub[k*(Nvar-2) + 8])/2;
                    this->forces.params.x0[k*Nvar + 11]= (this->forces.params.lb[k*Nvar + 11] + this->forces.params.ub[k*(Nvar-2) + 9])/2;
                }
            }else{
                if(this->forces.exit_flag == 1){
                    this->forces.params.x0[k*Nvar]     =  0.0;
                    this->forces.params.x0[k*Nvar + 1] =  0.0;
                    this->forces.params.x0[k*Nvar + 2] = solCommands(N-1, 2);
                    this->forces.params.x0[k*Nvar + 3] = solCommands(N-1, 3);
                    this->forces.params.x0[k*Nvar + 4] = solCommands(N-1, 4);
                    this->forces.params.x0[k*Nvar + 5] = solCommands(N-1, 5);
                    this->forces.params.x0[k*Nvar + 6] = solCommands(N-1, 6);
                    this->forces.params.x0[k*Nvar + 7] = solStates(N-1, 0);
                    this->forces.params.x0[k*Nvar + 8] = solStates(N-1, 1);
                    this->forces.params.x0[k*Nvar + 9] = solStates(N-1, 2);
                    this->forces.params.x0[k*Nvar + 10]= solStates(N-1, 3);
                    this->forces.params.x0[k*Nvar + 11]= solStates(N-1, 4);
                }else{
                    this->forces.params.x0[k*Nvar]     =  this->forces.params.lb[k*Nvar];
                    this->forces.params.x0[k*Nvar + 1] =  this->forces.params.lb[k*Nvar + 1];
                    this->forces.params.x0[k*Nvar + 2] = (this->forces.params.lb[k*Nvar + 2] + this->forces.params.ub[k*(Nvar-2)])/2;
                    this->forces.params.x0[k*Nvar + 3] = (this->forces.params.lb[k*Nvar + 3] + this->forces.params.ub[k*(Nvar-2) + 1])/2;
                    this->forces.params.x0[k*Nvar + 4] = (this->forces.params.lb[k*Nvar + 4] + this->forces.params.ub[k*(Nvar-2) + 2])/2;
                    this->forces.params.x0[k*Nvar + 5] = (this->forces.params.lb[k*Nvar + 5] + this->forces.params.ub[k*(Nvar-2) + 3])/2;
                    this->forces.params.x0[k*Nvar + 6] = (this->forces.params.lb[k*Nvar + 6] + this->forces.params.ub[k*(Nvar-2) + 4])/2;
                    this->forces.params.x0[k*Nvar + 7] = (this->forces.params.lb[k*Nvar + 7] + this->forces.params.ub[k*(Nvar-2) + 5])/2;
                    this->forces.params.x0[k*Nvar + 8] = (this->forces.params.lb[k*Nvar + 8] + this->forces.params.ub[k*(Nvar-2) + 6])/2;
                    this->forces.params.x0[k*Nvar + 9] = (this->forces.params.lb[k*Nvar + 9] + this->forces.params.ub[k*(Nvar-2) + 7])/2;
                    this->forces.params.x0[k*Nvar + 10]= (this->forces.params.lb[k*Nvar + 10] + this->forces.params.ub[k*(Nvar-2) + 8])/2;
                    this->forces.params.x0[k*Nvar + 11]= (this->forces.params.lb[k*Nvar + 11] + this->forces.params.ub[k*(Nvar-2) + 9])/2;
                }
            }
        }else{

            if(k==0){
                this->forces.params.x0[k*Nvar]     = 0;
                this->forces.params.x0[k*Nvar + 1] = 0;
                this->forces.params.x0[k*Nvar + 2] = 0;
                this->forces.params.x0[k*Nvar + 3] = 0;
                this->forces.params.x0[k*Nvar + 4] = carState(8);
                this->forces.params.x0[k*Nvar + 5] = carState(6);
                this->forces.params.x0[k*Nvar + 6] = ax_to_throttle(carState(7));
                this->forces.params.x0[k*Nvar + 7] = forces.params.xinit[5];
                this->forces.params.x0[k*Nvar + 8] = forces.params.xinit[6];
                this->forces.params.x0[k*Nvar + 9] = carState(3);
                this->forces.params.x0[k*Nvar + 10]= carState(4);
                this->forces.params.x0[k*Nvar + 11]= carState(5);

            }else{
                this->forces.params.x0[k*Nvar]     =  this->forces.params.lb[k*Nvar];
                this->forces.params.x0[k*Nvar + 1] =  this->forces.params.lb[k*Nvar + 1];
                this->forces.params.x0[k*Nvar + 2] = (this->forces.params.lb[k*Nvar + 2] + this->forces.params.ub[k*(Nvar-2)])/2;
                this->forces.params.x0[k*Nvar + 3] = (this->forces.params.lb[k*Nvar + 3] + this->forces.params.ub[k*(Nvar-2) + 1])/2;
                this->forces.params.x0[k*Nvar + 4] = (this->forces.params.lb[k*Nvar + 4] + this->forces.params.ub[k*(Nvar-2) + 2])/2;
                this->forces.params.x0[k*Nvar + 5] = (this->forces.params.lb[k*Nvar + 5] + this->forces.params.ub[k*(Nvar-2) + 3])/2;
                this->forces.params.x0[k*Nvar + 6] = (this->forces.params.lb[k*Nvar + 6] + this->forces.params.ub[k*(Nvar-2) + 4])/2;
                this->forces.params.x0[k*Nvar + 7] = (this->forces.params.lb[k*Nvar + 7] + this->forces.params.ub[k*(Nvar-2) + 5])/2;
                this->forces.params.x0[k*Nvar + 8] = (this->forces.params.lb[k*Nvar + 8] + this->forces.params.ub[k*(Nvar-2) + 6])/2;
                this->forces.params.x0[k*Nvar + 9] = (this->forces.params.lb[k*Nvar + 9] + this->forces.params.ub[k*(Nvar-2) + 7])/2;
                this->forces.params.x0[k*Nvar + 10]= (this->forces.params.lb[k*Nvar + 10] + this->forces.params.ub[k*(Nvar-2) + 8])/2;
                this->forces.params.x0[k*Nvar + 11]= (this->forces.params.lb[k*Nvar + 11] + this->forces.params.ub[k*(Nvar-2) + 9])/2;

            }
        }
    }

    ROS_WARN_STREAM("Max actual dist: " << plannerIdx*delta_s);

}

void MPC::t_prediction(){

    predicted_t.setZero();

    lastState(0,0) = carState(0);
    lastState(0,1) = carState(1);
    lastState(0,2) = carState(2);
    lastState(0,3) = solStates(0,2);
    lastState(0,4) = solStates(0,3);
    lastState(0,5) = solStates(0,4);

    double diff_x = 0;
    double diff_y = 0;

    for(int i=1; i < N; i++){

        double theta = lastState(i-1, 2);
        double n = solStates(i-1, 0);
        double mu = solStates(i-1, 1);
        double vx = solStates(i-1, 2);
        double vy = solStates(i-1, 3);
        double w = solStates(i-1, 4);
        double k = forces.params.all_parameters[31 + (i-1)*this->Npar];

        double sdot = (vx*cos(mu) - vy*sin(mu))/(1 - n*k);

        // cout << "sdot:\n";
        // cout << sdot << endl;

        predicted_t(i) = predicted_t(i-1) + this->rk4_s/sdot;

        // Ensure sdot > 0
        if(sdot < 0){
            ROS_ERROR("MPC FATAL ERROR, negative sdot");
            this->firstIter = true;
            break;
        }

        // Predict states evolution with Euler
        lastState(i,0) = lastState(i-1,0) + (vx*cos(theta) - vy*sin(theta))*this->rk4_s/sdot;
        lastState(i,1) = lastState(i-1,1) + (vx*sin(theta) + vy*cos(theta))*this->rk4_s/sdot;
        lastState(i,2) = lastState(i-1,2) + w*this->rk4_s/sdot;
        lastState(i,3) = vx;
        lastState(i,4) = vy;
        lastState(i,5) = w;

        diff_x += (vx*cos(theta) - vy*sin(theta))*this->rk4_s/sdot;
        diff_y += (vx*sin(theta) + vy*cos(theta))*this->rk4_s/sdot;

        lastCommands(i-1,0) = solCommands(i-1,2); // diff_delta
        lastCommands(i-1,1) = solCommands(i-1,3); // diff_Fm
        lastCommands(i-1,2) = solCommands(i-1,4); // Mtv
        lastCommands(i-1,3) = solCommands(i-1,5); // Delta
        lastCommands(i-1,4) = solCommands(i-1,6); // Fm

    }

    cout << "diff_x: " << diff_x << endl;
    cout << "diff_y: " << diff_y << endl;
    cout << "diff: " << sqrt(pow(diff_x,2)+pow(diff_y,2)) << endl;

    // cout << "PREDICTED t:\n";
    // cout << predicted_t << endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////
//------------------------Auxiliar functions--------------------------------------------------

void MPC::get_solution(){

    // Change vec dimensions from x(5*N x 1) u(6*N x 1) to x(N x 5) u(N x 6)
    Eigen::MatrixXd u = output2eigen(forces.solution.U, sizeU);
    Eigen::MatrixXd x = output2eigen(forces.solution.X, sizeX);

    Eigen::Map<Eigen::MatrixXd> controlsT(u.data(), n_controls, N);
    Eigen::Map<Eigen::MatrixXd> statesT(x.data(), n_states, N);

    solStates = statesT.transpose();
    solCommands = controlsT.transpose();

    // cout << "solStates: " << endl;
    // cout << solStates << endl;

    cout << "solCommands:\n";
    cout << solCommands << endl;

    cout << "forces.solution.u: \n";
    cout << forces.solution.U[sizeU-7] << endl;
    cout << forces.solution.U[sizeU-6] << endl;
    cout << forces.solution.U[sizeU-5] << endl;
    cout << forces.solution.U[sizeU-4] << endl;
    cout << forces.solution.U[sizeU-3] << endl;
    cout << forces.solution.U[sizeU-2] << endl;
    cout << forces.solution.U[sizeU-1] << endl;

}

void MPC::msgCommands(as_msgs::CarCommands *msg){

    if(this->running){
        // Chosen time
        double t = 1/this->Hz + latency/Hz;

        u_idx = 0; // index of the chosen time within the predicted time vector
        for(int i=0; i < this->N; i++){
            if(predicted_t(i) > t) break;
            else u_idx++;
        }
        if(u_idx > N-2) u_idx = N-2;

        cout << "u_idx: " << u_idx << endl;

        msg->header.stamp = ros::Time::now();
        msg->motor      = interpolate(solCommands(u_idx, 6), solCommands(u_idx+1, 6), predicted_t(u_idx), predicted_t(u_idx+1), t, bounds.x_max[1], bounds.x_min[1]);
        msg->steering   = interpolate(solCommands(u_idx, 5), solCommands(u_idx+1, 5), predicted_t(u_idx), predicted_t(u_idx+1), t, bounds.x_max[0], bounds.x_min[0]);
        msg->Mtv        = interpolate(solCommands(u_idx, 4), solCommands(u_idx+1, 4), predicted_t(u_idx), predicted_t(u_idx+1), t, bounds.u_max[2], bounds.u_min[4]);

        cout << "steering: " << interpolate(solCommands(u_idx, 5), solCommands(u_idx+1, 5), predicted_t(u_idx), predicted_t(u_idx+1), t, bounds.x_max[0], bounds.x_min[0]) << endl;
        cout << "motor: " << interpolate(solCommands(u_idx, 6), solCommands(u_idx+1, 6), predicted_t(u_idx), predicted_t(u_idx+1), t, bounds.x_max[1], bounds.x_min[1]) << endl;
        cout << "Mtv: " << interpolate(solCommands(u_idx, 4), solCommands(u_idx+1, 4), predicted_t(u_idx), predicted_t(u_idx+1), t, bounds.u_max[2], bounds.u_min[4]) << endl;
    }

    return;
}

double MPC::interpolate(double u0, double u1, double t0, double t1, double t, double ub, double lb){
    double command = u0 + (t-t0)*(u1-u0)/(t1-t0);
    if(command>0) return min(command,ub);
    else return max(command,lb);
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

Eigen::MatrixXd MPC::output2eigen(double arr[], int size){

    Eigen::MatrixXd result(size,1);
    for(int i = 0; i < size; i++){
        result(i,0) = arr[i];
        // cout << arr[i] << endl;
    }
    return result;
}

double MPC::throttle_to_torque(double throttle){

    return throttle*this->Cm*this->Rwheel;
}

double MPC::ax_to_throttle(double ax){ 
    if(ax >= 0) return min(ax/ax_max, 1.0);
    else return max(ax/ax_max, -1.0);
}

void MPC::saveEigen(string filePath, string name, Eigen::MatrixXd data, bool erase){

    try{
        
        // Write csv
        ofstream file;
        if(erase) file.open(filePath + name, ios::out | ios::trunc);
        else file.open(filePath + name, ios::app);

        //https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
        const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");

        if (file.is_open()){
            file << data.format(CSVFormat);
            file.close(); // Remeber to close the file!
        }

        // ROS_INFO_STREAM("MPC: matrix data SAVED AT: " << this->savePath+name);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM( "MPC::saveEigen Exception was thrown: " << e.what() );
    }
}

template<typename mytype>
void MPC::save(string filePath, string name, mytype data, bool time){

    try{
        
        // Write csv
        ofstream file;
        file.open(filePath + name, ios::app);
        if(time) file << ros::Time::now() << " : " << data << endl;
        else file << data << endl;
        file.close(); // Remeber to close the file!

        // ROS_INFO_STREAM("MPC: data SAVED AT: " << this->savePath+name);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM( "MPC::save Exception was thrown: " << e.what() );
    }
}

const string MPC::currentDateTime(){ // Get current date/time, format is YYYY-MM-DD.HH:mm:ss
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
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
        this->q_nN = config.q_nN;
        this->q_mu = config.q_mu;
        this->lambda = config.lambda;
        this->q_s = config.q_s;
        this->latency = config.latency;
        this->Cm = config.Cm;
        this->dMtv = config.dMtv;
        this->ax_max = config.Ax_max;
        this->ay_max = config.Ay_max;
        this->q_sN = config.q_sN;
        this->q_slack_vx = config.q_slack_vx;
        this->q_slack_track = config.q_slack_track;
        this->Dist_min = config.Dist_min;
        this->Dist_max = config.Dist_max;

        this->bounds.u_min[2] = -config.diff_delta*M_PI/180.0;
        this->bounds.u_min[3] = -config.diff_Fm;

        this->bounds.u_max[0] = config.diff_delta*M_PI/180.0;
        this->bounds.u_max[1] = config.diff_Fm;

        this->bounds.x_max[4] = config.Vmax;
        this->bounds.x_min[4] = config.Vmin;
        this->bounds.vx_N = config.VmaxN;

        this->dynParamFlag = true;

    } catch (exception& e){
        ROS_ERROR_STREAM("MPC::RECONFIGURE DIED" << e.what());
    }

}

