#include "mpc.hh"

// Constructor
MPC::MPC(const Params& params){

    // NLOP params
    this->n_states = params.mpc.nlop.n_states;
    this->n_controls = params.mpc.nlop.n_controls;
    this->N = params.mpc.nlop.N;
    this->Npar = params.mpc.nlop.Npar;

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
    this->Rwheel = params.vehicle.Rwheel;

    this->sizeU = sizeof(forces.solution.U)/sizeof(forces.solution.U[0]); // size of control FORCES array
    this->sizeX = sizeof(forces.solution.X)/sizeof(forces.solution.X[0]); // size of states FORCES array

    sizeCommands = int(sizeU/N);    // number of commands variables
    sizeStates = int(sizeX/N);      // number of car state variables

    planner = Eigen::MatrixXd::Zero(nPlanning,7);
    carState = Eigen::VectorXd::Zero(9);
    predicted_s = Eigen::VectorXd::Zero(N);
    progress = Eigen::VectorXd::Zero(N); 

    lastState = Eigen::MatrixXd::Zero(N,6);                 // [x, y, theta, vx, vy, w]
    lastCommands = Eigen::MatrixXd::Zero(N,sizeCommands);   // [diff_delta, diff_Fm, Mtv, delta, acc]

    solStates = Eigen::MatrixXd::Zero(N,sizeStates);        // [n, mu, vx, vy, w]
    solCommands = Eigen::MatrixXd::Zero(N,sizeCommands);    // [diff_delta, diff_acc, MTV, delta, Fm]


    this->paramFlag = true;

}

///////////////////////////////////////////////////////////////////////////////////////////////
//------------------------Callback functions--------------------------------------------------

void MPC::stateCallback(const as_msgs::CarState::ConstPtr& msg){

    // carState << msg->odom.position.x, msg->odom.position.y, msg->odom.heading, msg->odom.velocity.x, msg->odom.velocity.y + msg->odom.velocity.w*d_IMU, msg->odom.velocity.w, msg->steering, msg->odom.acceleration.x, msg->Mtv;
    carState << msg->odom.position.x, msg->odom.position.y, msg->odom.heading, msg->odom.velocity.x, msg->odom.velocity.y + msg->odom.velocity.w*d_IMU, msg->odom.velocity.w, msg->steering, msg->odom.acceleration.x, 0.0;
    // carState << msg->odom.position.x, msg->odom.position.y, msg->odom.heading, msg->odom.velocity.x, msg->odom.velocity.y + msg->odom.velocity.w*d_IMU, msg->odom.velocity.w, 0.0, msg->odom.acceleration.x;

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

void MPC::troCallback(const as_msgs::ObjectiveArrayCurv::ConstPtr& msg){

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

    smax = msg->smax;
    cout << "SMAX: " << smax << endl;
    plannerFlag = true;

}


///////////////////////////////////////////////////////////////////////////////////////////////
//------------------------Principal functions--------------------------------------------------

void MPC::solve(){

    if(paramFlag && dynParamFlag && plannerFlag && stateFlag){
        
        auto start_time = chrono::system_clock::now();

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

        if(forces.exit_flag == 1) this->firstIter = false;
        else this->firstIter = true;

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

///////////////////////////////////////////////////////////////////////////////////////////////
//------------------------Auxiliar functions--------------------------------------------------

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

    // ROS_WARN_STREAM("mu0: " << mu0);
    // ROS_WARN_STREAM("n0: " << n0);

    // xinit = [diff_delta, diff_Fm, Mtv, delta, Fm, n, mu, Vx, Vy, w]
    forces.params.xinit[0] = 0.0;
    forces.params.xinit[1] = 0.0;
    forces.params.xinit[2] = carState(8);
    forces.params.xinit[3] = 0.0; //carState(6);
    forces.params.xinit[4] = 0.0; //ax_to_throttle(carState(7));
    forces.params.xinit[5] = n0;
    forces.params.xinit[6] = mu0;
    forces.params.xinit[7] = carState(3);
    forces.params.xinit[8] = carState(4);
    forces.params.xinit[9] = carState(5);

    cout << "Xinit:\n";
    for(int i;i<10;i++){
        cout << forces.params.xinit[i] << endl;
    }

    cout << "Steering feedback: " << carState(6) << endl;

}

void MPC::set_params_bounds(){

    int id_k = 0;       // curvature's index
    int id_sinit = 0;   // initial s index
    double mean_s = 0;  // mean of s discretization (delta_s)

    int size = sizeof(forces.params.hu)/sizeof(forces.params.hu[0]);
    int nh = int(size/this->N);
    int Nvar = this->n_states + this->n_controls;

    cout << "nh: " << nh << endl;
    cout << "Nvar: " << Nvar << endl;
    
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
        this->forces.params.all_parameters[19 + k*this->Npar] = this->q_n;
        this->forces.params.all_parameters[20 + k*this->Npar] = this->q_mu;
        this->forces.params.all_parameters[21 + k*this->Npar] = this->lambda;
        this->forces.params.all_parameters[22 + k*this->Npar] = this->q_s;
        this->forces.params.all_parameters[23 + k*this->Npar] = 7; // CHANGE!! get parameters for maximum acc (x,y)
        this->forces.params.all_parameters[24 + k*this->Npar] = 10;
        this->forces.params.all_parameters[25 + k*this->Npar] = this->Cm; 
        this->forces.params.all_parameters[26 + k*this->Npar] = this->dMtv; 

        int plannerIdx = 0;
        if(firstIter){
            plannerIdx = this->samplingS*k;
        }else{
            if(id_sinit < this->N){
                
                progress(k) = predicted_s(id_sinit);

                // Set k(s), L(s), R(s) with the prediction from last MPC iteration 
                double diff_s = progress(k) - this->planner(0, 2);
                id_k = int(round(diff_s/this->delta_s));

                if(diff_s < 0) id_k = 0;

                id_sinit++;
                plannerIdx = id_k;

                // Average of last 5 delta_s 
                // TO DO: 
                //      -make longer
                if(k != 0 && id_sinit > this->N - 5){

                    diff_s = progress(k) - progress(k-1);

                    if(diff_s < 0) diff_s += this->smax; // If we are passing the start line, reset diff
                    mean_s += diff_s;
                }

                if(id_sinit == this->N) {
                    mean_s = round(mean_s/5/delta_s);
                }

            }else{
                // Set k(s), L(s), R(s) with planner actual discretization
                id_k += int(mean_s);
                plannerIdx = id_k;
            }
        }

        progress(k) = planner(plannerIdx, 2);

        cout << "PLANNER INDEX: " << plannerIdx << endl;

        this->forces.params.all_parameters[27 + k*this->Npar] = planner(plannerIdx, 3); // curvature 
        // cout << "Curvature: " << this->forces.params.all_parameters[27 + k*this->Npar] << endl;

        // Inequality constraints bounds:
        // this->forces.params.hu[k*nh] =     this->lambda;
        // this->forces.params.hu[k*nh + 1] = this->lambda;
        // this->forces.params.hu[k*nh]     = 0.0;
        // this->forces.params.hu[k*nh + 1] = 0.0;
        this->forces.params.hu[k*nh + 1] = fabs(planner(plannerIdx, 5)); // L(s) ortogonal left dist from the path to the track limits
        this->forces.params.hu[k*nh + 2] = fabs(planner(plannerIdx, 6)); // R(s) ortogonal right dist from the path to the track limits


        // Equality constraints bounds:
        this->forces.params.lb[k*Nvar] =     this->bounds.u_min[0];
        this->forces.params.lb[k*Nvar + 1] = this->bounds.u_min[1];
        this->forces.params.lb[k*Nvar + 2] = this->bounds.u_min[2];
        this->forces.params.lb[k*Nvar + 3] = this->bounds.x_min[0];
        this->forces.params.lb[k*Nvar + 4] = this->bounds.x_min[1];
        this->forces.params.lb[k*Nvar + 5] = this->bounds.x_min[2];
        this->forces.params.lb[k*Nvar + 6] = this->bounds.x_min[3];
        this->forces.params.lb[k*Nvar + 7] = this->bounds.x_min[4];
        this->forces.params.lb[k*Nvar + 8] = this->bounds.x_min[5];
        this->forces.params.lb[k*Nvar + 9] = this->bounds.x_min[6];


        this->forces.params.ub[k*Nvar] =     this->bounds.u_max[0];
        this->forces.params.ub[k*Nvar + 1] = this->bounds.u_max[1];
        this->forces.params.ub[k*Nvar + 2] = this->bounds.u_max[2];
        this->forces.params.ub[k*Nvar + 3] = this->bounds.x_max[0];
        this->forces.params.ub[k*Nvar + 4] = this->bounds.x_max[1];
        this->forces.params.ub[k*Nvar + 5] = this->bounds.x_max[2];
        this->forces.params.ub[k*Nvar + 6] = this->bounds.x_max[3];
        this->forces.params.ub[k*Nvar + 7] = this->bounds.x_max[4];
        this->forces.params.ub[k*Nvar + 8] = this->bounds.x_max[5];
        this->forces.params.ub[k*Nvar + 9] = this->bounds.x_max[6];


        if(!firstIter){
            if((latency + k) < this->N - 1){
                if(this->forces.exit_flag == 1){
                    this->forces.params.x0[k*Nvar] =     solCommands(latency+k, 0);
                    this->forces.params.x0[k*Nvar + 1] = solCommands(latency+k, 1);
                    this->forces.params.x0[k*Nvar + 2] = solCommands(latency+k, 2);
                    this->forces.params.x0[k*Nvar + 3] = solCommands(latency+k, 3);
                    this->forces.params.x0[k*Nvar + 4] = solCommands(latency+k, 4);
                    this->forces.params.x0[k*Nvar + 5] = solStates(latency+k, 0);
                    this->forces.params.x0[k*Nvar + 6] = solStates(latency+k, 1);
                    this->forces.params.x0[k*Nvar + 7] = solStates(latency+k, 2);
                    this->forces.params.x0[k*Nvar + 8] = solStates(latency+k, 3);
                    this->forces.params.x0[k*Nvar + 9] = solStates(latency+k, 4);
                }else{
                    this->forces.params.x0[k*Nvar] =     (this->forces.params.lb[k*Nvar] + this->forces.params.ub[k*Nvar])/2;
                    this->forces.params.x0[k*Nvar + 1] = (this->forces.params.lb[k*Nvar + 1] + this->forces.params.ub[k*Nvar + 1])/2;
                    this->forces.params.x0[k*Nvar + 2] = (this->forces.params.lb[k*Nvar + 2] + this->forces.params.ub[k*Nvar + 2])/2;
                    this->forces.params.x0[k*Nvar + 3] = (this->forces.params.lb[k*Nvar + 3] + this->forces.params.ub[k*Nvar + 3])/2;
                    this->forces.params.x0[k*Nvar + 4] = (this->forces.params.lb[k*Nvar + 4] + this->forces.params.ub[k*Nvar + 4])/2;
                    this->forces.params.x0[k*Nvar + 5] = (this->forces.params.lb[k*Nvar + 5] + this->forces.params.ub[k*Nvar + 5])/2;
                    this->forces.params.x0[k*Nvar + 6] = (this->forces.params.lb[k*Nvar + 6] + this->forces.params.ub[k*Nvar + 6])/2;
                    this->forces.params.x0[k*Nvar + 7] = (this->forces.params.lb[k*Nvar + 7] + this->forces.params.ub[k*Nvar + 7])/2;
                    this->forces.params.x0[k*Nvar + 8] = (this->forces.params.lb[k*Nvar + 8] + this->forces.params.ub[k*Nvar + 8])/2;
                    this->forces.params.x0[k*Nvar + 9] = (this->forces.params.lb[k*Nvar + 9] + this->forces.params.ub[k*Nvar + 9])/2;
                }
            }else{
                if(this->forces.exit_flag == 1){
                    this->forces.params.x0[k*Nvar] =     solCommands(N-1, 0);
                    this->forces.params.x0[k*Nvar + 1] = solCommands(N-1, 1);
                    this->forces.params.x0[k*Nvar + 2] = solCommands(N-1, 2);
                    this->forces.params.x0[k*Nvar + 3] = solCommands(N-1, 3);
                    this->forces.params.x0[k*Nvar + 4] = solCommands(N-1, 4);
                    this->forces.params.x0[k*Nvar + 5] = solStates(N-1, 0);
                    this->forces.params.x0[k*Nvar + 6] = solStates(N-1, 1);
                    this->forces.params.x0[k*Nvar + 7] = solStates(N-1, 2);
                    this->forces.params.x0[k*Nvar + 8] = solStates(N-1, 3);
                    this->forces.params.x0[k*Nvar + 9] = solStates(N-1, 4);
                }else{
                    this->forces.params.x0[k*Nvar] =     (this->forces.params.lb[k*Nvar] + this->forces.params.ub[k*Nvar])/2;
                    this->forces.params.x0[k*Nvar + 1] = (this->forces.params.lb[k*Nvar + 1] + this->forces.params.ub[k*Nvar + 1])/2;
                    this->forces.params.x0[k*Nvar + 2] = (this->forces.params.lb[k*Nvar + 2] + this->forces.params.ub[k*Nvar + 2])/2;
                    this->forces.params.x0[k*Nvar + 3] = (this->forces.params.lb[k*Nvar + 3] + this->forces.params.ub[k*Nvar + 3])/2;
                    this->forces.params.x0[k*Nvar + 4] = (this->forces.params.lb[k*Nvar + 4] + this->forces.params.ub[k*Nvar + 4])/2;
                    this->forces.params.x0[k*Nvar + 5] = (this->forces.params.lb[k*Nvar + 5] + this->forces.params.ub[k*Nvar + 5])/2;
                    this->forces.params.x0[k*Nvar + 6] = (this->forces.params.lb[k*Nvar + 6] + this->forces.params.ub[k*Nvar + 6])/2;
                    this->forces.params.x0[k*Nvar + 7] = (this->forces.params.lb[k*Nvar + 7] + this->forces.params.ub[k*Nvar + 7])/2;
                    this->forces.params.x0[k*Nvar + 8] = (this->forces.params.lb[k*Nvar + 8] + this->forces.params.ub[k*Nvar + 8])/2;
                    this->forces.params.x0[k*Nvar + 9] = (this->forces.params.lb[k*Nvar + 9] + this->forces.params.ub[k*Nvar + 9])/2;
                }
            }
        }else{

            if(k==0){
                this->forces.params.x0[k*Nvar] =     0;
                this->forces.params.x0[k*Nvar + 1] = 0;
                this->forces.params.x0[k*Nvar + 2] = carState(8);
                this->forces.params.x0[k*Nvar + 3] = carState(6);
                this->forces.params.x0[k*Nvar + 4] = ax_to_throttle(carState(7));
                this->forces.params.x0[k*Nvar + 5] = forces.params.xinit[5];
                this->forces.params.x0[k*Nvar + 6] = forces.params.xinit[6];
                this->forces.params.x0[k*Nvar + 7] = carState(3);
                this->forces.params.x0[k*Nvar + 8] = carState(4);
                this->forces.params.x0[k*Nvar + 9] = carState(5);

                // this->forces.params.x0[k*Nvar] =     (this->forces.params.lb[k*Nvar] + this->forces.params.ub[k*Nvar])/2;
                // this->forces.params.x0[k*Nvar + 1] = (this->forces.params.lb[k*Nvar + 1] + this->forces.params.ub[k*Nvar + 1])/2;
                // this->forces.params.x0[k*Nvar + 2] = (this->forces.params.lb[k*Nvar + 2] + this->forces.params.ub[k*Nvar + 2])/2;
                // this->forces.params.x0[k*Nvar + 3] = (this->forces.params.lb[k*Nvar + 3] + this->forces.params.ub[k*Nvar + 3])/2;
                // this->forces.params.x0[k*Nvar + 4] = (this->forces.params.lb[k*Nvar + 4] + this->forces.params.ub[k*Nvar + 4])/2;
                // this->forces.params.x0[k*Nvar + 5] = (this->forces.params.lb[k*Nvar + 5] + this->forces.params.ub[k*Nvar + 5])/2;
                // this->forces.params.x0[k*Nvar + 6] = (this->forces.params.lb[k*Nvar + 6] + this->forces.params.ub[k*Nvar + 6])/2;
                // this->forces.params.x0[k*Nvar + 7] = (this->forces.params.lb[k*Nvar + 7] + this->forces.params.ub[k*Nvar + 7])/2;
                // this->forces.params.x0[k*Nvar + 8] = (this->forces.params.lb[k*Nvar + 8] + this->forces.params.ub[k*Nvar + 8])/2;
                // this->forces.params.x0[k*Nvar + 9] = (this->forces.params.lb[k*Nvar + 9] + this->forces.params.ub[k*Nvar + 9])/2;

            }else{
                this->forces.params.x0[k*Nvar] =     (this->forces.params.lb[k*Nvar] + this->forces.params.ub[k*Nvar])/2;
                this->forces.params.x0[k*Nvar + 1] = (this->forces.params.lb[k*Nvar + 1] + this->forces.params.ub[k*Nvar + 1])/2;
                this->forces.params.x0[k*Nvar + 2] = (this->forces.params.lb[k*Nvar + 2] + this->forces.params.ub[k*Nvar + 2])/2;
                this->forces.params.x0[k*Nvar + 3] = (this->forces.params.lb[k*Nvar + 3] + this->forces.params.ub[k*Nvar + 3])/2;
                this->forces.params.x0[k*Nvar + 4] = (this->forces.params.lb[k*Nvar + 4] + this->forces.params.ub[k*Nvar + 4])/2;
                this->forces.params.x0[k*Nvar + 5] = (this->forces.params.lb[k*Nvar + 5] + this->forces.params.ub[k*Nvar + 5])/2;
                this->forces.params.x0[k*Nvar + 6] = (this->forces.params.lb[k*Nvar + 6] + this->forces.params.ub[k*Nvar + 6])/2;
                this->forces.params.x0[k*Nvar + 7] = (this->forces.params.lb[k*Nvar + 7] + this->forces.params.ub[k*Nvar + 7])/2;
                this->forces.params.x0[k*Nvar + 8] = (this->forces.params.lb[k*Nvar + 8] + this->forces.params.ub[k*Nvar + 8])/2;
                this->forces.params.x0[k*Nvar + 9] = (this->forces.params.lb[k*Nvar + 9] + this->forces.params.ub[k*Nvar + 9])/2;

            }
        }
    }


    cout << "ALL PARAMETERS:\n";
    for(int i=0;i<27;i++){
        cout << forces.params.all_parameters[i] << endl;
    }

}

void MPC::s_prediction(){

    predicted_s.setZero();
    cout << "S_PRED inicial: " << progress(0) << endl;
    predicted_s(0) = fmod(progress(0), smax);

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
        double k = forces.params.all_parameters[27 + (i-1)*this->Npar];

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
        lastCommands(i-1,4) = solCommands(i-1,4);

    }

    // If predicted s is too small set initial s again
    double totalLength = predicted_s(this->N-1) - predicted_s(0);
    if(totalLength < 3){ 
        ROS_ERROR("Predicted s smaller than threshold!");
        firstIter = true;    
    }
}

void MPC::get_solution(){

    // Change vec dimensions from x(5*N x 1) u(4*N x 1) to x(N x 5) u(N x 4)
    Eigen::MatrixXd u = output2eigen(forces.solution.U, sizeU);
    Eigen::MatrixXd x = output2eigen(forces.solution.X, sizeX);

    Eigen::Map<Eigen::MatrixXd> controlsT(u.data(), sizeCommands, N);
    Eigen::Map<Eigen::MatrixXd> statesT(x.data(), sizeStates, N);

    solStates = statesT.transpose();
    solCommands = controlsT.transpose();

    // cout << "solStates: " << endl;
    // cout << solStates << endl;

    // cout << "solCommands:\n";
    // cout << solCommands << endl;

}

void MPC::msgCommands(as_msgs::CarCommands *msg){

    msg->header.stamp = ros::Time::now();
    msg->motor = 0.1; //solCommands(this->latency, 4); //getTorquefromThrottle(solCommands(this->latency, 4));
    msg->steering = solCommands(this->latency, 3);
    msg->Mtv = solCommands(this->latency, 2);

    cout << "steering: " << solCommands(this->latency, 3) << endl;
    cout << "motor: " << solCommands(this->latency, 4) << endl;
    cout << "Mtv: " << solCommands(this->latency, 2) << endl;
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

Eigen::MatrixXd MPC::output2eigen(double arr[], int size){

    Eigen::MatrixXd result(size,1);
    for(int i = 0; i < size; i++){
        result(i,0) = arr[i];
        // cout << arr[i] << endl;
    }
    return result;
}

double MPC::torque_to_throttle(double throttle){

    return throttle*this->Cm*this->Rwheel;
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
        this->Cm = config.Cm;
        this->dMtv = config.dMtv;

        this->bounds.u_min[0] = -config.diff_delta;
        this->bounds.u_min[1] = -config.diff_Fm;

        this->bounds.u_max[0] = config.diff_delta;
        this->bounds.u_max[1] = config.diff_Fm;

        this->dynParamFlag = true;

    } catch (exception& e){
        ROS_ERROR_STREAM("MPC RECONFIGURE DIED" << e.what());
    }

}

