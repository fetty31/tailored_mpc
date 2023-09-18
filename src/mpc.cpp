#include "mpc.hh"

// Constructor
MPC::MPC(const Params* params){

    // NLOP params
    this->N = params->mpc.nlop.N;
    this->Nslacks = params->mpc.nlop.Nslacks;

    // MPC
    this->Hz            = params->mpc.Hz;
    this->rk4_t         = params->mpc.rk4_t;
    this->delta_s       = params->mpc.delta_s;
    this->nPlanning     = params->mpc.nPlanning;
    this->nSearch       = params->mpc.nSearch;
    this->Nthreads      = params->mpc.Nthreads;

    // Vehicle params
    this->m         = params->vehicle.m;
    this->longue    = params->vehicle.longue;
    this->width     = params->vehicle.width;
    this->Lr        = params->vehicle.Lr;
    this->Lf        = params->vehicle.Lf;
    this->I         = params->vehicle.I;
    this->Ar        = params->vehicle.Ar;
    this->rho       = params->vehicle.rho;
    this->gravity   = params->vehicle.gravity;

    this->sizeU = sizeof(forces.solution.U)/sizeof(forces.solution.U[0]); // size of control FORCES array
    this->sizeX = sizeof(forces.solution.X)/sizeof(forces.solution.X[0]); // size of states FORCES array

    int sizeParam = sizeof(forces.params.all_parameters)/sizeof(forces.params.all_parameters[0]); // size of params FORCES array
    this->Npar = int(sizeParam/N);

    this->n_states = int(sizeX/N);      // number of car state variables
    this->n_controls = int(sizeU/N);    // number of commands variables    

    planner = Eigen::MatrixXd::Zero(nPlanning,7);
    carState = Eigen::VectorXd::Zero(9);
    predicted_s = Eigen::VectorXd::Zero(N);
    progress = Eigen::VectorXd::Zero(N); 
    pred_velocities = Eigen::VectorXd::Zero(nPlanning);

    lastState = Eigen::MatrixXd::Zero(N,6);                         // [x, y, theta, vx, vy, w]
    lastCommands = Eigen::MatrixXd::Zero(N,n_controls-Nslacks);     // [diff_delta, Mtv, delta]

    solStates = Eigen::MatrixXd::Zero(N,n_states);        // [n, mu, vy, w]
    solCommands = Eigen::MatrixXd::Zero(N,n_controls);    // [slack_track, diff_delta, Mtv, delta]

    this->debug_path = params->debug.path;
    this->debug_flag = params->debug.flag;
}

///////////////////////////////////////////////////////////////////////////////////////////////
//------------------------Callback functions--------------------------------------------------

void MPC::stateCallback(const as_msgs::CarState::ConstPtr& msg){

    // Update car state vector with new state
    carState << msg->odom.position.x, msg->odom.position.y, msg->odom.heading, msg->odom.velocity.x, msg->odom.velocity.y, msg->odom.velocity.w, msg->steering, msg->odom.acceleration.x, msg->Mtv;

    stateFlag = true; // we have received car state data
}


void MPC::plannerCallback(const as_msgs::ObjectiveArrayCurv::ConstPtr& msg){

    if(!troActive){ // if we are in AutoX mode (we follow planner's trajectory)
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
            planner(i, 3) = (msg->objectives[i].k == 0.0) ? 1e-7 : msg->objectives[i].k; // avoid absolut zeros
            planner(i, 4) = msg->objectives[i].vx;
            planner(i, 5) = msg->objectives[i].L;
            planner(i, 6) = msg->objectives[i].R;

            if(msg->objectives[i].s > smax) smax = msg->objectives[i].s; // save maximum progress
        }

        plannerFlag = true; // we have received planner data
    }

}

void MPC::troCallback(const as_msgs::ObjectiveArrayCurv::ConstPtr& msg){

    if (msg->objectives.size() < nPlanning){
			ROS_WARN("MPC: TRO is too short!");
			return;
    }

    // Fill planner matrix
    for (unsigned int i = 0; i < nPlanning ; i++)
    {	
        planner(i, 0) = msg->objectives[i].x;
        planner(i, 1) = msg->objectives[i].y;
        planner(i, 2) = msg->objectives[i].s; 
        planner(i, 3) = (msg->objectives[i].k == 0.0) ? 1e-7 : msg->objectives[i].k; 
        planner(i, 4) = msg->objectives[i].vx;
        planner(i, 5) = msg->objectives[i].L;
        planner(i, 6) = msg->objectives[i].R;
    }

    smax = msg->smax;
    plannerFlag = troActive = true; // we are now following TRO's trajectory
    ROS_WARN_ONCE("MPC: FOLLOWING TRO! :)");

}

void MPC::velsCallback(const as_msgs::CarVelocityArray::ConstPtr& msg){

    if (msg->velocities.size() < this->N){ 
        ROS_WARN("MPC: Velocity profile too short!");
        return;
    }

    for(int i=0; i < msg->velocities.size(); i++){
        pred_velocities(i) = msg->velocities[i].x;
    }
    this->velsFlag = true; // we have received velocity data

}


///////////////////////////////////////////////////////////////////////////////////////////////
//------------------------Principal functions--------------------------------------------------

void MPC::solve(){

    if(dynParamFlag && plannerFlag && stateFlag && velsFlag){
        
        auto start_time = chrono::system_clock::now();

        // Set number of internal threads
        // forces.params.num_of_threads = this->Nthreads;

        initial_conditions();   // compute initial state (xinit)
        set_params_bounds();    // set parameters & boundaries of the NLOP (Non Linear Optimization Problem)

        // Get i-th memory buffer
        int i = 0;
        forces.mem_handle = TailoredSolver_internal_mem(i);
        /* Note: number of available memory buffers is controlled by code option max_num_mem */

        // Solve
        forces.exit_flag = TailoredSolver_solve(&forces.params, &forces.solution, &forces.info, forces.mem_handle, NULL, forces.ext_func);

        ROS_WARN_STREAM("TAILORED MPC exit flag: " << forces.exit_flag);
        // ROS_WARN_STREAM("TAILORED MPC solve time: " << forces.info.solvetime*1000 << " ms");
        // ROS_WARN_STREAM("TAILORED MPC iterations: " << forces.info.it);

        if(forces.exit_flag == 1 || forces.exit_flag == 0) this->firstIter = false;
        else this->firstIter = true; // we go back to first iteration's pipeline if the NLOP didn't converge

        get_solution(); // save current solution (predicted states & controls)
        if(forces.exit_flag == 1 || forces.exit_flag == 0) s_prediction(); 
        /* Note: we predict the evolution of the progress (s) if optimal solution is found (exit flag == 1) or maximum number of iterations is reached (exit flag == 0)*/

        auto finish_time = chrono::system_clock::now();

        elapsed_time = finish_time - start_time;

        // ROS_WARN("TAILORED MPC elapsed time: %f ms", elapsed_time.count()*1000);

        // Save data
        // save<float>(this->debug_path, "solve_time.txt", elapsed_time.count()*1000, true);
        // save<int>(this->debug_path, "exit_flags.csv", forces.exit_flag, true);

    }else{

        if(!dynParamFlag) ROS_ERROR("MPC: Dynamic Parameters aren't properly defined");
        else{
            ROS_ERROR("MPC: No data from state car, planner or velocity profile");
            ROS_ERROR("Planner status: %i", plannerFlag);
            ROS_ERROR("State status: %i", stateFlag);
            ROS_ERROR("Vel profile status: %i", velsFlag);
        }
    }
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

void MPC::initial_conditions(){

    // Calculate tangent vector of the trajectory
    Eigen::Vector2d tangent;
    tangent << planner(1,0) - planner(0,0), planner(1,1) - planner(0,1);

    // Calculate vector from the first point of the trajectory to the car
    Eigen::Vector2d s0_to_car;
    s0_to_car << carState(0) - planner(0,0), carState(1) - planner(0,1);

    // Calculate cross product to get angle
    double cross = s0_to_car(0)*tangent(1) - s0_to_car(1)*tangent(0);
    double angle = asin(cross/(s0_to_car.norm()*tangent.norm())); // angle between car position and heading of the trajectory

    // Get sign
    int sign = angle/fabs(angle);

    if(fabs(angle) > M_PI/2) angle = fabs(angle) - M_PI/2;
    else angle = M_PI/2 - fabs(angle);

    double n0 = -s0_to_car.norm()*cos(fabs(angle))*sign; // normal distance from car to track
    double t_angle = atan2(tangent(1), tangent(0)); // heading of the trajectory
    double mu0 = -(t_angle - continuous(carState(2), t_angle));

    // ROS_WARN_STREAM("mu0: " << mu0);
    // ROS_WARN_STREAM("n0: " << n0);

    vector<double> xinit(this->n_states); // xinit = [delta, n, mu, Vy, w]
    xinit[0] = carState(6);
    xinit[1] = n0;
    xinit[2] = mu0;
    xinit[3] = carState(4);
    xinit[4] = carState(5);

    cout << "XINIT: " << endl;
    printVec(xinit);

    // forces.params.xinit[0] = 0.0;
    // forces.params.xinit[1] = carState(8);
    // forces.params.xinit[2] = carState(6);
    // forces.params.xinit[3] = n0;
    // forces.params.xinit[4] = mu0;
    // forces.params.xinit[5] = carState(4);
    // forces.params.xinit[6] = carState(5);

    return xinit;

}

// set_boundaries_IPOPT: Set boundaries for state, control variables and equality & inequality constraints
void MPC::set_boundaries_IPOPT(){

    /* TO DO:
        - fill initial guess x0 with different values depending on previous solution
        - only execute this function once --> put outside X0 calculation/filling
    */

    vector<double> X_MIN;
    vector<double> X_MAX;
    // vector<double> X0;
    vector<double> U_MIN;
    vector<double> U_MAX;
    // vector<double> U0;

    // Reserve memory
    X_MIN.reserve(this->bounds.x_min.size()*(this->N+1));
    X_MAX.reserve(this->bounds.x_max.size()*(this->N+1));
    // X0.reserve(this->bounds.x0.size()*(this->N+1));
    U_MIN.reserve(this->bounds.u_min.size()*this->N);
    U_MAX.reserve(this->bounds.u_max.size()*this->N);
    // U0.reserve(this->bounds.u0.size()*this->N);

    // Bounds of constraints
    vector<double> lbg_next(n_states*(N+1), this->bounds.lower_continuity);  // lower boundaries continuity constraint
    vector<double> lbg_track(2*(N+1), this->bounds.lower_track);             // lower boundaries track constraint

    vector<double> ubg_next(n_states*(N+1), this->bounds.upper_continuity);  // upper boundaries continuity constraint

    vector<double> ubg_track; // upper boundaries track constraint
    for(unsigned i=0; i<N+1; i++){
        ubg_track.push_back(planner(i,5));
        ubg_track.push_back(planner(i,6));
    }

    ipopt.lbg_next = lbg_next;
    ipopt.lbg_track = lbg_track;
    ipopt.ubg_next = ubg_next;
    ipopt.ubg_track = ubg_track;

    for(int k=0; k<N+1; k++){

        X_MIN.insert(X_MIN.end(), this->bounds.x_min.begin(), this->bounds.x_min.end());
        X_MAX.insert(X_MAX.end(), this->bounds.x_max.begin(), this->bounds.x_max.end());

        // X0.insert(X0.end(), this->bounds.x0.begin(), this->bounds.x0.end());

        if(k<N){
            U_MIN.insert(U_MIN.end(), this->bounds.u_min.begin(), this->bounds.u_min.end());
            U_MAX.insert(U_MAX.end(), this->bounds.u_max.begin(), this->bounds.u_max.end());

            // U0.insert(U0.end(), this->bounds.u0.begin(), this->bounds.u0.end());
        }
    }

    ipopt.lbx = vconcat(X_MIN,U_MIN);
    ipopt.ubx = vconcat(X_MAX,U_MAX);
    // ipopt.x0 = vconcat(X0,U0);

}

void MPC::set_parameters_IPOPT(){

    /* TO DO:
        - set curvature & velocity depending on progress prediction
    */

    vector<double> param = {this->dRd, 
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
                        this->gravity, 
                        this->q_slip, 
                        this->q_n, 
                        this->q_mu, 
                        this->q_s};

    vector<double> curvature(this->N), velocity(this->N);
    for (unsigned int i = 0; i < this->N; i++){
        curvature[i] = planner(i,3);
        velocity[i] = pred_velocities(i);
    }

    vector<double> xinit = initial_conditions(); // initial conditions evaluation

    param.reserve(xinit.size()+param.size()+curvature.size()+velocity.size()); // reserve memory

    param.insert(param.end(), xinit.begin(), xinit.end()); // insert initial state vector

    param.insert(param.end(), curvature.begin(), curvature.end()); // insert midline path's curvature

    param.insert(param.end(), velocity.begin(), velocity.end()); // insert velocity profile

    ipopt.p = param;

}

void MPC::s_prediction(){

    predicted_s.setZero();
    cout << "S_PRED inicial: " << progress(0) << endl;
    predicted_s(0) = progress(0);

    lastState(0,0) = carState(0);
    lastState(0,1) = carState(1);
    lastState(0,2) = carState(2);
    lastState(0,3) = carState(3);
    lastState(0,4) = solStates(0,2);
    lastState(0,5) = solStates(0,3);

    for(int i=1; i < N; i++){

        double theta = lastState(i-1, 2);
        double n = solStates(i-1, 0);
        double mu = solStates(i-1, 1);
        double vx = ipopt.p[this->Npar +this->n_states + this->N + (i-1)]; // predicted vel. from longitudinal pid
        double vy = solStates(i-1, 2);
        double w = solStates(i-1, 3);
        double k = ipopt.p[this->Npar +this->n_states + (i-1)]; // curvature from planner

        // save<double>(this->debug_path, "curvature.csv", k, true);
        // save<double>(this->debug_path, "deviation.csv", n, true);

        double sdot = (vx*cos(mu) - vy*sin(mu))/(1 - n*k);

        predicted_s(i) = predicted_s(i-1) + sdot*this->rk4_t;
        if(predicted_s(i) > this->smax) predicted_s(i) -= this->smax; // if we have done one lap, reset progress

        // Ensure sdot > 0
        if(sdot < 0){
            ROS_ERROR("MPC FATAL ERROR, negative sdot");
            this->firstIter = true;
            break;
        }

        // Predict states evolution with Euler
        lastState(i, 0) = lastState(i-1,0) + (vx*cos(theta) - vy*sin(theta))*this->rk4_t;
        lastState(i, 1) = lastState(i-1,1) + (vx*sin(theta) + vy*cos(theta))*this->rk4_t;
        lastState(i, 2) = lastState(i-1,2) + w*this->rk4_t;
        lastState(i, 3) = vx;
        lastState(i, 4) = vy;
        lastState(i, 5) = w;

        lastCommands(i-1, 0) = solCommands(i-1, 1); // diff_delta
        lastCommands(i-1, 1) = solCommands(i-1, 2); // Mtv
        lastCommands(i-1, 2) = solCommands(i-1, 3); // Delta

    }

    // saveEigen(this->debug_path, "horizon_fss.csv", lastState.leftCols(2), false);
    // saveEigen(this->debug_path, "solCommands.csv", solCommands, true);
    // saveEigen(this->debug_path, "solStates.csv", solStates, true);

    // If predicted s is too small set initial s again
    double totalLength = predicted_s(this->N-1) - predicted_s(0);
    if(totalLength < 1.0){ 
        ROS_ERROR("Predicted s smaller than threshold!");
        firstIter = true;    
    }

}

void MPC::set_params_bounds(){

    int id_k = 0;       // curvature's index
    int id_sinit = 0;   // initial progress (s) index
    double mean_s = 0;  // mean of progress discretization (delta_s)

    int size = sizeof(forces.params.hu)/sizeof(forces.params.hu[0]);
    int nh = int(size/this->N);                    // number of inequality constraints
    int Nvar = this->n_states + this->n_controls;  // number of total variables (state + control)

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
        this->forces.params.all_parameters[ 1 + k*this->Npar] = this->m;
        this->forces.params.all_parameters[ 2 + k*this->Npar] = this->I;
        this->forces.params.all_parameters[ 3 + k*this->Npar] = this->Lf;
        this->forces.params.all_parameters[ 4 + k*this->Npar] = this->Lr;
        this->forces.params.all_parameters[ 5 + k*this->Npar] = this->Dr;
        this->forces.params.all_parameters[ 6 + k*this->Npar] = this->Df;
        this->forces.params.all_parameters[ 7 + k*this->Npar] = this->Cr;
        this->forces.params.all_parameters[ 8 + k*this->Npar] = this->Cf;
        this->forces.params.all_parameters[ 9 + k*this->Npar] = this->Br;
        this->forces.params.all_parameters[10 + k*this->Npar] = this->Bf;
        this->forces.params.all_parameters[11 + k*this->Npar] = this->u_r;   
        this->forces.params.all_parameters[12 + k*this->Npar] = this->gravity;
        this->forces.params.all_parameters[13 + k*this->Npar] = this->Cd;
        this->forces.params.all_parameters[14 + k*this->Npar] = this->rho;   
        this->forces.params.all_parameters[15 + k*this->Npar] = this->Ar;
        this->forces.params.all_parameters[16 + k*this->Npar] = this->q_slip;
        this->forces.params.all_parameters[17 + k*this->Npar] = (k == N-1) ? this->q_nN : this->q_n;
        this->forces.params.all_parameters[18 + k*this->Npar] = this->q_mu;
        this->forces.params.all_parameters[19 + k*this->Npar] = (k == N-1) ? this->q_sN : this->q_s;
        this->forces.params.all_parameters[20 + k*this->Npar] = this->dMtv; 

        this->forces.params.all_parameters[21 + k*this->Npar] = this->rk4_t;
        this->forces.params.all_parameters[22 + k*this->Npar] = this->q_slack_track;

        int plannerIdx = 0;
        if(firstIter){
            plannerIdx = this->samplingS*k; // in first iteration we pick equally spaced points from the planner

        }else{ // Set k(s), L(s), R(s), v(s) with the prediction from last MPC iteration 
            if(id_sinit < this->N){
                
                progress(k) = predicted_s(id_sinit);

                cout << "predicted_s: " << progress(k) << endl;
                cout << "planner(0, 2): " << planner(0, 2) << endl;
                cout << "smax: " << this->smax << endl;
                
                double diff_s = progress(k) - planner(0,2);
                cout << "diff_s: " << diff_s << endl;

                if(diff_s < -15.0) diff_s += this->smax; // If we are passing the start line, reset diff (here we use 15m to make sure we have actually crossed the start line)
                id_k = int(round(diff_s/this->delta_s));

                cout << "diff_s: " << diff_s << endl;
                cout << "id_k: " << id_k << endl;

                id_sinit++;
                if(diff_s > 0.0) plannerIdx = id_k; 
                /* NOTE: if -15 <= diff_s <= 0, we take the first idx of the planner (plannerIdx = 0) */

                // Average of last 5 delta_s 
                if(k != 0 && id_sinit > this->N - 5){
                    
                    diff_s = progress(k) - progress(k-1);
                    cout << "diff_s: " << diff_s << endl;
 
                    if(diff_s < -15.0) diff_s += this->smax; // If we are passing the start line, reset diff
                    // if(diff_s < 0) diff_s = 0;

                    mean_s += diff_s;
                }

                if(id_sinit == this->N) {
                    mean_s = round(mean_s/5.0/delta_s);
                    cout << "mean_s finished: " << mean_s << endl;
                }

            }else{
                // Set k(s), L(s), R(s) with planner actual discretization
                id_k += int(mean_s);
                plannerIdx = id_k;

            }
        }

        cout << "PlannerIDX: " << plannerIdx << endl;

        progress(k) = planner(plannerIdx, 2); // save current progress from planner

        this->forces.params.all_parameters[23 + k*this->Npar] = pred_velocities(plannerIdx); // velocity profile from longitudinal pid
        this->forces.params.all_parameters[24 + k*this->Npar] = planner(plannerIdx, 3);      // curvature 

        // Inequality constraints bounds:
        this->forces.params.hu[k*nh]     = fabs(planner(plannerIdx, 5)); // L(s) ortogonal left dist from the path to the track limits
        this->forces.params.hu[k*nh + 1] = fabs(planner(plannerIdx, 6)); // R(s) ortogonal right dist from the path to the track limits

        // Variables bounds:
        this->forces.params.lb[k*Nvar]     = this->bounds.u_min[0];
        this->forces.params.lb[k*Nvar + 1] = this->bounds.u_min[1];
        this->forces.params.lb[k*Nvar + 2] = this->bounds.u_min[2];
        this->forces.params.lb[k*Nvar + 3] = this->bounds.x_min[0];
        this->forces.params.lb[k*Nvar + 4] = this->bounds.x_min[1];
        this->forces.params.lb[k*Nvar + 5] = this->bounds.x_min[2];
        this->forces.params.lb[k*Nvar + 6] = this->bounds.x_min[3];
        this->forces.params.lb[k*Nvar + 7] = this->bounds.x_min[4];

            // Slack variables doesn't have any upper bounds, so Nvar-Nslacks is used here
        this->forces.params.ub[k*(Nvar-Nslacks)]     = this->bounds.u_max[0];
        this->forces.params.ub[k*(Nvar-Nslacks) + 1] = this->bounds.u_max[1];
        this->forces.params.ub[k*(Nvar-Nslacks) + 2] = this->bounds.x_max[0];
        this->forces.params.ub[k*(Nvar-Nslacks) + 3] = this->bounds.x_max[1];
        this->forces.params.ub[k*(Nvar-Nslacks) + 4] = this->bounds.x_max[2];
        this->forces.params.ub[k*(Nvar-Nslacks) + 5] = this->bounds.x_max[3];
        this->forces.params.ub[k*(Nvar-Nslacks) + 6] = this->bounds.x_max[4];

        if(!firstIter){
            if((latency + k) < this->N - 1){
                if(this->forces.exit_flag == 1){
                    this->forces.params.x0[k*Nvar]     = 0.0;
                    this->forces.params.x0[k*Nvar + 1] = solCommands(latency+k, 1);
                    this->forces.params.x0[k*Nvar + 2] = solCommands(latency+k, 2);
                    this->forces.params.x0[k*Nvar + 3] = solCommands(latency+k, 3);
                    this->forces.params.x0[k*Nvar + 4] = solStates(latency+k, 0);
                    this->forces.params.x0[k*Nvar + 5] = solStates(latency+k, 1);
                    this->forces.params.x0[k*Nvar + 6] = solStates(latency+k, 2);
                    this->forces.params.x0[k*Nvar + 7] = solStates(latency+k, 3);
                }else{
                    this->forces.params.x0[k*Nvar]     =  this->forces.params.lb[k*Nvar];
                    this->forces.params.x0[k*Nvar + 1] = (this->forces.params.lb[k*Nvar + 1] + this->forces.params.ub[k*(Nvar-Nslacks)])/2;
                    this->forces.params.x0[k*Nvar + 2] = (this->forces.params.lb[k*Nvar + 2] + this->forces.params.ub[k*(Nvar-Nslacks) + 1])/2;
                    this->forces.params.x0[k*Nvar + 3] = (this->forces.params.lb[k*Nvar + 3] + this->forces.params.ub[k*(Nvar-Nslacks) + 2])/2;
                    this->forces.params.x0[k*Nvar + 4] = (this->forces.params.lb[k*Nvar + 4] + this->forces.params.ub[k*(Nvar-Nslacks) + 3])/2;
                    this->forces.params.x0[k*Nvar + 5] = (this->forces.params.lb[k*Nvar + 5] + this->forces.params.ub[k*(Nvar-Nslacks) + 4])/2;
                    this->forces.params.x0[k*Nvar + 6] = (this->forces.params.lb[k*Nvar + 6] + this->forces.params.ub[k*(Nvar-Nslacks) + 5])/2;
                    this->forces.params.x0[k*Nvar + 7] = (this->forces.params.lb[k*Nvar + 7] + this->forces.params.ub[k*(Nvar-Nslacks) + 6])/2;
                }
            }else{
                if(this->forces.exit_flag == 1){
                    this->forces.params.x0[k*Nvar]     = 0.0;
                    this->forces.params.x0[k*Nvar + 1] = solCommands(N-1, 1);
                    this->forces.params.x0[k*Nvar + 2] = solCommands(N-1, 2);
                    this->forces.params.x0[k*Nvar + 3] = solCommands(N-1, 3);
                    this->forces.params.x0[k*Nvar + 4] = solStates(N-1, 0);
                    this->forces.params.x0[k*Nvar + 5] = solStates(N-1, 1);
                    this->forces.params.x0[k*Nvar + 6] = solStates(N-1, 2);
                    this->forces.params.x0[k*Nvar + 7] = solStates(N-1, 3);
                }else{
                    this->forces.params.x0[k*Nvar]     =  this->forces.params.lb[k*Nvar];
                    this->forces.params.x0[k*Nvar + 1] = (this->forces.params.lb[k*Nvar + 1] + this->forces.params.ub[k*(Nvar-Nslacks)])/2;
                    this->forces.params.x0[k*Nvar + 2] = (this->forces.params.lb[k*Nvar + 2] + this->forces.params.ub[k*(Nvar-Nslacks) + 1])/2;
                    this->forces.params.x0[k*Nvar + 3] = (this->forces.params.lb[k*Nvar + 3] + this->forces.params.ub[k*(Nvar-Nslacks) + 2])/2;
                    this->forces.params.x0[k*Nvar + 4] = (this->forces.params.lb[k*Nvar + 4] + this->forces.params.ub[k*(Nvar-Nslacks) + 3])/2;
                    this->forces.params.x0[k*Nvar + 5] = (this->forces.params.lb[k*Nvar + 5] + this->forces.params.ub[k*(Nvar-Nslacks) + 4])/2;
                    this->forces.params.x0[k*Nvar + 6] = (this->forces.params.lb[k*Nvar + 6] + this->forces.params.ub[k*(Nvar-Nslacks) + 5])/2;
                    this->forces.params.x0[k*Nvar + 7] = (this->forces.params.lb[k*Nvar + 7] + this->forces.params.ub[k*(Nvar-Nslacks) + 6])/2;
                }
            }
        }else{

            if(k==0){
                this->forces.params.x0[k*Nvar]     = 0.0;
                this->forces.params.x0[k*Nvar + 1] = 0.0;
                this->forces.params.x0[k*Nvar + 2] = carState(8);
                this->forces.params.x0[k*Nvar + 3] = carState(6);
                this->forces.params.x0[k*Nvar + 4] = forces.params.xinit[3];
                this->forces.params.x0[k*Nvar + 5] = forces.params.xinit[4];;
                this->forces.params.x0[k*Nvar + 6] = carState(4);
                this->forces.params.x0[k*Nvar + 7] = carState(5);
            }else{
                this->forces.params.x0[k*Nvar]     =  this->forces.params.lb[k*Nvar];
                this->forces.params.x0[k*Nvar + 1] = (this->forces.params.lb[k*Nvar + 1] + this->forces.params.ub[k*(Nvar-Nslacks)])/2;
                this->forces.params.x0[k*Nvar + 2] = (this->forces.params.lb[k*Nvar + 2] + this->forces.params.ub[k*(Nvar-Nslacks) + 1])/2;
                this->forces.params.x0[k*Nvar + 3] = (this->forces.params.lb[k*Nvar + 3] + this->forces.params.ub[k*(Nvar-Nslacks) + 2])/2;
                this->forces.params.x0[k*Nvar + 4] = (this->forces.params.lb[k*Nvar + 4] + this->forces.params.ub[k*(Nvar-Nslacks) + 3])/2;
                this->forces.params.x0[k*Nvar + 5] = (this->forces.params.lb[k*Nvar + 5] + this->forces.params.ub[k*(Nvar-Nslacks) + 4])/2;
                this->forces.params.x0[k*Nvar + 6] = (this->forces.params.lb[k*Nvar + 6] + this->forces.params.ub[k*(Nvar-Nslacks) + 5])/2;
                this->forces.params.x0[k*Nvar + 7] = (this->forces.params.lb[k*Nvar + 7] + this->forces.params.ub[k*(Nvar-Nslacks) + 6])/2;
            }
        }
    }

}

///////////////////////////////////////////////////////////////////////////////////////////////
//------------------------Auxiliar functions--------------------------------------------------

void MPC::get_solution(){

    // Concatenate optimized stages in Xopt ( n_opt*(N+1) x 1) --> (n_opt x (N+1) ) where n_opt = n_states + n_controls
    Eigen::MatrixXd optimized = vector2eigen(ipopt.solution);
    Eigen::Map<Eigen::MatrixXd> stages(optimized.topRows(n_states*(N+1)).data(),n_states,N+1);
    Eigen::Map<Eigen::MatrixXd> controls(optimized.bottomRows(n_controls*(N+1)).data(),n_controls,N+1);
    Eigen::MatrixXd Xopt(stages.rows()+controls.rows(), stages.cols());
    Xopt << controls, 
            stages;
    Eigen::MatrixXd solStatesT, solCommandsT;
    solStatesT = Xopt.bottomRows(n_states-1);
    solCommandsT = Xopt.topRows(n_controls+1);

    solStates = solStatesT.transpose();     // [n, mu vy, w] 
    solCommands = solCommandsT.transpose(); // [diff_delta, Mtv, delta]

    // cout << "solStates: " << endl;
    // cout << solStates << endl;

    // cout << "solCommands:\n";
    // cout << solCommands << endl;

}

void MPC::msgCommands(as_msgs::CarCommands *msg){
    
    msg->header.stamp = ros::Time::now();
    msg->motor = 0.0;
    msg->steering = solCommands(this->latency, 3);
    msg->Mtv = solCommands(this->latency, 2);

    // cout << "steering: " << solCommands(this->latency, 3) << endl;
    // cout << "Mtv: " << solCommands(this->latency, 2) << endl;

    return;
}

void MPC::get_debug_solution(as_msgs::MPCdebug *msg){

    if(this->forces.exit_flag == 0 || this->forces.exit_flag == 1){

        msg->header.stamp = ros::Time::now();

        msg->diff_delta = solCommands(this->latency, 1);
        msg->Mtv        = solCommands(this->latency, 2);
        msg->delta      = solCommands(this->latency, 3);

        msg->n  = solStates(this->latency, 0);
        msg->mu = solStates(this->latency, 1);
        msg->vy = solStates(this->latency, 2);
        msg->r  = solStates(this->latency, 3);

        msg->s = planner(this->latency, 2);
        msg->k = this->forces.params.all_parameters[24 + this->latency * this->Npar];

        double vx = this->forces.params.all_parameters[23 + this->latency * this->Npar];

        msg->alpha_f = atan( (msg->vy + this->Lf * msg->r)/vx ) - msg->delta;
        msg->alpha_r = atan( (msg->vy - this->Lr * msg->r)/vx );

        msg->kin_beta = atan(msg->delta*this->Lr/(this->Lr+this->Lf));
        msg->beta     = atan(msg->vy/vx);

        msg->x = this->lastState(this->latency, 0);
        msg->y = this->lastState(this->latency, 1);
        msg->heading = this->lastState(this->latency, 2);
    }

}

int MPC::first_index(const as_msgs::ObjectiveArrayCurv::ConstPtr& msg){

	double dist = 0.0, minDist = 100.0;
    int firstIdx = 0;
	Eigen::Vector2d position, car_direction;

    if(this->stateFlag){
        car_direction(0) = cos(carState(2));
        car_direction(1) = sin(carState(2));
        for (unsigned int i = 0; i < this->nSearch; i++){

            position(0) = (msg->objectives[i].x - carState(0));
            position(1) = (msg->objectives[i].y - carState(1));

            Eigen::Vector2d position_norm_vec = position/position.norm();

            // Only take into account those points with an angle of <65º with respect to the actual position
            if ( position_norm_vec.dot(car_direction) > cos(65.0 / 180.0 * M_PI)) {

                dist = position.norm();
                if (dist < minDist){
                    minDist = dist;
                    firstIdx = i;
                }
            }
        }
    }

    return firstIdx;
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

double MPC::continuous(double psi, double psi_last){
    
    double diff = psi - psi_last;
    
    int k = 0;
    while(abs(diff) > (2 * M_PI - 1)){

        if (k > 3) {k = 0; break;}

        if(k>0) k = -k;
        else if(k <= 0) k = -k + 1;
        diff = psi - psi_last + 2 * M_PI * k;
    }

    return psi + 2 * M_PI * k;
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
            if(!erase) file << "\n";
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
void MPC::save(string filePath, string name, mytype data, bool time, bool unique){

    try{
        
        // Write csv
        ofstream file;
        std::string full_name;
        if(unique){
            std::string datetime = currentDateTime();
            datetime = regex_replace(datetime, regex(":"), "-");
            full_name = filePath + name + "_" + datetime + ".csv";
        }else{
            full_name = filePath + name;
        }
        file.open(full_name, ios::app);
        if(time) file << ros::Time::now().toSec() << "," << data << endl;
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
        this->Dr = config.Dr;
        this->Df = config.Df;
        this->Cr = config.Cr;
        this->Cf = config.Cf;
        this->Br = config.Br;
        this->Bf = config.Bf;
        this->u_r = config.u_r;
        this->Cd = config.Cd;
        this->q_slip = config.q_slip;
        this->q_n = config.q_n;
        this->q_nN = config.q_nN;
        this->q_mu = config.q_mu;
        this->q_s = config.q_s;
        this->latency = config.latency;
        this->dMtv = config.dMtv;
        this->q_sN = config.q_sN;
        this->q_slack_track = config.q_slack_track;

        this->bounds.u_min[1] = -config.diff_delta*M_PI/180.0;
        this->bounds.u_max[0] = config.diff_delta*M_PI/180.0;

        this->bounds.x_min[3] = -config.Vy_max;
        this->bounds.x_max[3] = config.Vy_max;

        this->bounds.x_min[4] = -config.YawRate_max*M_PI/180.0;
        this->bounds.x_max[4] = config.YawRate_max*M_PI/180.0;

        this->dynParamFlag = true;

    } catch (exception& e){
        ROS_ERROR_STREAM("MPC::RECONFIGURE DIED" << e.what());
    }

}

