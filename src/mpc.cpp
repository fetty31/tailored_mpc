#include "mpc.hh"

// Constructor
MPC::MPC(const Params* params){

    // NLOP params
    this->N = params->mpc.nlop.N;
    this->Nslacks = params->mpc.nlop.Nslacks;

    cout << "Nslacks: " << Nslacks << endl;

    // MPC
    this->Hz            = params->mpc.Hz;
    this->rk4_t         = params->mpc.rk4_t;
    this->delta_s       = params->mpc.delta_s;
    this->nPlanning     = params->mpc.nPlanning;
    this->nSearch       = params->mpc.nSearch;
    this->Nthreads      = params->mpc.Nthreads;

    cout << "Hz: " << Hz << endl;
    cout << "rk4_t: " << rk4_t << endl;

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

    int sizeParam = sizeof(forces.params.all_parameters)/sizeof(forces.params.all_parameters[0]); // size of params FORCES array
    this->Npar = int(sizeParam/N);

    this->n_states = int(sizeX/N);      // number of car state variables
    this->n_controls = int(sizeU/N);    // number of commands variables    

    cout << "n_states: " << n_states << endl;
    cout << "n_controls: " << n_controls << endl;
    cout << "Npar: " << Npar << endl;

    planner = Eigen::MatrixXd::Zero(nPlanning,7);
    carState = Eigen::VectorXd::Zero(9);

    lastState = Eigen::MatrixXd::Zero(N,6);                         // [x, y, theta, vx, vy, r]
    lastCommands = Eigen::MatrixXd::Zero(N,n_controls-Nslacks);     // [delta]

    solStates = Eigen::MatrixXd::Zero(N,n_states);        // [y, vy, theta (heading/yaw), r]
    solCommands = Eigen::MatrixXd::Zero(N,n_controls);    // [delta]

    this->paramFlag = true;
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
        idx0 = first_index(msg);
        planner.resize(nPlanning-idx0, 7);
        for (unsigned int i = idx0; i < nPlanning-idx0 ; i++)
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
    idx0 = first_index(msg);
    planner.resize(nPlanning-idx0, 7);
    for (unsigned int i = idx0; i < nPlanning-idx0 ; i++)
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
    // cout << "SMAX: " << smax << endl;
    plannerFlag = troActive = true; // we are now following TRO's trajectory
    ROS_WARN_ONCE("MPC: FOLLOWING TRO! :)");

}

void MPC::velsCallback(const as_msgs::CarVelocityArray::ConstPtr& msg){

    if (msg->velocities.size() < this->N){
        ROS_WARN("MPC: Velocity profile too short!");
        return;
    }

    pred_velocities.resize(msg->velocities.size()-idx0);
    for(int i=idx0; i < msg->velocities.size()-idx0; i++){
        pred_velocities(i) = msg->velocities[i].x;
    }
    this->velsFlag = true; // we have received velocity data

}


///////////////////////////////////////////////////////////////////////////////////////////////
//------------------------Principal functions--------------------------------------------------

void MPC::solve(){

    if(paramFlag && dynParamFlag && plannerFlag && stateFlag && velsFlag){
        
        auto start_time = chrono::system_clock::now();

        // Set number of internal threads
        forces.params.num_of_threads = this->Nthreads;

        initial_conditions();   // compute initial state (xinit)
        set_params_bounds();    // set parameters & boundaries of the NLOP (Non Linear Optimization Problem)

        // Get i-th memory buffer
        int i = 0;
        forces.mem_handle = TailoredSolver_internal_mem(i);
        /* Note: number of available memory buffers is controlled by code option max_num_mem */

        // Solve
        forces.exit_flag = TailoredSolver_solve(&forces.params, &forces.solution, &forces.info, forces.mem_handle, NULL, forces.ext_func);

        ROS_ERROR_STREAM("MPC exit flag: " << forces.exit_flag);
        ROS_ERROR_STREAM("MPC solve time: " << forces.info.solvetime*1000 << " ms");
        ROS_ERROR_STREAM("MPC iterations: " << forces.info.it);

        if(forces.exit_flag == 1 || forces.exit_flag == 0) this->firstIter = false;
        else this->firstIter = true; // we go back to first iteration's pipeline if the NLOP didn't converge

        get_solution(); // save current solution (predicted states & controls)
        if(forces.exit_flag == 1 || forces.exit_flag == 0) state_prediction(); 
        /* Note: we predict the evolution of the progress (s) if optimal solution is found (exit flag == 1) or maximum number of iterations is reached (exit flag == 0)*/

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
            ROS_ERROR("MPC: No data from state car, planner or velocity profile");
            ROS_ERROR_STREAM("Planner status: " << plannerFlag);
            ROS_ERROR_STREAM("State status: " << stateFlag);
            ROS_ERROR_STREAM("Vel profile status: " << velsFlag);
        }
    }
}

void MPC::initial_conditions(){

    // xinit = [delta, y, Vy, theta (heading/yaw), r]
    forces.params.xinit[0] = carState(6);
    forces.params.xinit[1] = carState(1);
    forces.params.xinit[2] = carState(4);
    forces.params.xinit[3] = carState(2);
    forces.params.xinit[4] = carState(5);

    // Get heading of planner points
    cout << "Heading from planner path\n";
    this->heading.resize(planner.rows()-1);
    double xv, yv; // aux vars
    for(unsigned int i=0; i<planner.rows()-1; i++){
        xv = planner(i+1, 0) - planner(i, 0);
        yv = planner(i+1, 1) - planner(i, 1);
        heading(i) = atan2(yv, xv);
        cout << heading(i)*180.0/M_PI << endl;
    }

    // Set y boundaries
    y_bounds.resize(planner.rows(), 2);
    y_bounds(0,0) = planner(0,5)*sin(carState(2)+M_PI);
    y_bounds(0,1) = planner(0,6)*sin(carState(2)+M_PI);
    for(unsigned j=1; j<planner.rows(); j++){
        y_bounds(j,0) = planner(j,1) + planner(j,5)*cos(heading(j-1));
        y_bounds(j,1) = planner(j,1) - planner(j,6)*cos(heading(j-1));
    }

    cout << "Xinit:\n";
    for(int i=0;i<5;i++){
        cout << forces.params.xinit[i] << endl;
    }

    // cout << "Steering feedback: " << carState(6) << endl;

}

void MPC::set_params_bounds(){

    // int size = sizeof(forces.params.hu)/sizeof(forces.params.hu[0]);
    // int nh = int(size/this->N);                    // number of inequality constraints
    int Nvar = this->n_states + this->n_controls;  // number of total variables (state + control)

    // cout << "nh: " << nh << endl;
    cout << "Nvar: " << Nvar << endl;

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
        this->forces.params.all_parameters[17 + k*this->Npar] = this->q_theta;
        this->forces.params.all_parameters[18 + k*this->Npar] = (k == N-1) ? this->q_yN : this->q_y;
        this->forces.params.all_parameters[19 + k*this->Npar] = this->rk4_t;
        this->forces.params.all_parameters[20 + k*this->Npar] = this->q_slack_track;

        double plannerIdx = this->samplingS*k; // we pick equally spaced points from the planner

        cout << "plannerIdx: " << plannerIdx << endl;

        this->forces.params.all_parameters[21 + k*this->Npar] = pred_velocities(plannerIdx); // velocity profile from longitudinal pid
        this->forces.params.all_parameters[22 + k*this->Npar] = heading(plannerIdx-1);       // heading (theta/yaw) 
        this->forces.params.all_parameters[23 + k*this->Npar] = planner(plannerIdx, 1);      // y coord 
        cout << "y coord: " << forces.params.all_parameters[23+k*Npar] << endl;
        cout << "heading: " << forces.params.all_parameters[22+k*Npar] << endl;
        cout << "pred velocity: " << pred_velocities(plannerIdx) << endl;

        // Inequality constraints bounds:
        // this->forces.params.hu[k*nh]     = fabs(planner(plannerIdx, 5)); // L(s) ortogonal left dist from the path to the track limits
        // this->forces.params.hu[k*nh + 1] = fabs(planner(plannerIdx, 6)); // R(s) ortogonal right dist from the path to the track limits

        // Variables bounds:
        this->forces.params.lb[k*Nvar]     = this->bounds.u_min[0];
        this->forces.params.lb[k*Nvar + 1] = min(y_bounds(plannerIdx,0), y_bounds(plannerIdx,1));
        this->forces.params.lb[k*Nvar + 2] = this->bounds.x_min[1];
        this->forces.params.lb[k*Nvar + 3] = this->bounds.x_min[2];
        this->forces.params.lb[k*Nvar + 4] = this->bounds.x_min[3];

            // Slack variables doesn't have any upper bounds, so Nvar-Nslacks is used here
        this->forces.params.ub[k*(Nvar-Nslacks)]     = this->bounds.u_max[0];
        this->forces.params.ub[k*(Nvar-Nslacks) + 1] = max(y_bounds(plannerIdx,0), y_bounds(plannerIdx,1));
        this->forces.params.ub[k*(Nvar-Nslacks) + 2] = this->bounds.x_max[1];
        this->forces.params.ub[k*(Nvar-Nslacks) + 3] = this->bounds.x_max[2];
        this->forces.params.ub[k*(Nvar-Nslacks) + 4] = this->bounds.x_max[3];

        cout << "BOUNDARIES\n";
        cout << forces.params.lb[k*Nvar] << endl;
        cout << forces.params.ub[k*(Nvar-Nslacks)] << endl;

        cout << "UMAX\n";
        cout << this->bounds.u_max[0] << endl;

        if(!firstIter){
            if((latency + k) < this->N - 1){
                if(this->forces.exit_flag == 1){
                    this->forces.params.x0[k*Nvar]     = solCommands(latency+k, 0);
                    this->forces.params.x0[k*Nvar + 1] = solStates(latency+k, 0);
                    this->forces.params.x0[k*Nvar + 2] = solStates(latency+k, 1);
                    this->forces.params.x0[k*Nvar + 3] = solStates(latency+k, 2);
                    this->forces.params.x0[k*Nvar + 4] = solStates(latency+k, 3);
                }else{
                    this->forces.params.x0[k*Nvar]     = (this->forces.params.lb[k*Nvar] + this->forces.params.ub[k*(Nvar-Nslacks)])/2;
                    this->forces.params.x0[k*Nvar + 1] = planner(plannerIdx, 1);
                    this->forces.params.x0[k*Nvar + 2] = (this->forces.params.lb[k*(Nvar) + 2] + this->forces.params.ub[k*(Nvar-Nslacks) + 2])/2;
                    this->forces.params.x0[k*Nvar + 3] = (this->forces.params.lb[k*(Nvar) + 3] + this->forces.params.ub[k*(Nvar-Nslacks) + 3])/2;
                    this->forces.params.x0[k*Nvar + 4] = (this->forces.params.lb[k*(Nvar) + 4] + this->forces.params.ub[k*(Nvar-Nslacks) + 4])/2;
                }
            }else{
                if(this->forces.exit_flag == 1){
                    this->forces.params.x0[k*Nvar]     = solCommands(N-1, 0);
                    this->forces.params.x0[k*Nvar + 1] = solStates(N-1, 0);
                    this->forces.params.x0[k*Nvar + 2] = solStates(N-1, 1);
                    this->forces.params.x0[k*Nvar + 3] = solStates(N-1, 2);
                    this->forces.params.x0[k*Nvar + 4] = solStates(N-1, 3);
                }else{
                    this->forces.params.x0[k*Nvar]     = (this->forces.params.lb[k*Nvar] + this->forces.params.ub[k*(Nvar-Nslacks)])/2;
                    this->forces.params.x0[k*Nvar + 1] = planner(plannerIdx, 1);
                    this->forces.params.x0[k*Nvar + 2] = (this->forces.params.lb[k*(Nvar) + 2] + this->forces.params.ub[k*(Nvar-Nslacks) + 2])/2;
                    this->forces.params.x0[k*Nvar + 3] = (this->forces.params.lb[k*(Nvar) + 3] + this->forces.params.ub[k*(Nvar-Nslacks) + 3])/2;
                    this->forces.params.x0[k*Nvar + 4] = (this->forces.params.lb[k*(Nvar) + 4] + this->forces.params.ub[k*(Nvar-Nslacks) + 4])/2;
                }
            }
        }else{

            if(k==0){
                this->forces.params.x0[k*Nvar]     = carState(6);
                this->forces.params.x0[k*Nvar + 1] = carState(1);
                this->forces.params.x0[k*Nvar + 2] = carState(4);
                this->forces.params.x0[k*Nvar + 3] = carState(2);
                this->forces.params.x0[k*Nvar + 4] = carState(5);
            }else{
                this->forces.params.x0[k*Nvar]     = (this->forces.params.lb[k*Nvar] + this->forces.params.ub[k*(Nvar-Nslacks)])/2;
                this->forces.params.x0[k*Nvar + 1] = planner(plannerIdx, 1);
                this->forces.params.x0[k*Nvar + 2] = (this->forces.params.lb[k*(Nvar) + 2] + this->forces.params.ub[k*(Nvar-Nslacks) + 2])/2;
                this->forces.params.x0[k*Nvar + 3] = (this->forces.params.lb[k*(Nvar) + 3] + this->forces.params.ub[k*(Nvar-Nslacks) + 3])/2;
                this->forces.params.x0[k*Nvar + 4] = (this->forces.params.lb[k*(Nvar) + 4] + this->forces.params.ub[k*(Nvar-Nslacks) + 4])/2;
            }
        }
    }

}

void MPC::state_prediction(){

    lastState(0,0) = carState(0);
    lastState(0,1) = carState(1);
    lastState(0,2) = carState(2);
    lastState(0,3) = carState(3);
    lastState(0,4) = solStates(0,1);
    lastState(0,5) = solStates(0,3);

    for(int i=1; i < N; i++){

        double theta = lastState(i-1, 2);
        double vx = forces.params.all_parameters[21 + (i-1)*this->Npar]; // predicted vel. from longitudinal pid
        double vy = solStates(i-1, 1);
        double w = solStates(i-1, 3);

        // Predict states evolution with Euler
        lastState(i, 0) = lastState(i-1,0) + (vx*cos(theta) - vy*sin(theta))*this->rk4_t;
        lastState(i, 1) = lastState(i-1,1) + (vx*sin(theta) + vy*cos(theta))*this->rk4_t;
        lastState(i, 2) = lastState(i-1,2) + w*this->rk4_t;
        lastState(i, 3) = vx;
        lastState(i, 4) = vy;
        lastState(i, 5) = w;

        lastCommands(i-1, 0) = solCommands(i-1, 0); // Delta

    }

}

///////////////////////////////////////////////////////////////////////////////////////////////
//------------------------Auxiliar functions--------------------------------------------------

void MPC::get_solution(){

    // Change vec dimensions from x(4*N x 1) u(N x 1) to x(N x 4) u(N x 1)
    Eigen::MatrixXd u = output2eigen(forces.solution.U, sizeU);
    Eigen::MatrixXd x = output2eigen(forces.solution.X, sizeX);

    // Eigen::Map<Eigen::MatrixXd> controlsT(u.data(), n_controls, N);
    Eigen::Map<Eigen::MatrixXd> statesT(x.data(), n_states, N);

    solStates = statesT.transpose();
    solCommands = u; //controlsT.transpose();

    cout << "solStates: " << endl;
    cout << solStates << endl;

    cout << "solCommands:\n";
    // cout << solCommands << endl;
    for(int i=0; i<sizeU; i++) cout << forces.solution.U[i] << endl;

}

void MPC::msgCommands(as_msgs::CarCommands *msg){
    
    msg->header.stamp = ros::Time::now();
    msg->motor = 0.0;
    msg->steering = solCommands(this->latency, 0);
    msg->Mtv = 0.0;

    cout << "steering: " << solCommands(this->latency, 0) << endl;

    return;
}

int MPC::first_index(const as_msgs::ObjectiveArrayCurv::ConstPtr& msg){

	double dist, minDist;
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
        this->Dr = config.Dr;
        this->Df = config.Df;
        this->Cr = config.Cr;
        this->Cf = config.Cf;
        this->Br = config.Br;
        this->Bf = config.Bf;
        this->u_r = config.u_r;
        this->Cd = config.Cd;
        this->q_slip = config.q_slip;
        this->q_theta = config.q_theta;
        this->q_y = config.q_y;
        this->latency = config.latency;
        this->q_yN = config.q_yN;
        this->q_slack_track = config.q_slack_track;

        // this->bounds.u_min[0] = -config.diff_delta*M_PI/180.0;
        // this->bounds.u_max[0] = config.diff_delta*M_PI/180.0;

        this->bounds.x_min[1] = -config.Vy_max;
        this->bounds.x_max[1] = config.Vy_max;

        this->bounds.x_min[3] = -config.YawRate_max*M_PI/180.0;
        this->bounds.x_max[3] = config.YawRate_max*M_PI/180.0;

        this->dynParamFlag = true;

    } catch (exception& e){
        ROS_ERROR_STREAM("MPC::RECONFIGURE DIED" << e.what());
    }

}

