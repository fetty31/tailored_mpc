#include "mpc.hh"

// Constructor
MPC::MPC(const Params& params){

    // NLOP params
    this->n_states = params.mpc.nlop.n_states;
    this->n_controls = params.mpc.nlop.n_controls;
    this->N = params.mpc.nlop.N;
    this->Npar = params.mpc.nlop.Npar + this->n_states + this->N; // [ 23 (MPC parameters) + (initial state) + n (curvature points == N) ]

    // MPC
    this->T = params.mpc.PredTime/N;
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

    this->paramFlag = true;

    planner = Eigen::MatrixXd::Zero(nPlanning,9);
    carState = Eigen::VectorXd::Zero(carState.rows());
    predicted_s = Eigen::VectorXd::Zero(N);
    lastState = Eigen::MatrixXd::Zero(N,7);
    lastCommands = Eigen::MatrixXd::Zero(N,2);

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


void MPC::solve_IPOPT(){

    if(paramFlag && dynParamFlag && plannerFlag && stateFlag){

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
    Npar = param.size();

    cout << "XINIT: " << endl;
    printVec(xinit);

}

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
    predicted_s(0) = fmod(predicted_s(0),smax);


}

// void curv_mpc::Compute_s_pred(MatrixXd planner, VectorXd& state){

//     // Compute predicted s using s0, predicted states and the Euler method. Compute x, y and theta using Euler method
//     double Ts_curv = 25e-3;
//     // Clear previous prediction and initialize s
//     sPrediction.setZero();
//     // sPrediction(0) = curv_params.all_parameters[25];
//     sPrediction(0) = fmod(curv_params.all_parameters[25], s_max);
    
//     // Initial state
//     lastState(0, 0) = state(0);
//     lastState(0, 1) = state(1);
//     lastState(0, 2) = state(2);
//     lastState(0, 3) = lastStateCurv(0, 2);
//     lastState(0, 4) = lastStateCurv(0, 3);
//     lastState(0, 5) = lastStateCurv(0, 4);

//     // Curvature vector
//     VectorXd curv(lastStateCurv.rows());

//     // Predict rest of s
//     for(int i = 1; i < Ncurv; i++){

//         double n = lastStateCurv(i-1, 0);
//         double mu = lastStateCurv(i-1, 1);
//         double vx = lastStateCurv(i-1, 2);
//         double vy = lastStateCurv(i-1, 3);
//         double theta = lastState(i-1, 2);
//         double w = lastStateCurv(i-1, 4);
//         double k =  curv_params.all_parameters[19 + (i-1)*n_params_curv];

//         // Save curvature
//         curv(i-1) = k;

//         // cout << " vx: " << vx << " vy: " << vy << " mu: " << mu << " n: " <<n << endl;
            
//         sPrediction(i) = fmod((sPrediction(i-1) + (vx*cos(mu) - vy*sin(mu))/(1 - n*k)*Ts_curv), s_max);
//         // cout << "Spred(" << i << ") = " << sPrediction(i) << endl;
//         // sPrediction(i) = (sPrediction(i-1) + (vx*cos(mu) - vy*sin(mu))/(1 - n*k)*Ts_curv);

//         // Ensure that s_dot is grater than 0
//         if((vx*cos(mu) - vy*sin(mu))/(1 - n*k) < 0){
//             cout << "NEGATIVE S_DOT: " << (vx*cos(mu) - vy*sin(mu))/(1 - n*k) <<  endl;
//             break;
//         }

//         //States
//         lastState(i, 0) = lastState(i-1, 0) + (vx*cos(theta) - vy*sin(theta))*Ts_curv;
//         lastState(i, 1) = lastState(i-1, 1) + (vx*sin(theta) + vy*cos(theta))*Ts_curv;
//         lastState(i, 2) = lastState(i-1, 2) + w*Ts_curv;
//         lastState(i, 3) = vx;
//         lastState(i, 4) = vy;
//         lastState(i, 5) = w;

//         lastCommands(i-1, 0) = lastCommandsCurv(i-1, 0);
//         lastCommands(i-1, 1) = lastCommandsCurv(i-1, 1);
//         lastCommands(i-1, 2) = lastCommandsCurv(i-1, 2);
//         lastCommands(i-1, 3) = lastCommandsCurv(i-1, 3);
//         lastCommands(i-1, 4) = lastCommandsCurv(i-1, 4);
//     }

//     // If predicted s is too small set initial s again
//     double total_length = sPrediction(Ncurv - 1) - sPrediction(0);
//     if(total_length < 0 ) total_length += s_max;

//     if(total_length < 2.5 && state(3) > 4){
//         firstCurv = true;
//         cout << "S_DOT: " << endl;
//         for(int i = 0; i < lastStateCurv.rows(); i++){
//             cout << (lastStateCurv(i, 2)*cos(lastStateCurv(i, 1)) - lastStateCurv(i, 3)*sin(lastStateCurv(i, 1)))/(1 - lastStateCurv(i, 0)*curv(i)) << endl;
//         }
//         cout << "info" << curv_info.it << ", " <<  curv_info.pobj << endl;
//         cout << "S: " << endl;
//         cout << sPrediction << endl;
//     }
// }

void MPC::get_solution_IPOPT(){

    // Concatenate optimized stages in Xopt ( 9*(N+1) x 1) --> (9 x (N+1) )

    Eigen::MatrixXd optimized = vector2eigen(ipopt.solution);
    Eigen::Map<Eigen::MatrixXd> stages(optimized.topRows(n_states*(N+1)).data(),n_states,N+1);
    Eigen::Map<Eigen::MatrixXd> control(optimized.bottomRows(n_controls*(N+1)).data(),n_controls,N+1);
    // Eigen::MatrixXd Xopt(stages.rows()+control.rows(), stages.cols());
    // Xopt << control, 
    //         stages;

    solStates = stages;
    solCommands = control;

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

    Eigen::MatrixXd result;
    long int rows = 0;
    vector<double>::iterator row;
    for(row = vect.begin(); row != vect.end(); ++row){
            result(rows,0) = *row; 
            rows++;
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

        this->dynParamFlag = true;

    } catch (exception& e){
        ROS_ERROR_STREAM("MPC RECONFIGURE DIED" << e.what());
    }

}

