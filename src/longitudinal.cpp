#include "longitudinal.hh"

PID::PID(){
    carState = Eigen::VectorXd::Zero(9);
}

///////////////////////////////////////////////////////////////////////////////////////////////
//------------------------Callback functions--------------------------------------------------

void PID::stateCallback(const as_msgs::CarState::ConstPtr& msg){

    // carState << msg->odom.position.x, msg->odom.position.y, msg->odom.heading, msg->odom.velocity.x, msg->odom.velocity.y + msg->odom.velocity.w*d_IMU, msg->odom.velocity.w, msg->steering, msg->odom.acceleration.x, msg->Mtv;
    carState << msg->odom.position.x, msg->odom.position.y, msg->odom.heading, msg->odom.velocity.x, msg->odom.velocity.y, msg->odom.velocity.w, msg->steering, msg->odom.acceleration.x, msg->Mtv;
    stateFlag = true;
}

void PID::plannerCallback(const as_msgs::ObjectiveArrayCurv::ConstPtr& msg){

    if (msg->objectives.size() < minPoints){
        ROS_WARN("PID::Planner is too short!");
        return;
    }

    // Fill planner matrix
    planner.resize(msg->objectives.size(),4);
    for (unsigned int i = 0; i < msg->objectives.size(); i++)
    {	
        planner(i, 0) = msg->objectives[i].x;
        planner(i, 1) = msg->objectives[i].y;
        planner(i, 2) = msg->objectives[i].s; 
        planner(i, 3) = msg->objectives[i].k; 
    }

    if(this->stateFlag /*&& this->dynParamFlag*/) velocityProfile();
}

///////////////////////////////////////////////////////////////////////////////////////////////
//------------------------Principal functions--------------------------------------------------

void PID::run(){
    
    if(this->velFlag){
        // Proportional
        // double error = theorical_slip - slip; 
        // double Pout = Kp*error;

        // Integral
        // double period = 25e-3;
        // if(mode == 0) period = 50e-3;
        // integral += error*period;
        // double Iout = Ki*integral;
            
        // Apply PI
        // double torque = Pout + Iout + Dout;
        // cout << "T: " << torque << endl;
        // cout << "without control: " << desired_torque << endl;

        // // Save previous slip error
        // previous_error = error;
    }
}

void PID::velocityProfile(){

    vector<int> apexes = findLocalMax(planner.col(3));
    printVec(apexes,0);
    velocities.resize(planner.rows()); // final velocity profile vector

        // First step: restrain limit acceleration
    Eigen::VectorXd vAccel(planner.rows()); 
    vAccel(0) = carState(3);
    for(unsigned int j=0; j < planner.rows()-1; j++){
        vAccel(j+1) = vAccel(j) + spacing*f_accel(j, vAccel(j)); // Euler integration
        if( vAccel(j+1) > sqrt(ay_max/fabs(planner(j,3))) ) vAccel(j+1) = sqrt(ay_max/fabs(planner(j,3)));
    }

        // Second step: restrain limit deceleration
    Eigen::VectorXd vDecel(apexes[apexes.size()-1]+1);
    vDecel(0) = vAccel(apexes[apexes.size()-1]); 
    int idx; // global index in planner matrix
    for(unsigned int j=0; j < apexes[apexes.size()-1]; j++){
        idx = apexes[apexes.size()-1]-j;                            // transform index to global (we are going backwards)
        vDecel(j+1) = vDecel(j) + spacing*f_decel(idx, vDecel(j));  // Euler integration
        if( vDecel(j+1) > sqrt(ay_max/fabs(planner(idx,3))) ) vDecel(j+1) = sqrt(ay_max/fabs(planner(idx,3)));
    }
    Eigen::VectorXd vDecelGlobal = vDecel.reverse(); // reverse vDecel vector 

        // Third step: pick min values from both profiles
    for(unsigned int k=0; k < velocities.size(); k++){
        if(k <= apexes[apexes.size()-1]) velocities(k) = min(vAccel(k),vDecelGlobal(k));
        else velocities(k) = vAccel(k);
    }

    velFlag = true;
    saveEigen("/home/fetty/Desktop/", "velocity_profile.csv", velocities, true);
    saveEigen("/home/fetty/Desktop/", "v_accel.csv", vAccel, true);
    saveEigen("/home/fetty/Desktop/", "v_decel.csv", vDecelGlobal, true);
    saveEigen("/home/fetty/Desktop/", "curvature.csv", planner.col(3), true);
}


///////////////////////////////////////////////////////////////////////////////////////////////
//------------------------Auxiliar functions--------------------------------------------------

void PID::msgCommands(as_msgs::CarCommands *msg){
    
    msg->header.stamp = ros::Time::now();
    msg->motor = throttle;

    return;
}

// EDO models of the acceleration physics
double PID::f_accel(int k, double v){
    if(v < sqrt(ay_max/fabs(planner(k,3))) ){
        return ax_acc/v * sqrt( 1 - pow(pow(v,2)*fabs(planner(k,3))/ay_max, 2) ); // EDO Model
    } else {
        return 0; // Can't accelerate more
    }
}
double PID::f_decel(int k, double v){
    if(v < sqrt(ay_max/fabs(planner(k,3))) ){
        return ax_dec/v * sqrt( 1 - pow(pow(v,2)*fabs(planner(k,3))/ay_max, 2) ); // EDO Model
    } else {
        return 0; // Can't decelerate more
    }
}

template<typename mytype> 
void PID::printVec(vector<mytype> &input, int firstElements){
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

vector<int> PID::findLocalMax(Eigen::VectorXd curv){ // Find curvature apexes
    vector<int> result;

    if(curv(0) > curv(1)) result.push_back(0); // Check whether the first point is local max
    
    for(int i=1; i < curv.size()-1; i++){
        if(curv(i-1) < curv(i) && curv(i) > curv(i+1)) result.push_back(i);
    }

    if(curv(curv.size()-1) > curv(curv.size()-2)) result.push_back(curv.size()-1);

    return result;
}

void PID::saveEigen(string filePath, string name, Eigen::MatrixXd data, bool erase){

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
        ROS_ERROR_STREAM( "PID::saveEigen Exception was thrown: " << e.what() );
    }
}

void PID::reconfigure(tailored_mpc::longConfig& config){

    try{
        this->ax_acc = config.Ax_acc_max;
        this->ax_dec = config.Ax_dec_max;
        this->ay_max = config.Ay_max;
        this->vx_final = config.Vx_final;
        this->vx_max = config.Vx_max;

        this->dynParamFlag = true;

    } catch (exception& e){
        ROS_ERROR_STREAM("PID::RECONFIGURE DIED" << e.what());
    }
}
