#ifndef LONG_HH
#define LONG_HH

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <chrono>
#include <string>
#include <vector>
#include <stdio.h>
#include <time.h>
#include <eigen3/Eigen/Dense>

// Dynamic reconfigure headers
#include <dynamic_reconfigure/server.h>
#include <tailored_mpc/longConfig.h>

// Msgs used
#include "as_msgs/ObjectiveArrayCurv.h"
#include "as_msgs/CarState.h"
#include "as_msgs/CarCommands.h"

using namespace std;

class PID{

    private:

        int minPoints = 10;

        double ax_acc = 8, ax_dec = 10;     // Maximum longitudinal acceleration
        double ay_max = 10;                 // Maximum lateral acceleration
        double spacing = 0.025;             // Planner discretization
        double vx_max = 20, vx_final = 12;  // velocities restrictions

        bool stateFlag = false, velFlag = false;
        bool dynParamFlag = false;

        void velocityProfile();
        double f_accel(int k, double v);
        double f_decel(int k, double v);
        vector<int> findLocalMax(Eigen::VectorXd curv);
        void saveEigen(string filePath, string name, Eigen::MatrixXd data, bool erase);
        template<typename mytype> void printVec(vector<mytype> &input, int firstElements);

    public:

        PID();
        void stateCallback(const as_msgs::CarState::ConstPtr& msg);
        void plannerCallback(const as_msgs::ObjectiveArrayCurv::ConstPtr& msg);
        void reconfigure(tailored_mpc::longConfig& config);
        void msgCommands(as_msgs::CarCommands *msg);

        void run(); // run PI controller

        // Planner's trajectory matrix 
        Eigen::MatrixXd planner; // [x, y, s, k]

        //Actual state of the car
        Eigen::VectorXd carState; // [x, y, theta, vx, vy, w, delta(steering), acc, Mtv]

        // Velocity profile vector
        Eigen::VectorXd velocities;

        // throttle
        double throttle = 0;

};


#endif