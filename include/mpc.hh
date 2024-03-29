/*
 Copyright (c) 2023 Oriol Martínez @fetty31

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef MPC_HH
#define MPC_HH

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
#include <regex>

// Dynamic reconfigure headers
#include <dynamic_reconfigure/server.h>
#include <tailored_mpc/dynamicConfig.h>

// Msgs used
#include "as_msgs/ObjectiveArrayCurv.h"
#include "as_msgs/CarState.h"
#include "as_msgs/CarCommands.h"
#include "as_msgs/CarVelocityArray.h"
#include "as_msgs/MPCdebug.h"

// Utilities
#include "utils/params.hh"

// Include headers of solver
#include "forces.hh"

using namespace std;

struct Boundaries{

        // VARIABLES BOUNDARIES:

          // Bounds and initial guess for the control
        vector<double> u_min =  { 0.0, -3*M_PI/180, -800}; // delta max,min bounds will be overwriten by dynamic reconfigure callback
        vector<double> u_max  = { 3*M_PI/180, 800};
        vector<double> u0 = {  0.0, 0.0  };

          // Bounds and initial guess for the state
        vector<double> x_min  = { -23.0*M_PI/180, -2.0, -50.0*M_PI/180, -5.0, -150.0*M_PI/180 };
        vector<double> x_max  = { 23.0*M_PI/180, 2.0, 50.0*M_PI/180, 5.0, 150.0*M_PI/180 };
        vector<double> x0 = { 0.0, 0.0, 0.0, 0.0, 0.0 };

};

class MPC{

    private:

        // Internal variables/methods of MPC

        string debug_path;

        bool plannerFlag = false, stateFlag = false, velsFlag = false;
        bool dynParamFlag = false;                    // flag for dynamic parameters set up
        bool troActive = false, troProfile = false;   // whether TRO/GRO are publishing

        // NLOP params
        int n_states = 5;             // number of state vars
        int n_controls = 6;           // number of control vars
        int N = 40;                   // horizon length
        int Nslacks = 2;              // number of slack vars
        int Npar = 31;                // number of real time parameters
        int sizeU, sizeX;             // size of states and controls FORCES arrays
        // int idx0 = 0;                 // idx of closest point to the car

        // MPC
        int nPlanning = 1900;     // number of points wanted from the planner
        int nSearch = 200;        // number of points where we will look for the close point to the car [not used]
        bool firstIter = true;    // first iteration flag
        int samplingS = 10;       // s sampling distance 
        double delta_s = 0.025;   // planner discretization [m]
        double rk4_t = 0.025;     // Integration time [s]
        
        // DYNAMIC PARAMETERS:
          // see "dynamic.cfg" for explanation
        double dRd = 8;
        double Dr = 3152.3;
        double Df = 2785.4;
        double Cr = 1.6;
        double Cf = 1.6;
        double Br = 10.1507;
        double Bf = 10.8529;
        double u_r = 0.45;
        double Cd = 0.8727;
        double q_slip = 2;
        double q_n = 5;
        double q_nN = 5;
        double q_mu = 0.1;
        double lambda = 1;
        double q_s = 30;
        // int latency = 4; made public for debugging
        double dMtv = 1;
        double q_sN = 10;

        double q_slack_track = 0;

        // STATIC PARAMETERS: 
          // see "params.hh" for explanation
        double m = 240;
        double Lf = 0.708;
        double Lr = 0.822;
        double gravity = 9.81;
        double Ar = 1;
        double rho = 1.255;
        double I = 93;
        double longue = 2.72;
        double width = 1.5;

        // Initial conditions evaluation
        void initial_conditions();

        // S prediction
        void s_prediction();
        Eigen::VectorXd predicted_s;
        Eigen::VectorXd progress;
        double smax = 0;

        // FORCESPRO:
        void set_params_bounds(); // here parameters & boundaries are added in the same for loop
        void get_solution();

        // Aux:
        int first_index(const as_msgs::ObjectiveArrayCurv::ConstPtr& msg);
        vector<double> vconcat(const vector<double>& x, const vector<double>& y);
        void printVec(vector<double> &input, int firstElements=0);
        Eigen::MatrixXd vector2eigen(vector<double> vect);
        Eigen::MatrixXd output2eigen(double array[], int size);
        double continuous(double psi, double psi_last); // Garanty that both angles are on the same range
        const string currentDateTime(); // get current date/time, format is YYYY-MM-DD.HH:mm:ss

    public:

        MPC(const Params* params);

        void reconfigure(tailored_mpc::dynamicConfig& config);
        void msgCommands(as_msgs::CarCommands *msg);
        void saveEigen(string filePath, string name, Eigen::MatrixXd data, bool erase); // save matrix data into file
        template<typename mytype> void save(string filePath, string name, mytype data, bool time, bool unique=false);
        void get_debug_solution(as_msgs::MPCdebug *msg);

        // Callbacks
        void stateCallback(const as_msgs::CarState::ConstPtr& msg);
        void plannerCallback(const as_msgs::ObjectiveArrayCurv::ConstPtr& msg);
        void troCallback(const as_msgs::ObjectiveArrayCurv::ConstPtr& msg);
        void velsCallback(const as_msgs::CarVelocityArray::ConstPtr& msg);

        // Solve method
        void solve();

        // Structs declaration
        Boundaries bounds;
        ForcesproSolver forces = ForcesproSolver();

        // MPC
        double Hz = 20;     // [Hz]
        uint Nthreads = 1;  // number of internal threads
        int mission = 0;    // 0 for AX, 1 for TD, 2 for SkidPad, 3 for Acceleration
        chrono::duration<double> elapsed_time;

        int latency = 4;
        bool debug_flag = false;

        // Planner's trajectory matrix 
        Eigen::MatrixXd planner; // [x, y, s, k, vx, L, R]

        //Actual state of the car
        Eigen::VectorXd carState; // [x, y, theta, vx, vy, w, delta(steering), acc, Mtv]

        // Previous state
        Eigen::MatrixXd lastState;    // [x, y, theta, vx, vy, w]
        Eigen::MatrixXd lastCommands; // [diff_delta, Mtv, delta]

        // Previous solution
        Eigen::MatrixXd solStates;    // [n, mu, vy, w]
        Eigen::MatrixXd solCommands;  // [slack_track, diff_delta, Mtv, delta]

        // Predicted velocities (vel profile from long_pid pkg)
        Eigen::VectorXd pred_velocities; 
};


#endif