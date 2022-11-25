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

// Dynamic reconfigure headers
#include <dynamic_reconfigure/server.h>
#include <tailored_mpc/dynamicConfig.h>

// Msgs used
#include "as_msgs/ObjectiveArrayCurv.h"
#include "as_msgs/CarState.h"
#include "as_msgs/CarCommands.h"

// Utilities for parameters
#include "utils/params.hh"

// Include headers of both solvers
#include "forces.hh"

using namespace std;

struct Boundaries{

        // VARIABLES BOUNDARIES:

          // Bounds and initial guess for the control
        vector<double> u_min =  { 0.0, 0.0, -3*M_PI/180, -5.0, -300}; // delta, Fm max,min bounds will be overwriten by dynamic reconfigure callback
        vector<double> u_max  = { 3*M_PI/180, 0.25,  300};
        vector<double> u0 = {  0.0, 0.0  };

          // Bounds and initial guess for the state
        vector<double> x_min  = { -23.0*M_PI/180, -1, -2.5, -50.0*M_PI/180, 2.0, -2.0, -50.0*M_PI/180 };
        vector<double> x_max  = { 23.0*M_PI/180, 1, 2.5, 50.0*M_PI/180, 25.0, 2.0, 50.0*M_PI/180 };
        vector<double> x0 = { 0.0, -1.25, 0.0, 0.0, 15.0, 0.0, 0.0 };

};

class MPC{

    private:

        // Internal variables/methods of MPC

        bool plannerFlag = false, stateFlag = false;
        bool paramFlag = false;                       // flag for parameters set up
        bool dynParamFlag = false;                    // flag for dynamic parameters set up
        bool troActive = false, troProfile = false;   // whether TRO/GRO are publishing

        // NLOP params
        int n_states = 5;             // number of state vars
        int n_controls = 6;           // number of control vars
        int N = 40;                   // horizon length
        int Nslacks = 2;              // number of slack vars
        int Npar = 31;                // number of real time parameters
        int sizeU, sizeX;             // size of states and controls FORCES arrays

        // MPC
        int nPlanning = 1900;     // number of points wanted from the planner
        bool firstIter = true;    // first iteration flag
        int samplingS = 10;       // s sampling distance 
        double delta_s = 0.025;   // planner discretization [m]
        double rk4_t = 0.025;     // Integration time [s]
        
        // DYNAMIC PARAMETERS:
          // see "dynamic.cfg" for explanation
        double dRd = 8;
        double dRa = 2; 
        double Dr = 3152.3;
        double Df = 2785.4;
        double Cr = 1.6;
        double Cf = 1.6;
        double Br = 10.1507;
        double Bf = 10.8529;
        double u_r = 0.45;
        double Cd = 0.8727;
        double q_slip = 2;
        double p_long = 0.5;
        double q_n = 5;
        double q_nN = 5;
        double q_mu = 0.1;
        double lambda = 1;
        double q_s = 30;
        // int latency = 4;
        double Cm = 4000;
        double dMtv = 1;
        double ax_max = 7;
        double ay_max = 10;
        double q_sN = 10;

        double q_slack_vx = 0;
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
        double d_IMU = -0.318;
        double Rwheel = 0.2;

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
        vector<double> vconcat(const vector<double>& x, const vector<double>& y);
        void printVec(vector<double> &input, int firstElements=0);
        Eigen::MatrixXd vector2eigen(vector<double> vect);
        Eigen::MatrixXd output2eigen(double* array, int size);
        double throttle_to_torque(double throttle);
        double ax_to_throttle(double ax);
        double continuous(double psi, double psi_last); // Garanty that both angles are on the same range
        const string currentDateTime(); // get current date/time, format is YYYY-MM-DD.HH:mm:ss


    public:

        MPC(const Params* params);

        void reconfigure(tailored_mpc::dynamicConfig& config);
        void msgCommands(as_msgs::CarCommands *msg);
        void saveEigen(string filePath, string name, Eigen::MatrixXd data, bool erase); // save matrix data into file
        template<typename mytype> void save(string filePath, string name, mytype data, bool time);

        // Callbacks
        void stateCallback(const as_msgs::CarState::ConstPtr& msg);
        void plannerCallback(const as_msgs::ObjectiveArrayCurv::ConstPtr& msg);
        void troCallback(const as_msgs::ObjectiveArrayCurv::ConstPtr& msg);

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

        // Planner's trajectory matrix 
        Eigen::MatrixXd planner; // [x, y, s, k, vx, L, R]

        //Actual state of the car
        Eigen::VectorXd carState; // [x, y, theta, vx, vy, w, delta(steering), acc, Mtv]

        // Previous state
        Eigen::MatrixXd lastState;    // [x, y, theta, vx, vy, w]
        Eigen::MatrixXd lastCommands; // [diff_delta, diff_Fm, Mtv, delta, Fm]

        // Previous solution
        Eigen::MatrixXd solStates;    // [n, mu, vx, vy, w]
        Eigen::MatrixXd solCommands;  // [slack_vx, slack_track, diff_delta, diff_acc, Mtv, delta, Fm]

};


#endif