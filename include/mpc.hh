#ifndef MPC_HH
#define MPC_HH

#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <chrono>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>

#include <dynamic_reconfigure/server.h>
#include <tailored_mpc/dynamicConfig.h>

#include "as_msgs/ObjectiveArrayCurv.h"
#include "as_msgs/CarState.h"

#include "casadi/casadi.hpp"
#include "structures/params.hh"

using namespace std;

struct IPOPT{

    vector<double> lbx;        // Lower boundaries stages (x,u)
    vector<double> ubx;        // Upper boundaries stages (x,u)

    vector<double> lbg_next;   // Lower boundaries continuity constraints 
    vector<double> ubg_next;   // Upper boundaries continuity constraints

    vector<double> lbg_track;  // Lower boundaries track constraints
    vector<double> ubg_track;  // Upper boundaries track constraints

    vector<double> lbg_elipse;  // Lower boundaries ellipse constraints 
    vector<double> ubg_elipse;  // Upper boundaries ellipse constraints

    vector<double> x0;         // Initial guess
    vector<double> p;          // Parameters + curvature
    vector<double> solution;   // Solution --> Optimized stages

    string exit_flag;

    shared_ptr<casadi::Function> solver_ptr; // Solver object

};

struct Boundaries{

        // VARIABLES BOUNDARIES:

          // Bounds and initial guess for the control
        vector<double> u_min =  { -3*M_PI/180, -5.0 };
        vector<double> u_max  = {  3*M_PI/180, 0.25  };
        vector<double> u0 = {  0.0, 0.0  };

          // Bounds and initial guess for the state
        vector<double> x_min  = { -23.0*M_PI/180, -8.0, -3, -150.0*M_PI/180, 0.0, -2.0, -100.0*M_PI/180 };
        vector<double> x_max  = { 23.0*M_PI/180, 5.5, 3, 150.0*M_PI/180, 25.0, 2.0, 100.0*M_PI/180 };
        vector<double> x0 = { 0.0, -1.25, 0.0, 0.0, 15.0, 0.0, 0.0 };

        double lower_continuity = -0.01; // Lower bound continuity constraint
        double upper_continuity = 0.01;  // Upper bound continuity constraint

        double lower_track = 0.0;        // Lower bound track constraint
        double upper_track = 1.5;        // Upper bound track constraint

        double lower_ellipse = -casadi::inf;       // Lower bound ellipse constraint 
        double upper_ellipse = 0.0;      // Upper bound ellipse constraint

};

class MPC{

    private:

        // Internal variables/methods of MPC

        bool plannerFlag = false, stateFlag = false;
        bool paramFlag = false; // flag for parameters set up
        bool dynParamFlag = false; // flag for dynamic parameters set up
        bool FORCES = false; // flag for using FORCESPRO

        // NLOP params
        int n_states = 7;
        int n_controls = 2;
        int N = 40;
        int Npar; // [ 23 (MPC parameters) + (initial state) + n (curvature points == N) ]

        // MPC
        int nPlanning = 1900; // number of points wanted from the planner
        
        // DYNAMIC PARAMETERS:
        double dRd = 1;
        double dRa = 0.3; 
        double Dr = 3152.3;
        double Df = 2785.4;
        double Cr = 1.6;
        double Cf = 1.6;
        double Br = 10.1507;
        double Bf = 10.8529;
        double u_r = 0.45;
        double Cd = 0.8727;
        double q_slip = 0.1;
        double p_long = 0.5;
        double q_n = 0.1;
        double q_mu = 0.1;
        double lambda = 1;
        double q_s = 1;

        // STATIC PARAMETERS:
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

        // Initial conditions evaluation
        vector<double> initial_conditions();

        // S prediction
        void s_prediction();
        Eigen::VectorXd predicted_s;
        double smax = 0;

        // Previous state
        Eigen::MatrixXd lastState;    // [x, y, heading, vx, vy, w]
        Eigen::MatrixXd lastCommands; // [delta, acc]

        // Previous solution
        Eigen::MatrixXd solStates;    // [n, mu, vx, vy, w]
        Eigen::MatrixXd solCommands;  // [diff_delta, diff_acc, delta, acc]

        // CASADI + IPOPT:
        void set_boundaries_IPOPT();
        void set_parameters_IPOPT();
        void solve_IPOPT();

        // FORCESPRO:

        // Aux:
        void get_solution();
        vector<double> vconcat(const vector<double>& x, const vector<double>& y);
        void printVec(vector<double> &input, int firstElements=0);
        Eigen::MatrixXd vector2eigen(vector<double> vect);


    public:

        MPC(const Params& params);

        void reconfigure(tailored_mpc::dynamicConfig& config);

        // Callbacks
        void stateCallback(const as_msgs::CarState::ConstPtr& msg);
        void plannerCallback(const as_msgs::ObjectiveArrayCurv::ConstPtr& msg);

        // Solve method
        void solve();

        // Structs declaration
        Boundaries bounds;
        IPOPT ipopt;

        // MPC
        double T = 0.05; // [s]

        // Planner's trajectory matrix 
        Eigen::MatrixXd planner; // [x, y, s, k, vx, vy, w, L, R]

        //Actual state of the car
        Eigen::VectorXd carState; // [x, y, theta, vx, vy, w, delta(steering), acc]

};


#endif