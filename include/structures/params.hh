#ifndef PARAMS_MPC_HPP
#define PARAMS_MPC_HPP

#include <ros/ros.h>
#include <string>

using namespace std;

struct Params{

    // Constructor
    Params(const ros::NodeHandle &nh);

    struct Vehicle{
        int m;                          // Car's mass (without pilot)
        double width, longue;           // Car's track width & length
        double Lf, Lr;                  // Longitudinal distance from CoG to front and rear wheels
        double ax_max, ax_min;          // Maximum/Minimum longitudinal acceleration 
        double d_IMU;                   // distance from IMU's sensor to CoG
        double Ar, rho;                 // aero area & air density
        double I;                       // moment of inertia (of the car)
        double gravity;

    } vehicle;

    struct MPC{
        int Hz;                 // frequency of MPC [Hz]
        double rk4_t;           // runge kutta integration time [s]
        int nPlanning;          // number of points we want from the planner
        struct Topics{
            string commands;        // Car Commands topic
            string state;           // Car State topic
            string planner;         // Planner topic
            
        } topics;
        struct NLOP{
            int n_states;           // Number of state variables [delta, acc, n, mu, Vx, Vy, w]
            int n_controls;         // Number of controls variables [diffDelta, diffAcc] = [d(delta)/dt, d(acc)/dt]
            int N;                  // Horizon length of the optimization problem 
            int Npar;               // Number of parameters for optimization problem [ 23 (MPC parameters) + (initial state) + n (curvature points == N) ]

        } nlop;
    } mpc;

    bool FORCES;                // FORCESPRO flag (true meaning we are using FORCESPRO's solver, false for using IPOPT)
};



#endif