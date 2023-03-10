#ifndef PARAMS_MPC_HPP
#define PARAMS_MPC_HPP

#include <ros/ros.h>
#include <string>

using namespace std;

struct Params{

    // Constructor
    Params(const ros::NodeHandle* nh);

    struct Vehicle{
        int m;                          // Car's mass (without pilot)
        double width, longue;           // Car's track width & length
        double Lf, Lr;                  // Longitudinal distance from CoG to front and rear wheels
        double d_IMU;                   // distance from IMU's sensor to CoG
        double Ar, rho;                 // aero area & air density
        double I;                       // moment of inertia (of the car)
        double gravity;
        double Rwheel;                  // effective wheel radius

    } vehicle;

    struct MPC{
        int Hz;                 // frequency of MPC [Hz]
        double rk4_t;           // runge kutta integration time [s]
        int Nthreads;           // number of threads
        int nPlanning;          // number of points we want from the planner
        bool TroProfile;        // set to true to follow TRO velocity profile 
        double minVelFinish;    // minimum velocity to consider the car at finish state (depends on sensor accuracy)
        struct Topics{
            string commands;            // Car Commands topic
            string state;               // Car State topic
            string planner;             // Planner topic
            string tro;                 // Offline planner topic
            string finish;              // Finish flag topic
            string predictedSteering;   // Visualization topics
            string predictedPath;
            string predictedHeading;
            string actualPath;
            
        } topics;
        struct NLOP{
            int N;          // Horizon length of the optimization problem 
            int Nslacks;    // Number of slack variables

        } nlop;
    } mpc;

};



#endif