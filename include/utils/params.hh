#ifndef PARAMS_MPC_HPP
#define PARAMS_MPC_HPP

#include <ros/ros.h>
#include <string>

using namespace std;

struct Params{

    // Constructor
    Params(const ros::NodeHandle* nh);

    struct Debug{
        string path;  // path where we will save debugging info
        bool flag;  // path where we will save debugging info
    } debug;

    struct Vehicle{
        int m;                          // Car's mass (without pilot)
        double width, longue;           // Car's track width & length
        double Lf, Lr;                  // Longitudinal distance from CoG to front and rear wheels
        double Ar, rho;                 // aero area & air density
        double I;                       // moment of inertia (of the car)
        double gravity;

    } vehicle;

    struct MPC{
        int Hz;                 // frequency of MPC [Hz]
        double rk4_t;           // runge kutta integration time [s]
        double delta_s;         // distance between planner points [m]
        int Nthreads;           // number of threads
        int nPlanning;          // number of points we want from the planner
        int nSearch;            // number of points where we will look for the closer point to the car
        struct Topics{
            string commands;            // Car Commands topic
            string state;               // Car State topic
            string planner;             // Planner topic
            string tro;                 // Offline planner topic
            string velocities;
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