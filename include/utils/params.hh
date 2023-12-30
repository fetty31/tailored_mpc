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
            int Nh;         // Number of inequality constraints
            int n_states;   // Number of state variables
            int n_controls; // Number of control variables
            int Npar;       // Number of parameters for optimization problem [ 23 (MPC parameters) + (initial state) + n (curvature points == N) ]
            int Nslacks;    // Number of slack variables

        } nlop;
    } mpc;

};



#endif