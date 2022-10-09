#ifndef OPTIMIZER_MPC_HH
#define OPTIMIZER_MPC_HH

#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <fstream>
#include <chrono>
#include <string>
#include <vector>
#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream

#include "casadi/casadi.hpp"
#include "structures/params.hh"

using namespace casadi;
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

// optimizer class
class Optimizer{

    private:

        // Internal variables/methods of optimizer

        bool params_set = false;

        int n_states = 7; // Number of states variables [delta, acc, n, mu, Vx, Vy, w]
        int n_controls = 2; // Number of controls variables [diffDelta, diffAcc] = [d(delta)/dt, d(acc)/dt]
        int N; // Horizon length for optimization problem 
        int Npar; // Number of parameters for optimization problem [ 23 (MPC parameters) + (initial state) + n (curvature points == N) ]

            // Vehicle variables
        double m, Lf, Lr, width, longue, ax_max, ay_max; // see "params.hh" for explanation

        double T; // integration time [s]

            // Optimization variables
        SX X; // symbolic states vector
        SX U; // symbolic control vector
        SX P; // symbolic parameters vector

        SX obj = 0; // Objective function

        vector<SX> g; // Constraints vector

        void nlop_formulation(); // set Non Linear Optimization Problem (multiple shooting)
        void track_constraints(); // set track constraints
        void forces_constraints(); // set ellipse of forces constraints

        vector<SX> continuous_dynamics( SX st, SX con, SX k);
        vector<double> vconcat(const vector<double>& x, const vector<double>& y);
        void printVec(vector<double> &input, int firstElements=0);


    public:

        Optimizer(const Params &params); // Constructor

        shared_ptr<Function> generate_solver(); // Initialize

        // Solver options --> for IPOPT solver options, see http://casadi.sourceforge.net/v2.0.0/api/html/d6/d07/classcasadi_1_1NlpSolver.html#plugin_NlpSolver_ipopt
        //                                                  https://coin-or.github.io/Ipopt/OPTIONS.html

        Dict ipoptOptions = {{"max_iter",1000},{"expect_infeasible_problem","yes"},{"print_level", 1}}; 
        Dict solverOptions = {{"ipopt",ipoptOptions},{"print_time",true}};

};

#endif