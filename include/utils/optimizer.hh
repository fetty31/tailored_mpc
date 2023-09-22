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
#include "utils/params.hh"

using namespace casadi;
using namespace std;

struct IPOPT{

    vector<double> lbx;        // Lower boundaries stages (x,u)
    vector<double> ubx;        // Upper boundaries stages (x,u)

    vector<double> lbg_next;   // Lower boundaries continuity constraints 
    vector<double> ubg_next;   // Upper boundaries continuity constraints

    vector<double> lbg_track;  // Lower boundaries track constraints
    vector<double> ubg_track;  // Upper boundaries track constraints

    vector<double> x0;         // Initial guess
    vector<double> p;          // Parameters + curvature
    vector<double> solution;   // Solution --> Optimized stages

    int exit_flag;

    shared_ptr<casadi::Function> solver_ptr; // Solver object

};

// optimizer class
class Optimizer{

    private:

        // Internal variables/methods of optimizer

        bool params_set = false;

            // Optimization variables
        SX X; // symbolic states vector
        SX U; // symbolic control vector
        SX P; // symbolic parameters vector

        SX obj = 0; // Objective function

        vector<SX> g; // Constraints vector

        void nlop_formulation(); // set Non Linear Optimization Problem (multiple shooting)
        void track_constraints(); // set track constraints

        vector<SX> continuous_dynamics( SX st, SX delta, SX k, SX vx);

    public:

        Optimizer(const Params* params); // Constructor

        int n_states;   // Number of states variables [delta, n, mu, Vy, w]
        int n_controls; // Number of controls variables [diffDelta, Mtv] = [d(delta)/dt, Mtv]
        int N;          // Horizon length for optimization problem 
        int Npar;       // Number of parameters for optimization problem [ 23 (MPC parameters) + (initial state) + n (curvature points == N) + vx (target velocity points == N)]

        // Aux variables
        int curv_idx0; // First idx where curvature values start at
        int vx_idx0;   // First idx where long. velocity values start at

            // Vehicle variables
        double m, Lf, Lr, width, longue; // see "params.hh" for explanation

        double T; // integration time [s]

        shared_ptr<Function> generate_solver(); // Initialize
        int get_ineq_size();

        // Solver options --> for IPOPT solver options, see http://casadi.sourceforge.net/v2.0.0/api/html/d6/d07/classcasadi_1_1NlpSolver.html#plugin_NlpSolver_ipopt
        //                                                  https://coin-or.github.io/Ipopt/OPTIONS.html

        Dict ipoptOptions = {{"max_iter",100},{"expect_infeasible_problem","no"},{"print_level", 1}}; 
        Dict solverOptions = {{"ipopt",ipoptOptions},{"print_time",true}};

};

#endif