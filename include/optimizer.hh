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
#include <eigen3/Eigen/Dense>

#include "casadi/casadi.hpp"
#include "structures/params.hh"

using namespace casadi;
using namespace std;
using namespace Eigen;

// optimizer class
class optimizer{

    private:

        // Internal variables/methods of optimizer

        int n_states = 7; // Number of states variables [delta, acc, n, mu, Vx, Vy, w]
        int n_controls = 2; // Number of controls variables [diffDelta, diffAcc] = [d(delta)/dt, d(acc)/dt]
        int N; // Horizon length for optimization problem 
        int Npar; // Number of parameters for optimization problem [ 23 (MPC parameters) + 7 (initial state) + n (curvature points == N) ]

        double T; // period of MPC [s]

        // Optimization variables
        SX X; // symbolic states vector
        SX U; // symbolic control vector
        SX P; // symbolic parameters vector

        SX obj = 0; // Objective function

        vector<SX> g; // Constraints vector

        void nlop_formulation(); // set Non Linear Optimization Problem (multiple shooting)
        void track_constraints(); // set track constraints
        void forces_constraints(); // set ellipse of forces constraints

        vector<SX> continuous_dynamics( SX st, SX con, SX P, SX k);
        vector<double> vconcat(const vector<double>& x, const vector<double>& y);
        void printVec(vector<double> &input, int firstElements=0);


    public:

        optimizer(const Params &params); // Constructor
        Function solver; // solver object

        // Solver options --> for IPOPT solver options, see http://casadi.sourceforge.net/v2.0.0/api/html/d6/d07/classcasadi_1_1NlpSolver.html#plugin_NlpSolver_ipopt
        //                                                  https://coin-or.github.io/Ipopt/OPTIONS.html

        Dict ipoptOptions = {{"max_iter",5000},{"expect_infeasible_problem","yes"},{"print_level", 5}}; 
        Dict solverOptions = {{"ipopt",ipoptOptions},{"print_time",true}};

};

#endif