#ifndef PARAMS_MPC_HPP
#define PARAMS_MPC_HPP

#include <ros/ros.h>
#include <string>

using namespace std;

struct Params{

    // Constructor
    Params(const ros::NodeHandle &nh);

    struct Vehicle{
        int m; // Car's mass (without pilot)
        double width, longue; // Car's track width & length
        double Lf, Lr; // Longitudinal distance from CoG to front and rear wheels

        // TO DO: rest of parameters from MPC that are not in dynamic reconfig

    } vehicle;

    struct MPC{
        int T; // period of MPC (1/freq) [s]
        struct Topics{
            string commands; // Car Commands topic
            string state; // Car State topic

            // TO DO: rest of topics
            
        } topics;
    } mpc;
};



#endif