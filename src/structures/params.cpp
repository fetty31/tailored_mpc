#include "structures/params.hh"

Params::Params(const ros::NodeHandle &nh) {

    // Vehicle params
    nh.param<int>("/tailored_mpc/Vehicle/m", vehicle.m, 240);
    nh.param<double>("/tailored_mpc/Vehicle/longue", vehicle.longue, 2.72);
    nh.param<double>("/tailored_mpc/Vehicle/width", vehicle.width, 1.5);
    nh.param<double>("/tailored_mpc/Vehicle/Lf", vehicle.Lf, 0.708);
    nh.param<double>("/tailored_mpc/Vehicle/Lr", vehicle.Lr, 0.822);
    nh.param<double>("/tailored_mpc/Vehicle/ax_max", vehicle.ax_max, 5.5);
    nh.param<double>("/tailored_mpc/Vehicle/ax_min", vehicle.ax_min, -8);
    nh.param<double>("/tailored_mpc/Vehicle/d_IMU", vehicle.d_IMU, -0.318);
    nh.param<double>("/tailored_mpc/Vehicle/inertia", vehicle.I, 93);
    nh.param<double>("/tailored_mpc/Vehicle/gravity", vehicle.gravity, 9.81);
    nh.param<double>("/tailored_mpc/Vehicle/rho", vehicle.rho, 1.255);
    nh.param<double>("/tailored_mpc/Vehicle/Ar", vehicle.rho, 1.0);

    // Topics
    nh.param<string>("/tailored_mpc/Topics/State", mpc.topics.state, "/AS/C/state");
    nh.param<string>("/tailored_mpc/Topics/Commands", mpc.topics.commands, "/AS/C/commands");
    nh.param<string>("/tailored_mpc/Topics/Planner", mpc.topics.planner, "/AS/C/trajectory/partial");

    // NLOP
    nh.param<int>("/tailored_mpc/NLOP/Nstates", mpc.nlop.n_states, 7);
    nh.param<int>("/tailored_mpc/NLOP/Ncontrols", mpc.nlop.n_controls, 2);
    nh.param<int>("/tailored_mpc/NLOP/N", mpc.nlop.N, 40);
    nh.param<int>("/tailored_mpc/NLOP/Npar", mpc.nlop.Npar, 23);

    // FORCES
    nh.param<bool>("/tailored_mpc/FORCES", FORCES, true);

    // MPC period (1/freq)
    nh.param<double>(("/tailored_mpc/PredTime"), mpc.PredTime, 0.05);
    nh.param<int>(("/tailored_mpc/nPlanning"), mpc.nPlanning, 1900);

}