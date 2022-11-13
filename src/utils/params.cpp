#include "utils/params.hh"

Params::Params(const ros::NodeHandle &nh) {

    // Vehicle params
    nh.param<int>("/tailored_mpc/Vehicle/m", vehicle.m, 240);
    nh.param<double>("/tailored_mpc/Vehicle/longue", vehicle.longue, 2.72);
    nh.param<double>("/tailored_mpc/Vehicle/width", vehicle.width, 1.5);
    nh.param<double>("/tailored_mpc/Vehicle/Lf", vehicle.Lf, 0.708);
    nh.param<double>("/tailored_mpc/Vehicle/Lr", vehicle.Lr, 0.822);
    nh.param<double>("/tailored_mpc/Vehicle/d_IMU", vehicle.d_IMU, -0.318);
    nh.param<double>("/tailored_mpc/Vehicle/inertia", vehicle.I, 93);
    nh.param<double>("/tailored_mpc/Vehicle/gravity", vehicle.gravity, 9.81);
    nh.param<double>("/tailored_mpc/Vehicle/rho", vehicle.rho, 1.255);
    nh.param<double>("/tailored_mpc/Vehicle/Ar", vehicle.Ar, 1.0);
    nh.param<double>("/tailored_mpc/Vehicle/Rwheel", vehicle.Rwheel, 0.2);

    // Topics
    nh.param<string>("/tailored_mpc/Topics/State", mpc.topics.state, "/AS/C/state");
    nh.param<string>("/tailored_mpc/Topics/Commands", mpc.topics.commands, "/AS/C/commands");
    nh.param<string>("/tailored_mpc/Topics/Planner", mpc.topics.planner, "/AS/C/trajectory/partial");
    nh.param<string>("/tailored_mpc/Topics/Tro", mpc.topics.tro, "/AS/C/trajectory/full");
        // Visualization topics
    nh.param<string>("/tailored_mpc/Topics/Vis/PredictedSteering", mpc.topics.predictedSteering, "/AS/C/mpc/vis/predicted/steering");
    nh.param<string>("/tailored_mpc/Topics/Vis/PredictedPath", mpc.topics.predictedPath, "/AS/C/mpc/vis/predicted/path");
    nh.param<string>("/tailored_mpc/Topics/Vis/PredictedHeading", mpc.topics.predictedHeading, "/AS/C/mpc/vis/predicted/heading");
    nh.param<string>("/tailored_mpc/Topics/Vis/ActualPath", mpc.topics.actualPath, "/AS/C/mpc/vis/actual/path");

    // NLOP
    nh.param<int>("/tailored_mpc/NLOP/N", mpc.nlop.N, 40);

    // MPC period (1/freq)
    nh.param<double>(("/tailored_mpc/rk4_t"), mpc.rk4_t, 0.025);
    nh.param<int>(("/tailored_mpc/nPlanning"), mpc.nPlanning, 1900);
    nh.param<int>(("/tailored_mpc/Hz"), mpc.Hz, 20);
    nh.param<int>(("/tailored_mpc/Nthreads"), mpc.Nthreads, 2);
    nh.param<bool>(("/tailored_mpc/TroProfile"), mpc.TroProfile, false);

}