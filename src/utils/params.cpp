#include "utils/params.hh"

Params::Params(const ros::NodeHandle* nh) {

    // Vehicle params
    nh->param<int>("Vehicle/m", vehicle.m, 240);
    nh->param<double>("Vehicle/longue", vehicle.longue, 2.72);
    nh->param<double>("Vehicle/width", vehicle.width, 1.5);
    nh->param<double>("Vehicle/Lf", vehicle.Lf, 0.708);
    nh->param<double>("Vehicle/Lr", vehicle.Lr, 0.822);
    nh->param<double>("Vehicle/d_IMU", vehicle.d_IMU, -0.318);
    nh->param<double>("Vehicle/inertia", vehicle.I, 93);
    nh->param<double>("Vehicle/gravity", vehicle.gravity, 9.81);
    nh->param<double>("Vehicle/rho", vehicle.rho, 1.255);
    nh->param<double>("Vehicle/Ar", vehicle.Ar, 1.0);
    nh->param<double>("Vehicle/Rwheel", vehicle.Rwheel, 0.2);

    // Topics
    nh->param<string>("Topics/State", mpc.topics.state, "/AS/C/state");
    nh->param<string>("Topics/Commands", mpc.topics.commands, "/AS/C/commands");
    nh->param<string>("Topics/Planner", mpc.topics.planner, "/AS/C/trajectory/partial");
    nh->param<string>("Topics/Tro", mpc.topics.tro, "/AS/C/trajectory/full");
    nh->param<string>("Topics/Finish", mpc.topics.finish, "/AS/C/flags/finish");

        // Visualization topics
    nh->param<string>("Topics/Vis/PredictedSteering", mpc.topics.predictedSteering, "/AS/C/mpc/vis/predicted/steering");
    nh->param<string>("Topics/Vis/PredictedPath", mpc.topics.predictedPath, "/AS/C/mpc/vis/predicted/path");
    nh->param<string>("Topics/Vis/PredictedHeading", mpc.topics.predictedHeading, "/AS/C/mpc/vis/predicted/heading");
    nh->param<string>("Topics/Vis/ActualPath", mpc.topics.actualPath, "/AS/C/mpc/vis/actual/path");

    // NLOP
    nh->param<int>("NLOP/N", mpc.nlop.N, 40);
    nh->param<int>("NLOP/Nslacks", mpc.nlop.Nslacks, 2);

    // MPC aux vars 
    nh->param<double>(("rk4_t"), mpc.rk4_t, 0.025);
    nh->param<double>(("plannerAccuracy"), mpc.delta_s, 0.025);
    nh->param<int>(("nPlanning"), mpc.nPlanning, 1900);
    nh->param<int>(("Hz"), mpc.Hz, 20);
    nh->param<int>(("Nthreads"), mpc.Nthreads, 2);
    nh->param<bool>(("TroProfile"), mpc.TroProfile, false);
    nh->param<int>(("nSearchAhead"), mpc.nSearch, 5);
    mpc.nSearch = (int) mpc.nSearch/mpc.delta_s;

}