#include "optimizer.hh"
#include "mpc.hh"
#include "structures/params.hh"
#include <ros/ros.h>

void dynamicCallback(tailored_mpc::dynamicConfig &config, uint32_t level, MPC* mpc){

    ROS_WARN("MPC: Setting new dynamic parameters..");
	mpc->reconfigure(config);

}

int main(int argc, char **argv) {

    // Init Node:
    ros::init(argc, argv, "tailored_mpc");

    // Handle Connections:
    ros::NodeHandle nh;

    // Params object
    Params params = Params(nh);

    // Optimizer object
    Optimizer opt(params);
    shared_ptr<Function> solverPtr = opt.generate_solver();
    
    // MPC object
    MPC mpc(params);
    mpc.ipopt.solver_ptr = solverPtr;

    // Publishers & Subscribers
    ros::Subscriber subState = nh.subscribe(params.mpc.topics.state, 1, &MPC::stateCallback, &mpc);
    ros::Subscriber subPlanner = nh.subscribe(params.mpc.topics.planner, 1, &MPC::plannerCallback, &mpc);

    // Dynamic reconfigure
	dynamic_reconfigure::Server<tailored_mpc::dynamicConfig> server;
	dynamic_reconfigure::Server<tailored_mpc::dynamicConfig>::CallbackType f;
   
	f = boost::bind(&dynamicCallback, _1, _2, &mpc);
	server.setCallback(f);

    // ros::Rate r(1/mpc.T);

    ros::Rate r(20);
    while(ros::ok()){

        mpc.solve();

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}