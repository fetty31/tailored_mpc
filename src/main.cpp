#include <ros/ros.h>
#include "mpc.hh"
#include "utils/vis_tools.hh"
#include "utils/params.hh"
#include <signal.h>

#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"

ros::Publisher pubCommands; // Commands publisher

void dynamicCallback(tailored_mpc::dynamicConfig &config, uint32_t level, MPC* mpc){

    ROS_WARN("MPC: Setting new dynamic parameters..");
	mpc->reconfigure(config);

}

void my_handler(int sig){

    ROS_ERROR("MPC says Goodbye :)");
	as_msgs::CarCommands msgCommands;
	msgCommands.motor = 0.0;
	msgCommands.steering = 0.0;
    msgCommands.Mtv = 0.0;
	pubCommands.publish(msgCommands);
    ros::shutdown();
}

int main(int argc, char **argv) {

    // Init Node:
    ros::init(argc, argv, "tailored_mpc");

	struct sigaction sigIntHandler;
    
    // Signal handler for publishing 0s when dying
    sigIntHandler.sa_handler = my_handler; 
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    // Handle Connections:
    ros::NodeHandle nh;

    // Params object
    Params params = Params(nh);

    // MPC object
    MPC mpc(params);

    // Visualization tools
    VisualizationTools rviz = VisualizationTools(&mpc, &params);

    // Publishers & Subscribers
    ros::Subscriber subState = nh.subscribe(params.mpc.topics.state, 1, &MPC::stateCallback, &mpc);
    ros::Subscriber subPlanner = nh.subscribe(params.mpc.topics.planner, 1, &MPC::plannerCallback, &mpc);
    ros::Subscriber subTro = nh.subscribe(params.mpc.topics.tro, 1, &MPC::troCallback, &mpc);
    pubCommands = nh.advertise<as_msgs::CarCommands>(params.mpc.topics.commands, 1);

    string timeTopic, exitFlagTopic;
    nh.param<string>("/tailored_mpc/Topics/Debug/Time", timeTopic, "/AS/C/mpc/debug/time");
    nh.param<string>("/tailored_mpc/Topics/Debug/ExitFlag", exitFlagTopic, "/AS/C/mpc/debug/exitflags");
    ros::Publisher pubTime = nh.advertise<std_msgs::Float32>(timeTopic, 10);
    ros::Publisher pubExitflag = nh.advertise<std_msgs::Int32>(exitFlagTopic, 10);

    // Dynamic reconfigure
	dynamic_reconfigure::Server<tailored_mpc::dynamicConfig> server;
	dynamic_reconfigure::Server<tailored_mpc::dynamicConfig>::CallbackType f;
   
	f = boost::bind(&dynamicCallback, _1, _2, &mpc);
	server.setCallback(f);

    mpc.mission = 1; // mision hardcoded

    ros::Duration(2).sleep();

    ROS_INFO_STREAM("MPC: publish frequency: " << mpc.Hz << "Hz");
    ROS_WARN_STREAM("MPC: internal threads: " << mpc.Nthreads);

    ros::Rate r(mpc.Hz);
    // launch-prefix="gdb -ex run --args"

    while(ros::ok()){

        mpc.solve(); // Solve the NLOP

        as_msgs::CarCommands msg;
        mpc.msgCommands(&msg);
        pubCommands.publish(msg); // publish car commands

        // DEBUG
        std_msgs::Float32 time_msg;
        time_msg.data = mpc.elapsed_time.count()*1000;
        pubTime.publish(time_msg);

        std_msgs::Int32 exitflag_msg;
        exitflag_msg.data = mpc.forces.exit_flag;
        pubExitflag.publish(exitflag_msg);


        rviz.rviz_predicted();  // visualize predicted states
        rviz.rviz_actual();     // visualize actual states

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}