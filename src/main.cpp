#include <ros/ros.h>
#include "mpc.hh"
#include "utils/vis_tools.hh"
#include "utils/params.hh"
#include <signal.h>
#include <boost/thread/thread.hpp>

#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"

ros::Publisher pubCommands; // Commands publisher

void dynamicCallback(tailored_mpc::dynamicConfig &config, uint32_t level, MPC* mpc){

    ROS_WARN("MPC: Setting new dynamic parameters..");
	mpc->reconfigure(config);

}

void my_SIGhandler(int sig){

    if(ros::master::check()){
        as_msgs::CarCommands msgCommands;
        msgCommands.motor = -1.0; // no one will read this value, but what if..
        msgCommands.steering = 0.0;
        msgCommands.Mtv = 0.0;
        for(int i=0; i<5; i++){
            pubCommands.publish(msgCommands);
            ros::Duration(0.05).sleep();
        }
        ROS_ERROR("MPC says Goodbye :)");
    }
    ros::shutdown();
}

int main(int argc, char **argv) {

    // Init Node:
    ros::init(argc, argv, "tailored_mpc");

    // Handle Connections:
    ros::NodeHandle nh("~");

    // Signal handler for publishing 0s when dying
    signal(SIGINT,  my_SIGhandler); // override default ros sigint signal

    // Params object
    Params params = Params(&nh);

    // MPC object
    MPC mpc(&params);

    // Spawn another thread
    // boost::thread thread_subs(subscribing, &mpc);
    // ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>("~");

    // Visualization tools
    VisualizationTools rviz = VisualizationTools(&mpc, &params);

    // Publishers & Subscribers
    ros::Subscriber subState = nh.subscribe(params.mpc.topics.state, 1, &MPC::stateCallback, &mpc);
    ros::Subscriber subPlanner = nh.subscribe(params.mpc.topics.planner, 1, &MPC::plannerCallback, &mpc);
    ros::Subscriber subTro = nh.subscribe(params.mpc.topics.tro, 1, &MPC::troCallback, &mpc);
    ros::Subscriber subVel = nh.subscribe(params.mpc.topics.velocities, 1, &MPC::velsCallback, &mpc);
    pubCommands = nh.advertise<as_msgs::CarCommands>(params.mpc.topics.commands, 1);

        // DEBUG
    string timeTopic, exitFlagTopic, solutionTopic, velocityTopic;
    nh.param<string>("Topics/Debug/Time", timeTopic, "/AS/C/mpc/debug/time");
    nh.param<string>("Topics/Debug/ExitFlag", exitFlagTopic, "/AS/C/mpc/debug/exitflags");
    nh.param<string>("Topics/Debug/Velocity", velocityTopic, "/AS/C/mpc/debug/velocity");
    nh.param<string>("Topics/Debug/Solution", solutionTopic, "/AS/C/mpc/debug/solution");

    nh.param<int>("mission", mpc.mission, 0); // current mission

    bool debug;
    nh.param<bool>("Debug/Flag", debug, false);

    ros::Publisher pubTime = nh.advertise<std_msgs::Float32>(timeTopic, 1);
    ros::Publisher pubExitflag = nh.advertise<std_msgs::Int32>(exitFlagTopic, 1);
    ros::Publisher pubVel = nh.advertise<std_msgs::Float32>(velocityTopic, 1);
    ros::Publisher pubDebug = nh.advertise<as_msgs::MPCdebug>(solutionTopic, 1);

    // Dynamic reconfigure
	dynamic_reconfigure::Server<tailored_mpc::dynamicConfig> server;
	dynamic_reconfigure::Server<tailored_mpc::dynamicConfig>::CallbackType f;
	f = boost::bind(&dynamicCallback, _1, _2, &mpc);
	server.setCallback(f);

    ros::Duration(2.0).sleep();

    ROS_WARN_STREAM("MPC: publish frequency: " << mpc.Hz << "Hz");
    ROS_WARN_STREAM("MPC: internal threads: " << mpc.Nthreads);

    // Msgs declaration
    as_msgs::CarCommands msg;
    as_msgs::MPCdebug debug_msg;
    std_msgs::Float32 float_msg;
    std_msgs::Int32 exitflag_msg;

    ros::Rate r(mpc.Hz);
    // launch-prefix="gdb -ex run --args"
    while(ros::ok()){

        mpc.solve(); // Solve the NLOP

        mpc.msgCommands(&msg);
        if(mpc.forces.exit_flag == 1 || mpc.forces.exit_flag == 0 ) pubCommands.publish(msg); // publish car commands

        // DEBUG
        float_msg.data = mpc.elapsed_time.count()*1000;
        pubTime.publish(float_msg);

        exitflag_msg.data = mpc.forces.exit_flag;
        pubExitflag.publish(exitflag_msg);

        float_msg.data = mpc.pred_velocities(0);
        pubVel.publish(float_msg);

        if(debug){
            mpc.get_debug_solution(&debug_msg);
            pubDebug.publish(debug_msg);
        }

        rviz.rviz_predicted();  // visualize predicted states
        rviz.rviz_actual();     // visualize actual states

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}