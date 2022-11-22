#include <ros/ros.h>
#include "mpc.hh"
#include "utils/vis_tools.hh"
#include "utils/params.hh"
#include <signal.h>

#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"

ros::Publisher pubCommands; // Commands publisher

void dynamicCallback(tailored_mpc::dynamicConfig &config, uint32_t level, MPC* mpc){

    ROS_WARN("MPC: Setting new dynamic parameters..");
	mpc->reconfigure(config);

}

void my_SIGhandler(int sig){

	as_msgs::CarCommands msgCommands;
	msgCommands.motor = -1.0;
	msgCommands.steering = 0.0;
    msgCommands.Mtv = 0.0;
	for(int i=0; i<5; i++){ 
        pubCommands.publish(msgCommands);
        ros::Duration(0.05).sleep();
    }

    ROS_ERROR("MPC says Goodbye :)");
    ros::shutdown();
}

int main(int argc, char **argv) {

    // Init Node:
    ros::init(argc, argv, "tailored_mpc");

    // Handle Connections:
    ros::NodeHandle nh("~");

    // Signal handler for publishing 0s when dying
    signal(SIGINT,  my_SIGhandler); // override default ros sigint signal
    // signal(SIGTERM, my_SIGhandler);
    // signal(SIGKILL, my_SIGhandler);

    // Params object
    Params params = Params(&nh);

    // MPC object
    MPC mpc(&params);

    // Visualization tools
    VisualizationTools rviz = VisualizationTools(&mpc, &params);

    // Publishers & Subscribers
    ros::Subscriber subState = nh.subscribe(params.mpc.topics.state, 1, &MPC::stateCallback, &mpc);
    ros::Subscriber subPlanner = nh.subscribe(params.mpc.topics.planner, 1, &MPC::plannerCallback, &mpc);
    ros::Subscriber subTro = nh.subscribe(params.mpc.topics.tro, 1, &MPC::troCallback, &mpc);
    pubCommands = nh.advertise<as_msgs::CarCommands>(params.mpc.topics.commands, 1);

    // Debug
    string timeTopic, exitFlagTopic, CurvTopic;
    nh.param<string>("Topics/Debug/Time", timeTopic, "/AS/C/mpc/debug/time");
    nh.param<string>("Topics/Debug/ExitFlag", exitFlagTopic, "/AS/C/mpc/debug/exitflags");
    nh.param<string>("Topics/Debug/Curvature", CurvTopic, "/AS/C/mpc/debug/curvature");
    ros::Publisher pubTime = nh.advertise<std_msgs::Float32>(timeTopic, 10);
    ros::Publisher pubExitflag = nh.advertise<std_msgs::Int32>(exitFlagTopic, 10);
    ros::Publisher pubCurvature = nh.advertise<std_msgs::Float64MultiArray>(CurvTopic, 10);

    // Dynamic reconfigure
	dynamic_reconfigure::Server<tailored_mpc::dynamicConfig> server;
	dynamic_reconfigure::Server<tailored_mpc::dynamicConfig>::CallbackType f;
	f = boost::bind(&dynamicCallback, _1, _2, &mpc);
	server.setCallback(f);

    mpc.mission = 1; // mision hardcoded

    ros::Duration(2).sleep();

    ROS_INFO_STREAM("MPC: publish frequency: " << mpc.Hz << "Hz");
    ROS_WARN_STREAM("MPC: internal threads: " << mpc.Nthreads);

    // Msgs declaration
    as_msgs::CarCommands msg;
    std_msgs::Float32 time_msg;
    std_msgs::Int32 exitflag_msg;

    std_msgs::Float64MultiArray curv_msg;
    curv_msg.layout.dim.push_back(std_msgs::MultiArrayDimension()); // set up dimensions
    curv_msg.layout.dim[0].size = 2;     // 2 curvatures
    curv_msg.layout.dim[0].stride = 1;   // we have a stride of 1
    curv_msg.layout.dim[0].label = "curv"; 

    ros::Rate r(mpc.Hz);
    // launch-prefix="gdb -ex run --args"

    while(ros::ok()){

        curv_msg.data.clear();
        curv_msg.data.push_back(mpc.planner(0, 3)); // save raw curvature (just after callbacks have been called)

        mpc.solve(); // Solve the NLOP

        msg = as_msgs::CarCommands();
        mpc.msgCommands(&msg);
        if(mpc.forces.exit_flag == 1 || mpc.forces.exit_flag == 0) pubCommands.publish(msg); // publish car commands

        // DEBUG
        time_msg = std_msgs::Float32();
        time_msg.data = mpc.elapsed_time.count()*1000;
        pubTime.publish(time_msg);

        exitflag_msg = std_msgs::Int32();
        exitflag_msg.data = mpc.forces.exit_flag;
        pubExitflag.publish(exitflag_msg);

        
        curv_msg.data.push_back(mpc.planner(0, 3)); // save smooth curvature
        pubCurvature.publish(curv_msg);

        rviz.rviz_predicted();  // visualize predicted states
        rviz.rviz_actual();     // visualize actual states

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}