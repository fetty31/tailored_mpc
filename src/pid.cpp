#include <ros/ros.h>
#include <signal.h>
#include "longitudinal.hh"

ros::Publisher pubCommands; // Commands publisher

// void dynamicCallback(tailored_mpc::longConfig &config, uint32_t level, PID* pid){

//     ROS_WARN("PID: Setting new dynamic parameters..");
// 	pid->reconfigure(config);

// }

void my_SIGhandler(int sig){

	as_msgs::CarCommands msgCommands;
	msgCommands.motor = -1.0;
	msgCommands.steering = 0.0;
    msgCommands.Mtv = 0.0;
	for(int i=0; i<5; i++){
        pubCommands.publish(msgCommands);
        ros::Duration(0.05).sleep();
    }
    ROS_ERROR("PID says Goodbye :)");
    ros::shutdown();
}

int main(int argc, char **argv) {

    // Init Node:
    ros::init(argc, argv, "pid");

    // Handle Connections:
    ros::NodeHandle nh;

    // Signal handler for publishing 0s when dying
    signal(SIGINT,  my_SIGhandler); // override default ros sigint signal

    // Get params
    string stateTopic, plannerTopic, troTopic, commandsTopic;
    nh.param<string>("/tailored_mpc/Topics/State", stateTopic, "/AS/C/state");
    nh.param<string>("/tailored_mpc/Topics/Planner", plannerTopic, "/AS/C/trajectory/partial");
    nh.param<string>("/tailored_mpc/Topics/Tro", troTopic, "/AS/C/trajectory/full");
    nh.param<string>("/tailored_mpc/PID/Topics/Commands", commandsTopic, "/AS/C/motor");

    int freq;
    nh.param<int>("/tailored_mpc/PID/Hz", freq, 40);

    // Longitudinal controller object
    PID pid;

    // Publishers & Subscribers
    ros::Subscriber subState = nh.subscribe(stateTopic, 1, &PID::stateCallback, &pid);
    ros::Subscriber subPlanner = nh.subscribe(plannerTopic, 1, &PID::plannerCallback, &pid);
    ros::Subscriber subTro = nh.subscribe(troTopic, 1, &PID::plannerCallback, &pid);
    pubCommands = nh.advertise<as_msgs::CarCommands>(commandsTopic, 1);

    // Dynamic reconfigure
	// dynamic_reconfigure::Server<tailored_mpc::longConfig> server;
	// dynamic_reconfigure::Server<tailored_mpc::longConfig>::CallbackType f;
	// f = boost::bind(&dynamicCallback, _1, _2, &pid);
	// server.setCallback(f);

    // Msg declaration
    as_msgs::CarCommands msg;

    ros::Rate r(freq);
    while(ros::ok()){

        pid.run(); // Solve the NLOP

        pid.msgCommands(&msg);
        pubCommands.publish(msg); // publish car commands

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}