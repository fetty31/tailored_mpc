#include "utils/vis_tools.hh"

VisualizationTools::VisualizationTools(MPC *mpc, const Params *params){

    this->mpc = mpc;

    // RVIZ Solution MPC
    pubPredictedPath = n.advertise<nav_msgs::Path>(params->mpc.topics.predictedPath, 1);
    pubPredictedHeading = n.advertise<visualization_msgs::MarkerArray>(params->mpc.topics.predictedHeading, 1);
    pubPredictedSteering = n.advertise<visualization_msgs::MarkerArray>(params->mpc.topics.predictedSteering, 1);

	// DEBUG
    debugPointsPath = n.advertise<visualization_msgs::MarkerArray>("/vis/predicted/points",1);

    // RVIZ Planner
	pubActualPath = n.advertise<nav_msgs::Path>(params->mpc.topics.actualPath, 1);

}

VisualizationTools::~VisualizationTools(){}

visualization_msgs::MarkerArray VisualizationTools::getMarkersHeading(Eigen::MatrixXd &state){

    size_t id = 0;

	// Init message:
	visualization_msgs::Marker marker;
	visualization_msgs::MarkerArray markerArray;

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "global";

	marker.action= visualization_msgs::Marker::ADD;
	marker.type = visualization_msgs::Marker::ARROW;

	tf2::Quaternion q;
	
    for(unsigned int i = 0; i < state.rows(); i++ ){

		marker.ns= "heading";
		marker.id = id++;

		q.setRPY( 0, 0, state(i, 2) );

		marker.pose.position.x = state(i, 0);
		marker.pose.position.y = state(i, 1);
		marker.pose.position.z = 0.25;

		marker.pose.orientation.x = q[0];
		marker.pose.orientation.y = q[1];
		marker.pose.orientation.z = q[2];
		marker.pose.orientation.w = q[3];
		
		marker.scale.x = 0.3;   // shaft length
		marker.scale.y = 0.05;  // head diameter
		marker.scale.z = 0.05;  // head length

		marker.color.r = 0.0f;
		marker.color.g = 0.0f;
		marker.color.b = 1.0f;
		marker.color.a = 1.0;

        markerArray.markers.push_back(marker);
        
    }

	return markerArray;

}

visualization_msgs::MarkerArray VisualizationTools::getMarkersSteering(Eigen::MatrixXd &commands, Eigen::MatrixXd &state){

    size_t id = 0;

	// Init message:
	visualization_msgs::Marker marker;
	visualization_msgs::MarkerArray markerArray;

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "global";

	marker.action= visualization_msgs::Marker::ADD;
	marker.type = visualization_msgs::Marker::ARROW;

	tf2::Quaternion q;
	
    for(unsigned int i = 1; i < state.rows()-1; i++ ){

		marker.ns= "steering";
		marker.id = id++;

		q.setRPY( 0, 0, state(i, 2) + commands(i, 2)*3.9);

		marker.pose.position.x = state(i, 0);
		marker.pose.position.y = state(i, 1);
		marker.pose.position.z = 0.50;

		marker.pose.orientation.x = q[0];
		marker.pose.orientation.y = q[1];
		marker.pose.orientation.z = q[2];
		marker.pose.orientation.w = q[3];
		
		marker.scale.x = 0.3;   // shaft length
		marker.scale.y = 0.05;  // head diameter
		marker.scale.z = 0.05;  // head length

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        markerArray.markers.push_back(marker);
        
    }

	return markerArray;

}

visualization_msgs::MarkerArray VisualizationTools::getMarkersPoint(Eigen::MatrixXd &state){
    
    size_t id = 0;

    // Init message:
	visualization_msgs::Marker marker;
	visualization_msgs::MarkerArray markerArray;

    marker.header.stamp     = ros::Time::now();
    marker.header.frame_id  = "global";

	marker.action= visualization_msgs::Marker::ADD;
	marker.type = visualization_msgs::Marker::SPHERE_LIST;
	
	// POINTS markers use x and y scale for width/height respectively
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.pose.orientation.w = 1.0;
	
	geometry_msgs::Point point;
    for(unsigned int i = 0; i < state.rows(); i++ ){

	    marker.ns = "spheres";
        marker.id = id++;

        point.x = state(i, 0);
        point.y = state(i, 1);

        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

		// Delete the point from previous iteration in the for loop
		marker.points.clear();
		marker.points.push_back(point);

        markerArray.markers.push_back(marker);
        
    }

    return markerArray;

}

nav_msgs::Path VisualizationTools::getPath(Eigen::MatrixXd &matrix){

    nav_msgs::Path pathMsg;
	geometry_msgs::PoseStamped pose;

    pathMsg.header.stamp    = ros::Time::now();
    pathMsg.header.frame_id = "global";

    for(unsigned int i = 0; i < matrix.rows(); i++){

        pose.pose.position.x = matrix(i, 0);
        pose.pose.position.y = matrix(i, 1);

        pathMsg.poses.push_back(pose);
        
    }

    return pathMsg;
    
}

