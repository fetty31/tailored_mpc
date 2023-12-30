/*
 Copyright (c) 2023 Oriol Martínez @fetty31

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef VIS_TOOLS_HH
#define VIS_TOOLS_HH

#include <ros/ros.h>
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Path.h"
#include <tf2/LinearMath/Quaternion.h>

#include "params.hh"
#include "../mpc.hh"

using namespace std;

class VisualizationTools {

    private:

        MPC *mpc;

        ros::Publisher pubActualPath;
        ros::Publisher pubPredictedPath, pubPredictedHeading, pubPredictedSteering;
        ros::Publisher debugPointsPath;

        nav_msgs::Path getPath(Eigen::MatrixXd &matrix);

        visualization_msgs::MarkerArray getMarkersHeading(Eigen::MatrixXd &matrix);
        visualization_msgs::MarkerArray getMarkersSteering(Eigen::MatrixXd &commands, Eigen::MatrixXd &state);
        visualization_msgs::MarkerArray getMarkersPoint(Eigen::MatrixXd &matrix);

    public:

        // Node Handler
    	ros::NodeHandle n;

        VisualizationTools(MPC *mpc, const Params *params);
        ~VisualizationTools();

        void rviz_predicted(){
            visualization_msgs::MarkerArray markerArray;
            nav_msgs::Path pathMsg;

            // Heading
            markerArray = getMarkersHeading(mpc->lastState);
            pubPredictedHeading.publish(markerArray);

            // Steering
            markerArray = getMarkersSteering(mpc->solCommands, mpc->lastState);
            pubPredictedSteering.publish(markerArray);

            // DEBUG
            markerArray = getMarkersPoint(mpc->lastState);
            debugPointsPath.publish(markerArray);

            // Path
            pathMsg = getPath(mpc->lastState);
            pubPredictedPath.publish(pathMsg);


        };

        void rviz_actual(){

            visualization_msgs::MarkerArray markerArray;
            nav_msgs::Path pathMsg;

            // Path
            pathMsg = getPath(mpc->planner);
            pubActualPath.publish(pathMsg);

        };

};

#endif