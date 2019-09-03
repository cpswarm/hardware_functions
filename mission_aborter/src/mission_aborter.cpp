/*
 * mission_aborter.cpp
 *
 *  Created on: Sep 23, 2018
 *      Author: coriasco
 */




#include <ros/ros.h>
#include <swarmros/SimpleEvent.h>
#include <mavros_msgs/SetMode.h>
#include <string>
#include <stdint.h>

using namespace std;

uint8_t aborting;

void mission_abort_callback(swarmros::SimpleEvent msg) {
	aborting = 1;
	ROS_INFO("Aborting");
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "UWB_node");
	ros::NodeHandle n;
	mavros_msgs::SetMode land_set_mode;
	string mission_abort_topic = "bridge/events/mission_abort";
	string set_mode_topic = ros::this_node::getNamespace() + "/mavros/set_mode";

	ros::Subscriber mission_abort_pub = n.subscribe(mission_abort_topic, 10, mission_abort_callback);
	ros::ServiceClient set_mode_client = n.serviceClient <mavros_msgs::SetMode> (set_mode_topic);

	land_set_mode.request.custom_mode = "AUTO.LAND";

	ros::Rate loop_rate(10);

	while (ros::ok()) {
		if(aborting) {
			set_mode_client.call(land_set_mode);
//			if(land_set_mode.response.mode_sent) {
//				aborting = 0;
//			}
		}

		ros::spinOnce();

		loop_rate.sleep();
	}
}
