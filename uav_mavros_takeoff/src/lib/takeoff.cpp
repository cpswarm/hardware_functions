#include "lib/takeoff.h"

takeoff::takeoff ()
{
	NodeHandle nh;

	// read parameters
	nh.getParam(ros::this_node::getName() + "/loop_rate", loop_rate);
	int queue_size;
	nh.getParam(ros::this_node::getName() + "/queue_size", queue_size);
	nh.getParam(ros::this_node::getName() + "/pos_tolerance", pos_tolerance);
	nh.getParam(ros::this_node::getName() + "/stabilize_time", stabilize_time);
	nh.getParam(ros::this_node::getName() + "/fcu", fcu);
	nh.getParam(ros::this_node::getName() + "/uav", uav);
	double lon, lat, alt;
	nh.getParam(ros::this_node::getName() + "/lon", lon);
	nh.getParam(ros::this_node::getName() + "/lat", lat);
	nh.getParam(ros::this_node::getName() + "/alt", alt);
	gps_home.x_lat = lat;
	gps_home.y_long = lon;
	gps_home.z_alt = alt;


	// initialize global variables
	position_received = false;

	// ros communication
	arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	apm_takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
	apm_mission_client = nh.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
	state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", queue_size, &takeoff::state_cb, this);
	pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("pos_provider/pose", queue_size, &takeoff::position_cb, this);
	goal_pub = nh.advertise<geometry_msgs::PoseStamped>("pos_controller/goal_position", queue_size);

	ros::Rate rate(loop_rate);

	// wait for fcu connection
	while (ros::ok() && !state.connected) {
		ROS_DEBUG_ONCE("TAKEOFF - Waiting for FCU connection...");
		ros::spinOnce();
		rate.sleep();
	}
	ROS_DEBUG("TAKEOFF - FCU connected");

	// wait for position information
	while (ros::ok() && !position_received) {
		ROS_DEBUG_ONCE("TAKEOFF - Waiting for local position...");
		ros::spinOnce();
		rate.sleep();
	}
	ROS_DEBUG("TAKEOFF - Local position received");
}

bool takeoff::execute (double altitude)
{
	// get current position
	ros::spinOnce();

	// the setpoint publishing rate must be faster than 2 hz on px4
	ros::Rate rate(loop_rate);

	goal_position.pose = position.pose;
	goal_position.pose.position.z += altitude;

	ROS_DEBUG("Take off to altitude %.2f", goal_position.pose.position.z);

	if (fcu == "px4") {
		ROS_DEBUG("TAKEOFF - Using PX4 FCU");

		// send a few setpoints (current position) before starting
		sleep(2);
		goal_pub.publish(position);
		sleep(2);

		// set autonomous mode
		if (set_mode("OFFBOARD") == false)
			return false;

		// arm vehicle
		if (arm() == false)
			return false;

		// take off in semi-autonomous mode
		if (set_mode("AUTO.TAKEOFF") == false)
			return false;
		sleep(1);

		// switch back to autonomous mode
		if (set_mode("OFFBOARD") == false)
			return false;

		// move to desired altitude
		goal_pub.publish(goal_position);
	}
	else if (fcu == "apm" && uav == "copter") {
		ROS_DEBUG("TAKEOFF - Using ArduCopter FCU");

		// set autonomous mode
		if (set_mode("GUIDED") == false)
			return false;

		// arm vehicle
		if (arm() == false)
			return false;

		// take off
		apm_takeoff_request.request.altitude = altitude;
		apm_takeoff_client.call(apm_takeoff_request);
	}
	// else if (fcu == "apm" && uav == "plane") {
	// 	ROS_DEBUG("TAKEOFF - Using ArduPlane FCU");

	// 	// set autonomous mode
	// 	if (set_mode("TAKEOFF") == false)
	// 		return false;

	// 	// arm vehicle
	// 	if (arm() == false)
	// 		return false;

	// 	// take off
	// 	apm_takeoff_request.request.min_pitch = 15.0;
	// 	apm_takeoff_request.request.yaw = 0.0;
	// 	apm_takeoff_request.request.latitude = 46.6128; // increased from 7 to 8
	// 	apm_takeoff_request.request.longitude = 14.2652;
	// 	apm_takeoff_request.request.altitude = altitude;
	// 	apm_takeoff_client.call(apm_takeoff_request);

	// 	if (apm_takeoff_request.response.success) {
	// 		ROS_INFO("Take off succeeded with result %d.", apm_takeoff_request.response.result);
	// 	}
	// 	else {
	// 		ROS_ERROR("Take off failed with result %d!", apm_takeoff_request.response.result);
	// 		return false;
	// 	}
	// }
	else if (fcu == "apm" && uav == "plane") {
		ROS_DEBUG("TAKEOFF - Using ArduPlane FCU");

		// takeoff mission
		apm_mission_request.request.start_index = 0;

		// initialize waypoint
		mavros_msgs::Waypoint waypoint;
		waypoint = gps_home;
		waypoint.is_current = false;
		waypoint.autocontinue = true;

		// home position
		waypoint.frame = 0; // relative altitude
		waypoint.command = 16; // waypoint: https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT
		waypoint.param1 = 0; // hold time in seconds
		waypoint.param2 = 0; // acceptance radius in meter
		waypoint.param3 = 0; // pass radius in meter
		waypoint.param4 = 0; // yaw angle in degrees
		apm_mission_request.request.waypoints.push_back(waypoint);

		// take off
		waypoint.frame = 3; // relative altitude
		waypoint.command = 22; // take off: https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_TAKEOFF
		waypoint.param1 = 15.0; // minimum pitch in degrees
		waypoint.param2 = 0; // empty
		waypoint.param3 = 0; // empty
		waypoint.param4 = 0; // yaw angle in degrees
		waypoint.z_alt = altitude;
		apm_mission_request.request.waypoints.push_back(waypoint);

		// loiter
		waypoint.frame = 3; // relative altitude
		waypoint.command = 17; // loiter: https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LOITER_UNLIM
		waypoint.param1 = 0; // empty
		waypoint.param2 = 0; // empty
		waypoint.param3 = 10; // radius in meter
		waypoint.param4 = 0; // yaw angle in degrees
		waypoint.z_alt = altitude;
		apm_mission_request.request.waypoints.push_back(waypoint);

		// send mission to fcu
		apm_mission_client.call(apm_mission_request);
		if (apm_mission_request.response.success) {
			ROS_INFO("Successfully transferred %d waypoints.", apm_mission_request.response.wp_transfered);
		}
		else {
			ROS_ERROR("Failed to transfer %d waypoints!", apm_mission_request.response.wp_transfered);
			return false;
		}

		// arm vehicle
		if (arm() == false)
			return false;

		// set mission mode
		if (set_mode("AUTO") == false)
			return false;
	}

	else {
		ROS_FATAL("TAKEOFF - Unknown FCU firmware, cannot perform takeoff");
		return false;
	}

	// everything worked as expected
	return true;
}

bool takeoff::wait ()
{
	// repeat take of request for ardupilot copter
	if (fcu == "apm" && uav == "copter" && apm_takeoff_request.response.success == false) {
		apm_takeoff_client.call(apm_takeoff_request);
	}

	// altitude not yet reached
	if (abs(goal_position.pose.position.z - position.pose.position.z) > pos_tolerance) {
		return false;
	}

	// altitude reached, stabilize
	else {
		ROS_DEBUG("TAKEOFF - Waiting %.2f seconds to stabilize", stabilize_time);
		sleep(stabilize_time);
		return true;
	}
}

bool takeoff::arm ()
{
	// initialize message
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;

	// arm uav
	if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
		ROS_INFO("TAKEOFF - Vehicle ARMED");
		return true;
	}
	else {
		ROS_ERROR("TAKEOFF - Failed to arm vehicle");
		return false;
	}
}

bool takeoff::set_mode (string mode)
{
	// initialize mode message
	mavros_msgs::SetMode set_mode;
	set_mode.request.custom_mode = mode;

	// set mode
	if (set_mode_client.call(set_mode) && set_mode.response.mode_sent) {
		ROS_INFO("TAKEOFF - Changed to %s mode", mode.c_str());
		return true;
	} else {
		ROS_ERROR("TAKEOFF - Failed to change to %s mode", mode.c_str());
		return false;
	}
}

void takeoff::position_cb (const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	position = *msg;
	position_received = true;
	ROS_DEBUG_ONCE("TAKEOFF - Got local position: (%.2f,%.2f,%.2f)", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void takeoff::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	state = *msg;
}
