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

	// initialize global variables
	position_received = false;

	// ros communication
	arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
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

	// send a few setpoints (current position) before starting
	geometry_msgs::PoseStamped goal_position;
	if (fcu == "px4") {
		goal_position.pose.position.x = position.pose.position.x;
		goal_position.pose.position.y = position.pose.position.y;
		goal_position.pose.position.z = position.pose.position.z;
		goal_position.pose.orientation = position.pose.orientation;
		goal_pub.publish(goal_position);
		sleep(2);
	}

	// enable offboard mode
	if (set_offboard() == false)
		return false;

	// arm vehicle
	if (arm() == false)
		return false;

	// take off
	goal_position.pose.position.z = goal_position.pose.position.z + altitude;
	if (fcu == "px4") {
		ROS_DEBUG("TAKEOFF - Using PX4 FCU");
		goal_pub.publish(goal_position);
	}
	if (fcu == "apm") {
		ROS_DEBUG("TAKEOFF - Using Ardupilot FCU");
		mavros_msgs::CommandTOL takeoff_request;
		takeoff_request.request.altitude = altitude;
		while (!takeoff_request.response.success) {
			takeoff_client.call(takeoff_request);
			ros::spinOnce();
			rate.sleep();
		}
	}

	// wait until take off completed
	while (abs(goal_position.pose.position.z - position.pose.position.z) > pos_tolerance) {
		ros::spinOnce();
		rate.sleep();
	}

	// stabilize position
	ROS_DEBUG("TAKEOFF - Waiting %.2f seconds to stabilize", stabilize_time);
	sleep(stabilize_time);
	ros::spinOnce();

    ROS_DEBUG("TAKEOFF - Completed, reached position (%.2f,%.2f,%.2f)", position.pose.position.x, position.pose.position.y, position.pose.position.z);
	return true;
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

bool takeoff::set_offboard ()
{
	// initialize mode message
	mavros_msgs::SetMode set_offboard;
	if (fcu == "px4") {
		set_offboard.request.custom_mode = "OFFBOARD";
	}
	else if (fcu == "apm") {
		set_offboard.request.custom_mode = "GUIDED";
	}
	else {
		ROS_FATAL("TAKEOFF - Unknown FCU firmware, cannot perform takeoff");
		return false;
	}

	// set mode to offboard
	if (set_mode_client.call(set_offboard) && set_offboard.response.mode_sent) {
		ROS_INFO("TAKEOFF - Offboard enabled");
		return true;
	} else {
		ROS_ERROR("TAKEOFF - Failed to set offboard mode");
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
