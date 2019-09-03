#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <uav_mavros_takeoff/TakeOff.h>
#include <math.h>

using namespace std;

// Variables
ros::Publisher goal_pos_pub;

ros::Subscriber state_sub;
ros::Subscriber local_pos_sub;

ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped local_position;
bool local_position_received = false;
double pos_tolerance;
double freq;
int stabilize_time;
int takeoff_steps;

//**********************************************************************************************************
bool isAtAltitude(double altitude, double tolerance) {
	double distance = abs(altitude - local_position.pose.position.z);
	ROS_DEBUG_THROTTLE(1, "TAKEOFF - Distance: %.4f", distance);
	return distance < tolerance;
}

void setMode() {
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.base_mode = 0;
	offb_set_mode.request.custom_mode = "OFFBOARD";

	if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
		ROS_INFO("TAKEOFF - OFFBOARD enabled");
	} else {
		ROS_ERROR("TAKEOFF - Failed to set OFFBOARD");
	}
}

void setArm(bool arm) {
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = arm;

	if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
		if (arm)
			ROS_INFO("TAKEOFF - Vehicle ARMED");
		else
			ROS_INFO("TAKEOFF - Vehicle DISARMED");
	} else {
		ROS_ERROR("TAKEOFF - Failed arming or disarming");
	}
}

//**************** CALLBACKS *******************************************************************************

void localPosition_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	local_position = *msg;
	local_position_received = true;
	ROS_DEBUG_ONCE("TAKEOFF - Got local position: [%.2f, %.2f, %.2f]", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	//ROS_INFO("State Updates");
	current_state = *msg;
}

bool execute_cb(uav_mavros_takeoff::TakeOff::Request &request, uav_mavros_takeoff::TakeOff::Response &response) {
    ROS_DEBUG("TAKEOFF - Executing Takeoff service..");

	double altitude_step = request.altitude / takeoff_steps;

	//Get current position
	ros::spinOnce();

	// set target position
	geometry_msgs::PoseStamped goal_position;
	goal_position.pose.position.x = local_position.pose.position.x;
	goal_position.pose.position.y = local_position.pose.position.y;
	goal_position.pose.position.z = local_position.pose.position.z;

	// the setpoint publishing rate MUST be faster than 2Hz
	ros::Rate rate(freq);

	//start sending setpoints (current location)
	goal_pos_pub.publish(goal_position);
	//send a few setpoints before starting
	sleep(2);

	////////////////////////////////////////////
	/////////////////SET MODE////////////////////
	////////////////////////////////////////////
	setMode();

	////////////////////////////////////////////
	/////////////////ARMING////////////////////
	////////////////////////////////////////////
	setArm(true);

	////////////////////////////////////////////
	/////////////////TAKEOFF////////////////////
	////////////////////////////////////////////
	int current_step = 0;
	goal_position.pose.position.z = goal_position.pose.position.z + altitude_step;
	goal_pos_pub.publish(goal_position);
	while (ros::ok()) {
		if (isAtAltitude(goal_position.pose.position.z, pos_tolerance)) {
			current_step++;
            ROS_DEBUG("TAKEOFF - Step %d completed", current_step);
			if (current_step < takeoff_steps) {
				goal_position.pose.position.z = goal_position.pose.position.z + altitude_step;
				goal_pos_pub.publish(goal_position);
			} else {
				break;
			}
		}
		ros::spinOnce();
		rate.sleep();
	}

	ROS_DEBUG("TAKEOFF - Waiting %d seconds to stabilize", stabilize_time);
	//Stabilize position
	sleep(stabilize_time);
	ros::spinOnce();

    ROS_DEBUG("TAKEOFF - TakeOff completed");
    ROS_DEBUG("TAKEOFF - Position reached: x: %f, y: %f, z: %f", local_position.pose.position.x, local_position.pose.position.y,
			local_position.pose.position.z);

	response.success = true;

	return true;
}

//*******************************************************************************************************************

int main(int argc, char **argv) {
	ros::init(argc, argv, "takeoff_service");
	ros::NodeHandle nh;

	//Get position tolerance
	nh.getParam(ros::this_node::getName() + "/pos_tolerance", pos_tolerance);
	//Get loop rate
	nh.getParam(ros::this_node::getName() + "/frequency", freq);
	//Get stabilization sleep
	nh.getParam(ros::this_node::getName() + "/stabilize_time", stabilize_time);
	//Get takeoff num steps
	nh.getParam(ros::this_node::getName() + "/takeoff_steps", takeoff_steps);

	string arming_topic = "mavros/cmd/arming";
	arming_client = nh.serviceClient < mavros_msgs::CommandBool > (arming_topic);

	string set_mode_topic = "mavros/set_mode";
	set_mode_client = nh.serviceClient < mavros_msgs::SetMode > (set_mode_topic);

	string state_topic = "mavros/state";
	state_sub = nh.subscribe < mavros_msgs::State > (state_topic, 10, state_cb);

	string local_pos_topic = "pos_provider/pose";
	local_pos_sub = nh.subscribe < geometry_msgs::PoseStamped > (local_pos_topic, 1, localPosition_cb);

	string goal_pos_topic = "pos_controller/goal_position";
	goal_pos_pub = nh.advertise < geometry_msgs::PoseStamped > (goal_pos_topic, 1);

	ros::Rate rate(freq);
	// wait for FCU connection
	while (ros::ok() && !current_state.connected) {
		ROS_DEBUG_ONCE("Waiting FCU connection..");
		ros::spinOnce();
		rate.sleep();
	}
	ROS_DEBUG("TAKEOFF - FCU connected");

	// wait for uav local position
	while (ros::ok() && !local_position_received) {
		ROS_DEBUG_ONCE("TAKEOFF - Waiting for local position...");
		ros::spinOnce();
		rate.sleep();
	}
	ROS_DEBUG("TAKEOFF - Local position received");

	ros::ServiceServer service = nh.advertiseService("cmd/takeoff", execute_cb);
	ROS_INFO("TAKEOFF - TakeOff service available");
	ros::spin();

	return 0;
}
