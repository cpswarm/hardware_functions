#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_moveto/MoveToAction.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <math.h>
#include <std_msgs/Empty.h>

using namespace std;

typedef actionlib::SimpleActionServer<mavros_moveto::MoveToAction> Server;

// Variables
ros::Publisher goal_pos_pub;
ros::Publisher stop_pos_pub;

ros::Subscriber state_sub;
ros::Subscriber local_pos_sub;

ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped local_position;
bool local_position_received = false;
double pos_tolerance;
double freq;

//**********************************************************************************************************
bool isAtPosition(double x, double y, double tolerance) {
	double dx = local_position.pose.position.x - x;
	double dy = local_position.pose.position.y - y;

	double distance = sqrt(dx * dx + dy * dy);
	ROS_INFO_THROTTLE(1, "MOVETO - Distance: %f", distance);
	return distance < tolerance;
}

void setMode() {
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.base_mode = 0;
	offb_set_mode.request.custom_mode = "GUIDED";

	if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
		ROS_INFO("MOVETO - GUIDED enabled");
	} else {
		ROS_ERROR("MOVETO - Failed to set GUIDED");
	}
}

void setArm(bool arm) {
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = arm;

	if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
		if (arm)
			ROS_INFO("MOVETO - Vehicle ARMED");
		else
			ROS_INFO("MOVETO - Vehicle DISARMED");
	} else {
		ROS_ERROR("MOVETO - Failed arming or disarming");
	}
}

//**************** CALLBACKS *******************************************************************************

void localPosition_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	local_position = *msg;
	local_position_received = true;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	//ROS_DEBUG("State Updates");
	current_state = *msg;
}

void execute_cb(const mavros_moveto::MoveToGoal::ConstPtr& goal, Server* as) {
	ROS_INFO("MOVETO - Executing MoveTo action..");

	//the setpoint publishing rate MUST be faster than 2Hz
	ros::Rate rate(freq);

	geometry_msgs::PoseStamped goal_position;
	goal_position.pose.position.x = goal->x;
	goal_position.pose.position.y = goal->y;
	goal_position.pose.position.z = local_position.pose.position.z;

	//start sending setpoints (goal location)
	goal_pos_pub.publish(goal_position);

	////////////////////////////////////////////
	/////////////////SET MODE////////////////////
	////////////////////////////////////////////
	setMode();

	///////////////////////////////////////////
	/////////////////ARMING////////////////////
	///////////////////////////////////////////
	setArm(true);

	///////////////////////////////////////////
	/////////////////MOVETO////////////////////
	///////////////////////////////////////////
	while (ros::ok() && !as->isPreemptRequested() && !isAtPosition(goal_position.pose.position.x, goal_position.pose.position.y, pos_tolerance)) {
		ros::spinOnce();
		rate.sleep();
	}

	////////////////////////////////////////////
	/////////////////DISARMING////////////////////
	////////////////////////////////////////////
	setArm(false);

	//Stop pos_controller publishing
	std_msgs::Empty stop_msg;
	stop_pos_pub.publish(stop_msg);

	ROS_INFO("MOVETO - MoveTo completed");
	ROS_INFO("MOVETO - Position reached: x: %f, y: %f", local_position.pose.position.x, local_position.pose.position.y);

	if (as->isPreemptRequested()) {
		as->setPreempted();
	} else {
		as->setSucceeded();
	}
}

//*******************************************************************************************************************

int main(int argc, char **argv) {
	ros::init(argc, argv, "local_moveto");
	ros::NodeHandle nh;

	//Get position tolerance
	nh.getParam(ros::this_node::getName() + "/pos_tolerance", pos_tolerance);
	//Get loop rate
	nh.getParam(ros::this_node::getName() + "/frequency", freq);

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

	string stop_topic = "pos_controller/stop";
	stop_pos_pub = nh.advertise < std_msgs::Empty > (stop_topic, 1);

	ros::Rate rate(freq);
	// wait for FCU connection
	while (ros::ok() && !current_state.connected) {
		ros::spinOnce();
		rate.sleep();
	}
	ROS_DEBUG("MOVETO - FCU connected");

	// wait for local position
	while (ros::ok() && !local_position_received) {
		ROS_DEBUG_ONCE("Waiting for local position..");
		ros::spinOnce();
		rate.sleep();
	}
	ROS_DEBUG("MOVETO - Local position received");

	Server server(nh, "cmd/moveto", boost::bind(&execute_cb, _1, &server), false);
	server.start();
	ROS_INFO("MOVETO - MoveTo action available");
	ros::spin();

	return 0;
}
