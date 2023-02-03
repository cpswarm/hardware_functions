#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <math.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>

using namespace std;

// Variables
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
bool isAtAltitude(double altitude, double tolerance) {
	return abs(altitude - local_position.pose.position.z) < tolerance;
}

void setMode(string fcu) {
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.base_mode = 0;
	if (fcu == "apm")
		offb_set_mode.request.custom_mode = "LAND";
	else
		offb_set_mode.request.custom_mode = "AUTO.LAND";

	if (set_mode_client.call(offb_set_mode)) {
		ROS_INFO_ONCE("LAND - SetMode success: %d", offb_set_mode.response.mode_sent);
	} else {
		ROS_ERROR("LAND - Failed SetMode");
	}
}

void setArm(bool arm) {
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = arm;

	if (arming_client.call(arm_cmd)) {
		ROS_INFO("LAND - Set ARM success: %d", arm_cmd.response.success);
	} else {
		ROS_ERROR("LAND - Failed arming or disarming");
	}
}

//**************** CALLBACKS *******************************************************************************

void localPosition_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	//ROS_INFO("Local position received");
	local_position = *msg;
	local_position_received = true;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	//ROS_INFO("State Updates");
	current_state = *msg;
}

bool execute_cb(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
	ROS_INFO("LAND - Executing Land service..");

	ros::Rate rate(freq);

	// check if firmware is px4 or ardupilot
	ros::NodeHandle nh;
	string fcu;
	nh.getParam(ros::this_node::getName() + "/fcu", fcu);
	if (fcu != "px4" && fcu != "apm") {
		ROS_FATAL("LAND - Unknown FCU firmware, cannot land");
		return false;
	}

	//Stop pos_controller publishing
	std_msgs::Empty stop_msg;
	stop_pos_pub.publish(stop_msg);

	////////////////////////////////////////////
	/////////////////LANDING////////////////////
	////////////////////////////////////////////
	while (ros::ok() && !isAtAltitude(0, pos_tolerance)) {
		////////////////////////////////////////////
		/////////////////SET LANDING MODE///////////
		////////////////////////////////////////////
		setMode(fcu);

		ros::spinOnce();
		rate.sleep();
	}

	////////////////////////////////////////////
	/////////////////DISARMING//////////////////
	////////////////////////////////////////////
	setArm(false);

	ROS_INFO("LAND - Landing completed");

	return true;
}

//*******************************************************************************************************************

int main(int argc, char **argv) {
	ros::init(argc, argv, "land_service");
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
	local_pos_sub = nh.subscribe < geometry_msgs::PoseStamped
			> (local_pos_topic, 1, localPosition_cb);

	string stop_topic = "pos_controller/stop";
	stop_pos_pub = nh.advertise < std_msgs::Empty > (stop_topic, 1, true);

	ros::Rate rate(freq);
	// wait for FCU connection
	while (ros::ok() && !current_state.connected) {
		ROS_DEBUG_ONCE("Waiting connection..");
		ros::spinOnce();
		rate.sleep();
	}
	ROS_DEBUG("LAND - FCU connected");

	// wait for uav local position
	while (ros::ok() && !local_position_received) {
		ROS_DEBUG_ONCE("LAND - Waiting for GPS...");
		ros::spinOnce();
		rate.sleep();
	}
	ROS_DEBUG("LAND - Local position received");

	ros::ServiceServer service = nh.advertiseService("cmd/land", execute_cb);
	ROS_INFO("LAND - Landing service available");
	ros::spin();

	return 0;
}
