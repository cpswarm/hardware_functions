#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <mavros_moveto_target/MoveToTargetAction.h>
#include <cpswarm_msgs/TargetPositionEvent.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <math.h>
#include <std_msgs/Empty.h>

using namespace std;

typedef actionlib::SimpleActionServer<mavros_moveto_target::MoveToTargetAction> Server;

// Variables
ros::Publisher goal_pos_pub;
ros::Publisher stop_pos_pub;
ros::Publisher target_done_pub;

ros::Subscriber state_sub;
ros::Subscriber pos_sub;
ros::Subscriber trg_update_sub;
ros::Subscriber trg_lost_sub;

ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped position;
geometry_msgs::PoseStamped goal_position;
bool position_received = false;
bool target_lost;
bool target_updated;
bool listen_to_update;
double pos_tolerance;
double freq;
int target_id;

//**********************************************************************************************************
bool isAtPosition(geometry_msgs::Pose pos, double tolerance) {
    double distance = hypot(pos.position.x - position.pose.position.x, pos.position.y - position.pose.position.y);

	ROS_INFO_THROTTLE(1, "MOVETO_TRG - Distance: %f", distance);
	return distance < tolerance;
}

void setMode() {
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.base_mode = 0;
	offb_set_mode.request.custom_mode = "GUIDED";

	if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
		ROS_INFO("MOVETO_TRG - GUIDED enabled");
	} else {
		ROS_ERROR("MOVETO_TRG - Failed to set GUIDED");
	}
}

void setArm(bool arm) {
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = arm;

	if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
		if (arm)
			ROS_INFO("MOVETO_TRG - Vehicle ARMED");
		else
			ROS_INFO("MOVETO_TRG - Vehicle DISARMED");
	} else {
		ROS_ERROR("MOVETO_TRG - Failed arming or disarming");
	}
}

//**************** CALLBACKS *******************************************************************************

void position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	position = *msg;
	position_received = true;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	//ROS_INFO("State Updates");
	current_state = *msg;
}

void targetUpdate_cb(const cpswarm_msgs::TargetPositionEvent::ConstPtr& msg) {
	if (listen_to_update && (target_id == msg->id)) {
		ROS_INFO("MOVETO_TRG - Received TARGET UPDATE");
		goal_position = msg->pose;
		target_updated = true;
	}
}

void targetLost_cb(const cpswarm_msgs::TargetPositionEvent::ConstPtr& msg) {
	if (listen_to_update) {
		ROS_INFO("MOVETO_TRG - Received TARGET LOST");
		target_lost = true;
	}
}

void execute_cb(const mavros_moveto_target::MoveToTargetGoal::ConstPtr& goal, Server* as) {
	ROS_INFO("MOVETO_TRG - Executing MoveToTarget action..");

	target_id = goal->target_id;
	target_lost = false;
	target_updated = false;

	ros::Rate rate(freq);

	goal_position = goal->pose;

	//start sending setpoints (goal location)
	goal_pos_pub.publish(goal_position);

	////////////////////////////////////////////
	/////////////////SET MODE////////////////////
	////////////////////////////////////////////
	setMode();

	////////////////////////////////////////////
	/////////////////ARMING////////////////////
	////////////////////////////////////////////
	setArm(true);

	////////////////////////////////////////////
	/////////////////MOVETO_TRG/////////////////////
	////////////////////////////////////////////
	while (ros::ok() && !as->isPreemptRequested() && !target_lost && !isAtPosition(goal_position.pose, pos_tolerance)) {
		if(target_updated) {
			goal_pos_pub.publish(goal_position);
			target_updated = false;
		}
		ros::spinOnce();
		rate.sleep();
	}

	listen_to_update = false;
	target_updated = false;

	////////////////////////////////////////////
	/////////////////DISARMING////////////////////
	////////////////////////////////////////////
	setArm(false);

	//Stop pos_controller publishing
	std_msgs::Empty stop_msg;
	stop_pos_pub.publish(stop_msg);

	if (!target_lost) {
		ROS_INFO("MOVETO_TRG - MoveToTarget completed");
		ROS_INFO("MOVETO_TRG - Position reached: %.2f, %.2f", position.pose.position.x, position.pose.position.x);
		//Inform others TARGET DONE
		cpswarm_msgs::TargetPositionEvent target_msg;
		target_msg.header.stamp = ros::Time::now();
		target_msg.swarmio.name = "target_done";
		target_msg.swarmio.node = "";
		target_msg.id = target_id;
		target_msg.pose = goal->pose;
		target_done_pub.publish(target_msg);
		ROS_INFO("MOVETO_TRG - Target done event SENT");
	} else {
		ROS_INFO("MOVETO_TRG - Target Lost, STOP moving");
	}

	if (as->isPreemptRequested()) {
		as->setPreempted();
	} else if (target_lost) {
		as->setAborted();
	} else {
		as->setSucceeded();
	}
}

//*******************************************************************************************************************

int main(int argc, char **argv) {
	ros::init(argc, argv, "moveto_target");
	ros::NodeHandle nh;

	target_id = -1;

	//Get position tolerance
	nh.getParam(ros::this_node::getName() + "/pos_tolerance", pos_tolerance);
	//Get loop rate
	nh.getParam(ros::this_node::getName() + "/frequency", freq);
	//Get target update topic
	string target_update_topic;
	nh.getParam(ros::this_node::getName() + "/target_update", target_update_topic);
	//Get target lost topic
	string target_lost_topic;
	nh.getParam(ros::this_node::getName() + "/target_lost", target_lost_topic);
	//Get target_done_topic
	string target_done_topic;
	nh.getParam(ros::this_node::getName() + "/target_done", target_done_topic);

	string arming_topic = "mavros/cmd/arming";
	arming_client = nh.serviceClient < mavros_msgs::CommandBool > (arming_topic);

	string set_mode_topic = "mavros/set_mode";
	set_mode_client = nh.serviceClient < mavros_msgs::SetMode > (set_mode_topic);

	string state_topic = "mavros/state";
	state_sub = nh.subscribe < mavros_msgs::State > (state_topic, 10, state_cb);

	string pos_topic = "pos_provider/pose";
	pos_sub = nh.subscribe < geometry_msgs::PoseStamped > (pos_topic, 1, position_cb);

	string goal_pos_topic = "pos_controller/goal_position";
	goal_pos_pub = nh.advertise < geometry_msgs::PoseStamped > (goal_pos_topic, 1);

	string stop_topic = "pos_controller/stop";
	stop_pos_pub = nh.advertise < std_msgs::Empty > (stop_topic, 1, true);

	//init target done publisher
	target_done_pub = nh.advertise < cpswarm_msgs::TargetPositionEvent > (target_done_topic, 1, true);
	//subscribe to target update
	trg_update_sub = nh.subscribe < cpswarm_msgs::TargetPositionEvent > (target_update_topic, 10, targetUpdate_cb);
	//subscribe to target lost
	trg_lost_sub = nh.subscribe < cpswarm_msgs::TargetPositionEvent > (target_lost_topic, 10, targetLost_cb);

	ros::Rate rate(freq);
	// wait for FCU connection
	while (ros::ok() && !current_state.connected) {
		ros::spinOnce();
		rate.sleep();
	}
	ROS_DEBUG("MOVETO_TRG - FCU connected");

	// wait for position
	while (ros::ok() && !position_received) {
		ROS_DEBUG_ONCE("Waiting for position..");
		ros::spinOnce();
		rate.sleep();
	}
	ROS_DEBUG("MOVETO_TRG - Position received");

	Server server(nh, "cmd/moveto_target", boost::bind(&execute_cb, _1, &server), false);
	server.start();
	ROS_INFO("MOVETO_TRG - MoveToTarget action available");
	ros::spin();

	return 0;
}
