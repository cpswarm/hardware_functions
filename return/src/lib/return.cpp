#include "lib/return.h"

rtl::rtl ()
{
    NodeHandle nh;

    // read parameters
    nh.getParam(this_node::getName() + "/loop_rate", loop_rate);
    int queue_size;
    nh.getParam(this_node::getName() + "/queue_size", queue_size);
    nh.getParam(this_node::getName() + "/pos_tolerance", pos_tolerance);

    // initialize global variables
    position_received = false;

    // ros communication
    pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("pos_provider/pose", queue_size, &rtl::position_cb, this);
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("pos_controller/goal_position", queue_size);

    Rate rate(loop_rate);

    // wait for position information
    while (ok() && !position_received) {
        ROS_DEBUG_ONCE("RETURN - Waiting for home position...");
        spinOnce();
        rate.sleep();
    }
    ROS_DEBUG("RETURN - Home position received");
    home = position;
}

geometry_msgs::PoseStamped rtl::get_pos ()
{
    return position;
}

void rtl::publish (double altitude)
{
    // the setpoint publishing rate
    Rate rate(loop_rate);

    // publish home position
    geometry_msgs::PoseStamped goal;
    goal = home;
    goal.header.stamp = Time::now();
    goal.pose.position.z = altitude;
    goal_pub.publish(home);

    ROS_DEBUG("RETURN - Completed, reached position (%.2f,%.2f,%.2f)", position.pose.position.x, position.pose.position.y, position.pose.position.z);
}

bool rtl::reached ()
{
	return hypot(home.pose.position.x - position.pose.position.x, home.pose.position.y - position.pose.position.y) <= pos_tolerance;
}

void rtl::position_cb (const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    position = *msg;
    position_received = true;
    ROS_DEBUG_ONCE("RETURN - Got home position: (%.2f,%.2f,%.2f)", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}
