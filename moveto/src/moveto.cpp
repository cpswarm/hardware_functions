#include <ros/ros.h>
#include <tf2/utils.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "cpswarm_msgs/OutOfBounds.h"

using namespace std;
using namespace ros;

/**
 * @brief An action server type that allows to move a CPS.
 */
typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> action_server_t;

/**
 * @brief Service client for determining whether the goal is out of the area bounds.
 */
ServiceClient out_of_bounds_client;

/**
 * @brief Publisher for sending the goal position of the CPS to the position controller in the abstraction library.
 */
Publisher pose_pub;

/**
 * @brief Current position of the CPS.
 */
geometry_msgs::Pose pose;

/**
 * @brief Whether a valid position has been received from the position provider.
 */
bool pose_valid;

/**
 * @brief The loop rate object for running the behavior control loops at a specific frequency.
 */
Rate* rate;

/**
 * @brief The distance that the CPS can be away from a goal while still being considered to have reached that goal.
 */
double goal_tolerance;

/**
 * @brief The angle that the CPS can be away from a goal while still being considered to have reached that goal.
 */
double yaw_tolerance;

/**
 * @brief Compute the straight-line distance between two positions.
 * @param p1 First pose.
 * @param p2 Second pose.
 * @return The distance in meters.
 */
double dist (geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
    return hypot(p1.position.x - p2.position.x, p1.position.y - p2.position.y);
}

/**
 * @brief Get the yaw orientation from a pose.
 * @param pose The pose that contains the orientation.
 * @return The yaw angle of the given pose counterclockwise starting from x-axis/east.
 */
double get_yaw (geometry_msgs::Pose pose)
{
    tf2::Quaternion orientation;
    tf2::fromMsg(pose.orientation, orientation);
    return tf2::getYaw(orientation);
}

/**
 * @brief Check whether a given pose is out of the mission area boundaries.
 * @param pose The pose to check.
 * @return True if the given pose is outside the mission area or it could not be checked, false otherwise.
 */
bool out_of_bounds (geometry_msgs::Pose pose)
{
    cpswarm_msgs::OutOfBounds oob;
    oob.request.pose = pose;
    if (out_of_bounds_client.call(oob)){
        return oob.response.out;
    }
    else{
        ROS_ERROR("Failed to check if goal is out of bounds");
        return true;
    }
}

/**
 * @brief Check whether the CPS has reached a given pose.
 * @param goal The pose to check.
 * @return True if the CPS is close to the given pose, false otherwise.
 */
bool reached (geometry_msgs::Pose goal)
{
    return dist(pose, goal) < goal_tolerance && remainder(get_yaw(pose) - get_yaw(goal), 2*M_PI) + M_PI < yaw_tolerance;
}

/**
 * @brief Callback function for position updates.
 * @param msg Position received from the CPS.
 */
void pose_callback (const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // valid pose received
    if (msg->header.stamp.isValid())
        pose_valid = true;

    // store new position and orientation in class variables
    pose = msg->pose;

    ROS_DEBUG_THROTTLE(1, "Yaw %.2f", get_yaw(pose));
    ROS_DEBUG_THROTTLE(1, "Pose [%.2f, %.2f, %.2f]", pose.position.x, pose.position.y, pose.position.z);
}

/**
 * @brief Callback function to move the CPS.
 * @param goal The action goal.
 * @param as Reference to the action server object.
 */
void moveto_callback(const move_base_msgs::MoveBaseGoal::ConstPtr& goal, action_server_t* as)
{
    // action server result
    move_base_msgs::MoveBaseFeedback feedback;

    // goal is out of bounds
    if (out_of_bounds(goal->target_pose.pose)) {
        ROS_ERROR("Cannot move to (%.2f,%.2f) because it is out of bounds!", goal->target_pose.pose.position.x, goal->target_pose.pose.position.y);
        as->setAborted();
        return;
    }

    // send goal pose to cps controller
    pose_pub.publish(goal->target_pose);

    // wait until cps reached goal
    while (ok() && reached(goal->target_pose.pose) == false) {
        // provide feedback to client
        geometry_msgs::PoseStamped ps;
        ps.header.stamp = Time::now();
        ps.pose = pose;
        feedback.base_position = ps;
        as->publishFeedback(feedback);

        // wait
        rate->sleep();

        // check if reached goal
        spinOnce();
    }

    // action server has been preempted
    if (as->isPreemptRequested()) {
        as->setPreempted();
    }

    // return position
    else {
        as->setSucceeded();
    }
}

/**
 * @brief A ROS node that moves a CPS to a given position using an action server.
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return Success.
 */
int main (int argc, char **argv)
{
    // init ros node
    init(argc, argv, "moveto");
    NodeHandle nh;

    // read parameters
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    rate = new Rate(loop_rate);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    nh.param(this_node::getName() + "/goal_tolerance", goal_tolerance, 0.1);
    nh.param(this_node::getName() + "/yaw_tolerance", yaw_tolerance, 0.02);

    // no pose received yet
    pose_valid = false;

    // init ros communication
    out_of_bounds_client = nh.serviceClient<cpswarm_msgs::OutOfBounds>("area/out_of_bounds");
    Subscriber pose_sub = nh.subscribe("pos_provider/pose", queue_size, pose_callback);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pos_controller/goal_position", queue_size, true);

    // init position and yaw
    while (ok() && pose_valid == false) {
        ROS_DEBUG_ONCE("Waiting for valid pose...");
        rate->sleep();
        spinOnce();
    }

    // start the action server and wait
    action_server_t server(nh, "cmd/moveto", boost::bind(&moveto_callback, _1, &server), false);
    server.start();
    ROS_INFO("MOVETO - MoveTo action available");
    spin();

    delete rate;
}
