#ifndef ACTION_SERVER_H
#define ACTION_SERVER_H

#include <ros/ros.h>
#include <tf2/utils.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "cpswarm_msgs/OutOfBounds.h"

using namespace std;
using namespace ros;

/**
 * @brief The action server implementation for the moveto action.
 */
class action_server
{
public:
    /**
     * @brief Constructor that starts the action server.
     */
    action_server ();

    /**
     * @brief Destructor.
     */
    ~action_server ();

private:
    /**
     * @brief Compute the straight-line distance between two positions.
     * @param p1 First pose.
     * @param p2 Second pose.
     * @return The distance in meters.
     */
    double dist (geometry_msgs::Pose p1, geometry_msgs::Pose p2);

    /**
     * @brief Get the yaw orientation from a pose.
     * @param pose The pose that contains the orientation.
     * @return The yaw angle of the given pose counterclockwise starting from x-axis/east.
     */
    double get_yaw (geometry_msgs::Pose pose);

    /**
     * @brief Check whether a given pose is out of the mission area boundaries.
     * @param pose The pose to check.
     * @return True if the given pose is outside the mission area or it could not be checked, false otherwise.
     */
    bool out_of_bounds (geometry_msgs::Pose pose);

    /**
     * @brief Check whether the CPS has reached a given pose.
     * @param goal The pose to check.
     * @return True if the CPS is close to the given pose, false otherwise.
     */
    bool reached (geometry_msgs::Pose goal);

    /**
     * @brief Callback function for position updates.
     * @param msg Position received from the CPS.
     */
    void pose_callback (const geometry_msgs::PoseStamped::ConstPtr& msg);

    /**
     * @brief Callback function to move the CPS.
     * @param goal The action goal.
     */
    void moveto_callback(const move_base_msgs::MoveBaseGoal::ConstPtr& goal);

    /**
     * @brief ROS node handle. Must be declared before the action server!
     */
    NodeHandle nh;

    /**
     * @brief Action server.
     */
    actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> server;

    /**
    * @brief Service client for determining whether the goal is out of the area bounds.
    */
    ServiceClient out_of_bounds_client;

    /**
    * @brief Publisher for sending the goal position of the CPS to the position controller in the abstraction library.
    */
    Publisher pose_pub;

    /**
     * @brief Subscriber for receiving the current CPS position.
     */
    Subscriber pose_sub;

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
};

#endif // ACTION_SERVER_H
