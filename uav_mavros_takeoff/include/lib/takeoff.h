#ifndef TAKEOFF_H
#define TAKEOFF_H

#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>

using namespace std;
using namespace ros;

/**
 * @brief A class to to perform take off with a UAV.
 */
class takeoff
{
public:
    /**
     * @brief Constructor.
     */
    takeoff ();

    /**
     * @brief Perform the take off.
     * @param altitude: The altitude to take off to.
     * @return Whether the take off succeeded.
     */
    bool execute (double altitude);

private:
    /**
     * @brief Arm the vehicle.
     * @return Whether arming succeeded.
     */
    bool arm ();

    /**
     * @brief Set the FCU to a given mode.
     * @param The mode to change to.
     * @return Whether the mode change succeeded.
     */
    bool set_mode (string mode);

    /**
     * @brief Callback function to receive the local position of the UAV.
     * @param msg Returns the local position in cartesian coordinates.
     */
    void position_cb (const geometry_msgs::PoseStamped::ConstPtr& msg);

    /**
     * @brief Callback function to receive the state of the UAV.
     * @param msg Returns the current FCU state of the UAV.
     */
    void state_cb(const mavros_msgs::State::ConstPtr& msg);

    /**
     * @brief Service client to arm the UAV.
     */
    ros::ServiceClient arming_client;

    /**
     * @brief Service client to set the FCU mode.
     */
    ros::ServiceClient set_mode_client;

    /**
     * @brief Service client to take off with ardupilot FCU.
     */
    ros::ServiceClient takeoff_client;

    /**
     * @brief Subscriber to get the current FCU state of the UAV.
     */
    ros::Subscriber state_sub;

    /**
     * @brief Subscriber to get the local position of the UAV.
     */
    ros::Subscriber pos_sub;

    /**
     * @brief Publisher to publish the desired goal position of the UAV.
     */
    ros::Publisher goal_pub;

    /**
     * @brief The frequency at which to run the control loops.
     */
    double loop_rate;

    /**
     * @brief How close the UAV has to be to the desired position after take off.
     */
    double pos_tolerance;

    /**
     * @brief The time to stabilize position after takeoff.
     */
    double stabilize_time;

    /**
     * @brief The FCU firmware, either px4 or apm.
     */
    string fcu;

    /**
     * @brief The current state of the FCU.
     */
    mavros_msgs::State state;

    /**
     * @brief The current position of the UAV.
     */
    geometry_msgs::PoseStamped position;

    /**
     * @brief Whether a position has been received from the FCU.
     */
    bool position_received;
};

#endif // TAKEOFF_H
