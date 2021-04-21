#ifndef RETURN_H
#define RETURN_H

#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
using namespace ros;

/**
 * @brief A class to let a CPS return to its starting position.
 */
class rtl
{
public:
    /**
     * @brief Constructor.
     */
    rtl ();

    /**
     * @brief Get the current position of the CPS.
     * @return The stamped pose of the CPS.
     */
    geometry_msgs::PoseStamped get_pos ();

    /**
     * @brief Publish home position.
     * @param altitude: The altitude at which to return.
     */
    void publish (double altitude);

    /**
     * @brief Check if the CPS reached its starting position.
     * @return True, when the horizontal position is closer than pos_tolerance to the starting position.
     */
    bool reached ();

private:
    /**
     * @brief Callback function to receive the local position of the CPS.
     * @param msg Returns the local position in cartesian coordinates.
     */
    void position_cb (const geometry_msgs::PoseStamped::ConstPtr& msg);

    /**
     * @brief Subscriber to get the local position of the CPS.
     */
    ros::Subscriber pos_sub;

    /**
     * @brief Publisher to publish the home position of the CPS.
     */
    ros::Publisher goal_pub;

    /**
     * @brief The frequency at which to run the control loops.
     */
    double loop_rate;

    /**
     * @brief How close the CPS has to return to the home position.
     */
    double pos_tolerance;

    /**
     * @brief The current position of the CPS.
     */
    geometry_msgs::PoseStamped position;

    /**
     * @brief Whether a position has been received from the FCU.
     */
    bool position_received;

    /**
     * @brief Position where the CPS will return to.
     */
    geometry_msgs::PoseStamped home;
};

#endif // RETURN_H
