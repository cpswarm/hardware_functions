#include "return_action.h"

/**
 * @brief Callback to execute the return action server.
 */
bool execute_cb(const cpswarm_msgs::MoveToGoal::ConstPtr& goal, Server* as)
{
    ROS_DEBUG("RETURN - Executing return action...");

    NodeHandle nh;

    cpswarm_msgs::MoveToFeedback feedback;
    cpswarm_msgs::MoveToResult result;

    double loop_rate;
    nh.getParam(this_node::getName() + "/loop_rate", loop_rate);
    Rate rate(loop_rate);

    // start returning
    ret->publish(goal->goal.pose.position.z);

    // wait until returned
    while (ret->reached() == false && as->isPreemptRequested() == false && ok()) {
        spinOnce();
        feedback.position = ret->get_pos();
        as->publishFeedback(feedback);
        rate.sleep();
    }

    // set action state
    if (as->isPreemptRequested()) {
        as->setPreempted();
    }
    else {
        result.reached = ret->get_pos();
        as->setSucceeded(result);
    }

    return true;
}

/**
 * @brief A ROS node that provides the return action server.
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return Success.
 */
int main(int argc, char **argv) {
    init(argc, argv, "return_action");
    NodeHandle nh;

    // initialize return library
    ret = new rtl();

    // provide action server
    Server server(nh, "cmd/return", boost::bind(&execute_cb, _1, &server), false);
    server.start();
    ROS_INFO("RETURN - Action server available");
    spin();

    return 0;
}
