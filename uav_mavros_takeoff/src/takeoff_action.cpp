#include "takeoff_action.h"

/**
 * @brief Callback to execute the take off action server.
 */
bool execute_cb(const cpswarm_msgs::TakeoffGoal::ConstPtr& goal, Server* as)
{
    ROS_DEBUG("TAKEOFF - Executing take off action...");

	// initiate take off
	bool success = to->execute(goal->altitude);

	// take off failed
	if (success == false) {
		as->setAborted();
    	ROS_DEBUG("TAKEOFF - Failed");
		return true;
	}

	// wait until altitude is reached
	bool reached = false;
	while (reached == false && as->isPreemptRequested() == false) {
		reached = to->wait();
	}

	if (as->isPreemptRequested()) {
		as->setPreempted();
    	ROS_DEBUG("TAKEOFF - Preempted");
	}
	else if (reached) {
		as->setSucceeded();
    	ROS_DEBUG("TAKEOFF - Completed");
    }
	else {
		as->setAborted();
    	ROS_DEBUG("TAKEOFF - Failed");
	}

    return true;
}

/**
 * @brief A ROS node that provides the take off action server.
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return Success.
 */
int main(int argc, char **argv) {
	init(argc, argv, "takeoff_action");
	NodeHandle nh;

	// initialize takeoff library
	to = new takeoff();

	// provide action server
	Server server(nh, "cmd/takeoff", boost::bind(&execute_cb, _1, &server), false);
	server.start();
	ROS_INFO("TAKEOFF - Action server available");
	spin();

	return 0;
}
