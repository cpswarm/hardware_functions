#include "takeoff_service.h"

/**
 * @brief Callback to execute the take off service.
 */
bool execute_cb(uav_mavros_takeoff::TakeOff::Request &request, uav_mavros_takeoff::TakeOff::Response &response)
{
    ROS_DEBUG("TAKEOFF - Executing take off service...");

	// perform take off
	response.success = to->execute(request.altitude);

	return true;
}

/**
 * @brief A ROS node that provides the take off service.
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return Success.
 */
int main(int argc, char **argv) {
	init(argc, argv, "takeoff_service");
	NodeHandle nh;

	// initialize takeoff library
	to = new takeoff();

	// provide take off service
	ServiceServer service = nh.advertiseService("cmd/takeoff", execute_cb);
	ROS_INFO("TAKEOFF - Service available");
	spin();

	return 0;
}
