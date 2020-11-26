#ifndef TAKEOFF_SERVICE_H
#define TAKEOFF_SERVICE_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <cpswarm_msgs/TakeoffAction.h>
#include <uav_mavros_takeoff/TakeOff.h>
#include "lib/takeoff.h"

using namespace std;
using namespace ros;

/**
 * @brief The object encapsulating the take off functionality.
 */
takeoff* to;

#endif // TAKEOFF_SERVICE_H
