#ifndef TAKEOFF_ACTION_H
#define TAKEOFF_ACTION_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <cpswarm_msgs/TakeoffAction.h>
#include "lib/takeoff.h"

using namespace std;
using namespace ros;

/**
 * @brief Action server for takeoff.
 */
typedef actionlib::SimpleActionServer<cpswarm_msgs::TakeoffAction> Server;

/**
 * @brief The object encapsulating the take off functionality.
 */
takeoff* to;

#endif // TAKEOFF_ACTION_H
