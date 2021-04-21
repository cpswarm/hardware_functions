#ifndef RETURN_ACTION_H
#define RETURN_ACTION_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <cpswarm_msgs/MoveToAction.h>
#include "lib/return.h"

using namespace std;
using namespace ros;

/**
 * @brief Action server for returning.
 */
typedef actionlib::SimpleActionServer<cpswarm_msgs::MoveToAction> Server;

/**
 * @brief The object encapsulating the return functionality.
 */
rtl* ret;

#endif // RETURN_ACTION_H
