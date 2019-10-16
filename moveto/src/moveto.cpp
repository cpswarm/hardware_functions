#include "lib/action_server.h"

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

    // start the action server and wait
    action_server moveto;
    ROS_INFO("MOVETO - MoveTo action available");
    spin();
}
