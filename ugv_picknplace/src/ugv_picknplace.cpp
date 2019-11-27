#include "lib/action_server.h"

/**
 * @brief A ROS node that moves a CPS to a given position, pick a cart, moves to another given position and place it, using an action server.
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return Success.
 */
int main (int argc, char **argv)
{
    // init ros node
    init(argc, argv, "picknplace");

    // start the action server and wait
    action_server picknplace;
    ROS_INFO("PICKNPLACE - PicknPlace action available");
    spin();
}

