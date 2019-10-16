#include "lib/action_server.h"

action_server::action_server () : server(nh, "cmd/moveto", boost::bind(&action_server::moveto_callback, this, _1), false)
{
    // read parameters
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    rate = new Rate(loop_rate);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    nh.param(this_node::getName() + "/goal_tolerance", goal_tolerance, 0.1);
    nh.param(this_node::getName() + "/yaw_tolerance", yaw_tolerance, 0.02);

    // no pose received yet
    pose_valid = false;

    // init ros communication
    out_of_bounds_client = nh.serviceClient<cpswarm_msgs::OutOfBounds>("area/out_of_bounds");
    out_of_bounds_client.waitForExistence();
    pose_sub = nh.subscribe("pos_provider/pose", queue_size, &action_server::pose_callback, this);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pos_controller/goal_position", queue_size, true);

    // init position and yaw
    while (ok() && pose_valid == false) {
        ROS_DEBUG_ONCE("Waiting for valid pose...");
        rate->sleep();
        spinOnce();
    }

    // start the action server
    server.start();
}

action_server::~action_server ()
{
    delete rate;
}

double action_server::dist (geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
    return hypot(p1.position.x - p2.position.x, p1.position.y - p2.position.y);
}

double action_server::get_yaw (geometry_msgs::Pose pose)
{
    tf2::Quaternion orientation;
    tf2::fromMsg(pose.orientation, orientation);
    return tf2::getYaw(orientation);
}

bool action_server::out_of_bounds (geometry_msgs::Pose pose)
{
    cpswarm_msgs::OutOfBounds oob;
    oob.request.pose = pose;
    if (out_of_bounds_client.call(oob)){
        return oob.response.out;
    }
    else{
        ROS_ERROR("Failed to check if goal is out of bounds");
        return true;
    }
}

bool action_server::reached (geometry_msgs::Pose goal)
{
    ROS_DEBUG("Yaw %.2f --> %.2f", get_yaw(pose), get_yaw(goal));
    ROS_DEBUG("Pose (%.2f,%.2f) --> (%.2f,%.2f)", pose.position.x, pose.position.y, goal.position.x, goal.position.y);
    ROS_DEBUG("%.2f > %.2f OR %.2f > %.2f", dist(pose, goal), goal_tolerance, abs(remainder(get_yaw(pose) - get_yaw(goal), 2*M_PI)), yaw_tolerance);

    return dist(pose, goal) <= goal_tolerance && abs(remainder(get_yaw(pose) - get_yaw(goal), 2*M_PI)) <= yaw_tolerance;
}

void action_server::pose_callback (const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // valid pose received
    if (msg->header.stamp.isValid())
        pose_valid = true;

    // store new position and orientation in class variables
    pose = msg->pose;

    ROS_DEBUG_THROTTLE(1, "Yaw %.2f", get_yaw(pose));
    ROS_DEBUG_THROTTLE(1, "Pose [%.2f, %.2f, %.2f]", pose.position.x, pose.position.y, pose.position.z);
}

void action_server::moveto_callback(const move_base_msgs::MoveBaseGoal::ConstPtr& goal)
{
    // action server result
    move_base_msgs::MoveBaseFeedback feedback;

    // goal is out of bounds
    if (out_of_bounds(goal->target_pose.pose)) {
        ROS_ERROR("Cannot move to (%.2f,%.2f) because it is out of bounds!", goal->target_pose.pose.position.x, goal->target_pose.pose.position.y);
        server.setAborted();
        return;
    }

    // send goal pose to cps controller
    pose_pub.publish(goal->target_pose);

    ROS_ERROR("Move to (%.2f,%.2f)", goal->target_pose.pose.position.x, goal->target_pose.pose.position.y);

    // wait until cps reached goal
    while (ok() && reached(goal->target_pose.pose) == false && server.isPreemptRequested() == false) {
        // provide feedback to client
        geometry_msgs::PoseStamped ps;
        ps.header.stamp = Time::now();
        ps.pose = pose;
        feedback.base_position = ps;
        server.publishFeedback(feedback);

        // wait
        rate->sleep();

        // check if reached goal
        spinOnce();
    }

    // action server has been preempted
    if (server.isPreemptRequested()) {
        server.setPreempted();
    }

    // return position
    else {
        server.setSucceeded();
    }
}
