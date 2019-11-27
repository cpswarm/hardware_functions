#include "lib/action_server.h"

action_server::action_server () : server(nh, "picknplace", boost::bind(&action_server::picknplace_callback, this, _1), false)
{
    // read parameters
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    rate = new Rate(loop_rate);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    
    //timeout to connect with the services
    ros::Duration timeout = ros::Duration(5);

    // init ros communication
    rlc_status_sub = nh.subscribe("robot_local_control/state", queue_size, &action_server::status_callback, this);
    pick_client = nh.serviceClient<robot_local_control_msgs::Pick>("robot_local_control/NavigationComponent/PickComponent/add");
    goto_client = nh.serviceClient<robot_local_control_msgs::Goto>("robot_local_control/NavigationComponent/GotoComponent/add");
    place_client = nh.serviceClient<robot_local_control_msgs::Place>("robot_local_control/NavigationComponent/PlaceComponent/add");

    //services for rlc
    servicegoto_add_rlc_                =pnh_.serviceClient<robot_local_control_msgs::GoToPetition> ("robot_local_control/NavigationComponent/GoToComponent/add");
    servicegoto_cancel_rlc_             =pnh_.serviceClient<procedures_msgs::ProcedureQuery>        ("robot_local_control/NavigationComponent/GoToComponent/cancel");
    servicegoto_pause_rlc_              =pnh_.serviceClient<procedures_msgs::ProcedureQuery>        ("robot_local_control/NavigationComponent/GoToComponent/pause");
    servicegoto_query_state_rlc_        =pnh_.serviceClient<procedures_msgs::ProcedureQuery>        ("robot_local_control/NavigationComponent/GoToComponent/query_state");
    servicegoto_resume_rlc_             =pnh_.serviceClient<procedures_msgs::ProcedureQuery>        ("robot_local_control/NavigationComponent/GoToComponent/resume");

    servicepick_add_rlc_                =pnh_.serviceClient<robot_local_control_msgs::PickPetition> ("robot_local_control/NavigationComponent/PickComponent/add");
    servicepick_cancel_rlc_             =pnh_.serviceClient<procedures_msgs::ProcedureQuery>        ("robot_local_control/NavigationComponent/PickComponent/cancel");
    servicepick_pause_rlc_              =pnh_.serviceClient<procedures_msgs::ProcedureQuery>        ("robot_local_control/NavigationComponent/PickComponent/pause");
    servicepick_query_state_rlc_        =pnh_.serviceClient<procedures_msgs::ProcedureQuery>        ("robot_local_control/NavigationComponent/PickComponent/query_state");
    servicepick_resume_rlc_             =pnh_.serviceClient<procedures_msgs::ProcedureQuery>        ("robot_local_control/NavigationComponent/PickComponent/resume");

    serviceplace_add_rlc_               =pnh_.serviceClient<robot_local_control_msgs::PlacePetition> ("robot_local_control/NavigationComponent/PlaceComponent/add");
    serviceplace_cancel_rlc_            =pnh_.serviceClient<procedures_msgs::ProcedureQuery>         ("robot_local_control/NavigationComponent/PlaceComponent/cancel");
    serviceplace_pause_rlc_             =pnh_.serviceClient<procedures_msgs::ProcedureQuery>         ("robot_local_control/NavigationComponent/PlaceComponent/pause");
    serviceplace_query_state_rlc_       =pnh_.serviceClient<procedures_msgs::ProcedureQuery>         ("robot_local_control/NavigationComponent/PlaceComponent/query_state");
    serviceplace_resume_rlc_            =pnh_.serviceClient<procedures_msgs::ProcedureQuery>         ("robot_local_control/NavigationComponent/PlaceComponent/resume");


    
    //checking the connection with the services
    if(pick_client.waitForExistence(timeout) == false){
        ROS_ERROR("Cannot connect to service "<< pick_client.getService());
    }
    if(goto_client.waitForExistence(timeout) == false){
        ROS_ERROR("Cannot connect to service "<< goto_client.getService());
    }
    if(place_client.waitForExistence(timeout) == false){
        ROS_ERROR("Cannot connect to service "<< place_client.getService());
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

//callback to update the state of RLC
void action_server::status_callback(robot_local_control_msgs::Status msg){

    rlc_status_ = msg;

}


void action_server::picknplace_callback(const move_base_msgs::MoveBaseGoal::ConstPtr& goal) cambiar el mensaje al de pinknplace
{
    // action server result
    move_base_msgs::MoveBaseFeedback feedback;

    //rlc_goto_msg    
    robot_local_control_msgs::Goto goto_msg;    
    goto_msg = 

    //call goto service of RLC
    if(goto_client.call())

    //wait until the goto service ends
    while(ok()){

        // wait
        rate->sleep();

        // check if reached goal
        spinOnce();
    }
    //rlc_pick_msg

    //call pick service of RLC

    //wait until the pick service ends

    //rlc_goto_msg

    //call goto service of RLC

    //wait until the goto service ends

    //rlc_place_msg

    //call place service of RLC

    //wait until the place service ends
    

    // wait until cps reached goal
    /*while (ok() && reached(goal->target_pose.pose) == false && server.isPreemptRequested() == false) {
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
    }*/

    // action server has been preempted
    if (server.isPreemptRequested()) {
        server.setPreempted();
    }

    // return position
    else {
        server.setSucceeded();
    }
}
