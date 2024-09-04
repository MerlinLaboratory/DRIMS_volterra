/* TASK SERVER EXAMPLE to call task server example service */
#include "ros/ros.h"
#include <iostream>
#include "ros/service_client.h"

// Object Includes
#include "abb_wrapper_control/TaskSequencer.h"

/**********************************************
ROS NODE MAIN TASK SEQUENCE SERVER 
**********************************************/
int main(int argc, char **argv)
{    
    ros::init(argc, argv, "task_server");

    ros::NodeHandle nh_;

    ROS_INFO("Creating the TaskSequencer object");

    TaskSequencer task_sequencer_obj(nh_);

    ROS_INFO("The main task sequence client is running. Running as fast as possible!");

    // ROS Async spinner (necessary for processing callbacks inside the service callbacks)
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::waitForShutdown();
    spinner.stop();

    return 0;
}