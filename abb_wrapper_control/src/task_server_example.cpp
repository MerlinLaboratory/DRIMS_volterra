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
    ros::init(argc, argv, "task_server_example");

    ros::NodeHandle nh_;

    ROS_INFO("Creating the TaskSequencer object");

    TaskSequencer task_sequencer_obj(nh_);

    ROS_INFO("The main task sequence client is running. Running as fast as possible!");

    // ROS Async spinner (necessary for processing callbacks inside the service callbacks)
    ros::AsyncSpinner spinner(2);
    spinner.start();
   
   // /* 1) HIGH LEVEL TASK*/
   // //Create the request and response object
      
   // std_srvs::SetBool::Request req;
   // req.data = true;
   // std_srvs::SetBool::Response resp;

   // ROS_INFO("Call Example Task");
   
   // bool success = task_sequencer_obj.call_example_task(req,resp); //This function is defined in "TaskSequencer.cpp"
    
   // // Check the success and use of the response

   // if(success){
   //    ROS_INFO_STREAM("Call Example Task completed correctly: " << resp.success);
   // } else {
   //    ROS_INFO_STREAM("Failed to completed the service");
   // }
   ros::waitForShutdown();
   spinner.stop();

   return 0;
}