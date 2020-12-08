#include "ros/ros.h"          // 加入ROS公用程序
#include "std_msgs/Int64.h"  // 所要publish的message header，在此是std_msgs package底下的String.msg
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <sstream>
#include <iostream>
#include <stdio.h>

/* ===============================
  1. Publish Topic: /State_Cmd
                    /Beacon_Target
                      1 : Beacon-600
                      2 : Beacon-1500

  2. Subscribe Topic: /robot_state
                      /IR_ratio
================================== */

// ========= Publishers ========== //
ros::Publisher Start_pub;
ros::Publisher StateCmd_pub;
ros::Publisher BeaconTarget_pub;

// ==========  Messages ==========  //
std_msgs::Int64 start_cmd;
std_msgs::Int64 beacon_target; 
std_msgs::Int64 robot_state_cmd;


void robot_state_Callback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Robot state: %s",msg->data.c_str());
}

void IR_ratio_Callback(const std_msgs::Float32::ConstPtr& msg)
{
  if(msg->data <=0.23){
    ROS_INFO("IR ratio: %f, should go to BEACON 2 (-1500)",msg->data);
  }else{
    ROS_INFO("IR ratio: %f, should go to BEACON 1 (-600)",msg->data);
  }
  // ROS_INFO("IR ratio: %f",msg->data);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "RPi_cp4_node");  //一開始必須先初始化，指定node名稱為talker

  ros::NodeHandle n;     
  
  Start_pub = n.advertise<std_msgs::Int64>("start", 1);
  StateCmd_pub = n.advertise<std_msgs::Int64>("State_Cmd", 1);
  BeaconTarget_pub = n.advertise<std_msgs::Int64>("Beacon_Target", 1);
  ros::Subscriber RPi_sub = n.subscribe("robot_state", 1, robot_state_Callback);  
  ros::Subscriber IR_sub = n.subscribe("IR_ratio", 1, IR_ratio_Callback);  


  // ros::Rate loop_rate(10);   // 10Hz

  // while (ros::ok())
  // {
    ROS_INFO("Enter a number to start: "); 
    std::cin>>start_cmd.data;
    ROS_INFO("State cmd: "); 
    ROS_INFO("Finding_Puck: 0"); 
    ROS_INFO("Finding_Beacon: 1"); 
    ROS_INFO("IDLE: 2"); 
    std::cin>>robot_state_cmd.data;
    ROS_INFO("Beacon Target: "); 
    ROS_INFO("Beacon-600: 1"); 
    ROS_INFO("Beacon-1500: 2"); 
    std::cin>> beacon_target.data;

    Start_pub.publish(start_cmd);
    StateCmd_pub.publish(robot_state_cmd);
    BeaconTarget_pub.publish(beacon_target);

    ros::spin();
    // ros::spinOnce();   // 呼叫一次 callback function，在subscriber才有用

    // loop_rate.sleep(); 
  // }

  return 0;
}