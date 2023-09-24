/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/CommandTOL.h>
#include <std_msgs/Int32.h>

mavros_msgs::State current_state;
bool takeoff_flag = true;
bool arming_flag = true;
bool land_flag = false;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void mqtt_cb(const std_msgs::Int32::ConstPtr& msg){
    if(msg->data == 1)
        takeoff_flag = true;
    else if(msg->data == 2)
        arming_flag = true;
    else if(msg->data == 3)
        land_flag = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    // ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    //         ("mavros/setpoint_position/local", 10);
    ros::ServiceClient land_cli = nh.serviceClient<mavros_msgs::CommandTOL>
            ("/mavros/cmd/land", 10);
    ros::ServiceClient takeoff_cli = nh.serviceClient<mavros_msgs::CommandTOL>
            ("/mavros/cmd/takeoff", 10);        
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::Subscriber mqtt_sub = nh.subscribe<std_msgs::Int32>("/pp", 10, mqtt_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    while(ros::ok() && takeoff_flag == false){
        ros::spinOnce();
        rate.sleep();
    }
    mavros_msgs::CommandTOL tol;
    takeoff_cli.call(tol);
    ROS_INFO("takeoff requested");

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    bool land_successfully = false;

    ros::Time last_request = ros::Time::now();
    while(ros::ok()){
        if( !current_state.armed &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if(arming_flag == true && land_successfully == false){
                arming_client.call(arm_cmd);
                if(arm_cmd.response.success)
                    ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
        if(land_flag == true && land_successfully == false){
            if(land_cli.call(tol) && tol.response.success){
                ROS_INFO("landing");
                land_successfully = true;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}