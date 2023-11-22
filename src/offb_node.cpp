/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/CommandTOL.h>
#include <std_msgs/Int32.h>

mavros_msgs::State current_state;
geometry_msgs::TwistStamped twist;
bool takeoff_flag = true;
bool arming_flag = true;
bool shut = false;
float current_x;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_x = msg->pose.position.x;
}
void mqtt_cb(const std_msgs::Int32::ConstPtr& msg){
    if(msg->data == 1)
        takeoff_flag = true;
    else if(msg->data == 2)
        arming_flag = true;
    else if (msg->data == 4)
    {
        shut = false;
    }
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    // subscriber
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber alt_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose_initialized", 10, pose_cb);
    ros::Subscriber mqtt_sub = nh.subscribe<std_msgs::Int32>("/pp", 10, mqtt_cb);
    //serviceclient
    ros::ServiceClient land_cli = nh.serviceClient<mavros_msgs::CommandTOL>
            ("/mavros/cmd/land", 10);
    ros::ServiceClient takeoff_cli = nh.serviceClient<mavros_msgs::CommandTOL>
            ("/mavros/cmd/takeoff", 10);        
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    //publisher
    ros::Publisher altitude_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("/mavros/setpoint_velocity/cmd_vel", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate loop_rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        loop_rate.sleep();
    }
    while(ros::ok() && takeoff_flag == false){
        ros::spinOnce();
        loop_rate.sleep();
    }
    mavros_msgs::CommandTOL tol;
    takeoff_cli.call(tol);
    ROS_INFO("takeoff requested");

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    while(ros::ok() && shut == false){
        if( !current_state.armed &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if(arming_flag == true){
                arming_client.call(arm_cmd);
                if(arm_cmd.response.success)
                    ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}