#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <math.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#include "ros/ros.h"

geometry_msgs::TwistStamped twist;
mavros_msgs::State state;
bool shut = false;
bool block = false;
bool forward = true;
float current_x;

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_x = msg->pose.position.x;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    state = *msg;
}

void block_cb(const std_msgs::Bool::ConstPtr& msg) {
    block = msg->data;
    ROS_INFO("block is %d", block);
}

void mqtt_cb(const std_msgs::Int32::ConstPtr& msg) {
    if (msg->data == 4)
        shut = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "altitude");
    ros::NodeHandle nh;
    ros::Subscriber alt_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber mqtt_sub = nh.subscribe<std_msgs::Int32>("/pp", 10, mqtt_cb);
    ros::Subscriber block_sub = nh.subscribe<std_msgs::Bool>("/bb", 10, block_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate loop_rate(20);
    while (ros::ok() && forward == false) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    for (int i = 0; i < 100; i++) {
        vel_pub.publish(twist);
        loop_rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    
    float goal = 3.0;
    ros::Time last_request = ros::Time::now();
    while (ros::ok())
    {   
        if(shut == false){

            if (state.mode != "OFFBOARD" && ros::Time::now() - last_request > ros::Duration(5.0))
            {
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            }
            if (((current_x - goal) <= 0.2 || ((goal - current_x) <= 0.2)) && forward == true)
            {
                float error = goal - current_x;
                if (error > 2){
                    error = 2;
                    twist.twist.linear.x = error/2;

                }         
                if (error < 0.1){
                    twist.twist.linear.x = 0;
                    twist.twist.linear.z = 0;
                }
                vel_pub.publish(twist);
            }        
        }
        
        else{
            if (block == true) {
            twist.twist.linear.x = 0.5;
           // twist.twist.linear.z = 0;
            vel_pub.publish(twist);
            }
            else if (block == false) {
                return 0;
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}