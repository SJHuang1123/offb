#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Altitude.h>
#include <std_msgs/Int32.h>
#include <math.h>
mavros_msgs::State state;
float current_alt;
geometry_msgs::TwistStamped twist;
bool shut = false;
bool takeoff = false;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    state = *msg;
}

void mqtt_cb(const std_msgs::Int32::ConstPtr& msg){
    if (msg->data == 3)shut = true;
    else if (msg->data == 1)takeoff = true;
    
}

void altitude_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_alt = msg->pose.position.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "altitude");
    ros::NodeHandle nh;
    ros::Subscriber alt_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose_initialized", 10, altitude_cb);
    ros::Subscriber mqtt_sub = nh.subscribe<std_msgs::Int32>
            ("/pp", 10, mqtt_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);                       
    ros::Publisher altitude_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("/mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    ros::Rate loop_rate(20);
    
    for(int i = 0; i < 100; i++){
        altitude_pub.publish(twist);
        loop_rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    
    float goal = 3.0;
    ros::Time last_request = ros::Time::now();
    while (ros::ok())
    {   
        if (state.mode != "OFFBOARD" && ros::Time::now() - last_request > ros::Duration(5.0))
        {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        if (((current_alt - goal) <= 0.2 || ((goal - current_alt) <= 0.2)) && takeoff == true)
        {
            float error = goal - current_alt;
            if (error > 2) error = 2;
            twist.twist.linear.z = error/2;        
            if (error < 0.1){
                twist.twist.linear.x=0;
                twist.twist.angular.z=0;
            }
            altitude_pub.publish(twist);
        }        

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
