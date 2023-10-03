#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Int32.h>
#include <cmath>
#include <tf/tf.h>
#include <geometry_msgs/Point.h>
#include <queue>

using namespace std;
bool takeoff = false;
mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void mqtt_cb(const std_msgs::Int32::ConstPtr& msg){
    if(msg->data == 1)
        takeoff = true;
}

int main(int argc, char **argv)
{
    //  ROS_initialize  //
    ros::init(argc, argv, "offb_node2");
    ros::NodeHandle nh;
    //    subscriber    //
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 100, state_cb);
    ros::Subscriber mqtt_sub = nh.subscribe<std_msgs::Int32>("/pp", 100, mqtt_cb);
    // publisher
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 2);
    // service
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Rate rate(100);
    
    ROS_INFO("Wait for FCU connection");
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Wait for FCU");
    }
    ROS_INFO("FCU connected");
    while (ros::ok() && takeoff == false) {
        ros::spinOnce();
        rate.sleep();
        //ROS_INFO("Wait for FCU");
    }
    ROS_INFO("takeoff requested");
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "GUIDED";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();
    
    if( arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed");
    }

    ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = 2.5;
    if(takeoff_cl.call(srv_takeoff))
    {
        ROS_INFO("srv_takeoff send success %d", srv_takeoff.response.success);
    }
    else
    {
        ROS_ERROR("Takeoff failed");
	return 0;
    }
	
    sleep(10);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}


