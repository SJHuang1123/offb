#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int32

current_state = State()
takeoff = True
cur_alt = float(0)

def mqtt_cb(msg):
    if msg == 1:
        takeoff = True

def state_cb(msg):
    global current_state
    current_state = msg

def pose_cb(msg):
    cur_alt = msg.pose.z

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    local_pos_sub = rospy.Subscriber("/mavros/local_position/pose_initialized", PoseStamped, callback = pose_cb)
    mqtt_sub = rospy.Subscriber("pp", Int32, callback = mqtt_cb)

    local_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()
    while takeoff == False:
        rate.sleep()

    twist = TwistStamped()

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_vel_pub.publish(twist)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()
    goal = 2.5

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()

        if cur_alt - goal < 0.2:
            err = goal - cur_alt
            if err > 2:
                err = 2
                twist.twist.linear.z = err / 2
                local_vel_pub.publish(twist)
            if  0.1 < err < 2:
                twist.twist.linear.z = err / 2
                local_vel_pub.publish(twist)    
            if err < 0.1:
                twist.twist.linear.z = 0
                local_vel_pub.publish(twist)

        rate.sleep()