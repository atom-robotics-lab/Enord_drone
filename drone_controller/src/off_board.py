#! /usr/bin/env python3

'''This code controls the movement of three drones.
The drones start by ascending to a height of 1.5 meters.
Then, they move in a square pattern for 150 seconds.
Finally, they land.

The state variable is used to keep track of the current state of the drones.
The state can be 0, 1, or 2.
State 0 is the ascent state.
State 1 is the square pattern state.
State 2 is the landing state.'''

# Import necessary libraries
import rospy
from sensor_msgs.msg import TimeReference, Image
from geometry_msgs.msg import PoseStamped,TwistStamped, Twist
from mavros_msgs.msg import State, Thrust
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, ParamPush,ParamPushRequest,ParamSet,ParamSetRequest,ParamSetResponse

# Global variables
current_state0 = State()
current_state1 = State()
current_state2 = State()
current_orien0 = PoseStamped()
current_orien1 = PoseStamped()
current_orien2 = PoseStamped()
current_vel0 = TwistStamped()
current_vel1 = TwistStamped()
current_vel2 = TwistStamped()
time_stamp0 = TimeReference()
time_stamp1 = TimeReference()
time_stamp2 = TimeReference()

reached = False
land = False
x_error0 = 0
y_error0= 0
x_error1 = 0
y_error1 = 0
x_error2 = 0
y_error2 = 0
state = 0
object_detected = False
flag_inititate = True
check_tag = False    

# Callback functions

# Image Callback
def aruco_cb0(msg):
    global x_error0, y_error0
    x_error0 = msg.linear.x
    y_error0 = msg.linear.y
def aruco_cb1(msg):
    global x_error1, y_error1
    x_error1 = msg.linear.x
    y_error1 = msg.linear.y
def aruco_cb2(msg):
    global x_error2, y_error2
    x_error2 = msg.linear.x
    y_error2 = msg.linear.y    

# Uav0 Callback
def state_cb0(msg):
    global current_state0
    current_state0 = msg
def orien_cb0(msg):
    global current_orien0
    current_orien0 =  msg
def vel_cb0(msg):
    global current_vel0
    current_vel0 =  msg
def time_cb0(msg):
    global time_stamp0
    time_stamp0 = msg

# Uav1 Callback
def state_cb1(msg):
    global current_state1
    current_state1 = msg
def orien_cb1(msg):
    global current_orien1
    current_orien1 =  msg
def vel_cb1(msg):
    global current_vel1
    current_vel1 =  msg
def time_cb1(msg):
    global time_stamp1
    time_stamp1 = msg

# Uav2 Callback
def state_cb2(msg):
    global current_state2
    current_state2 = msg
def orien_cb2(msg):
    global current_orien2
    current_orien2 =  msg
def vel_cb2(msg):
    global current_vel2
    current_vel2 =  msg
def time_cb2(msg):
    global time_stamp2
    time_stamp2 = msg

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    # Subscribe to necessary topics

    #Uav0
    state_sub0 = rospy.Subscriber("/uav0/mavros/state", State, callback = state_cb0)
    orientation_sub0 = rospy.Subscriber("/uav0/mavros/local_position/pose",PoseStamped, callback= orien_cb0)
    vel_sub0 = rospy.Subscriber("uav0/mavros/local_position/velocity_body",TwistStamped, callback= vel_cb0)
    local_pos_pub0 = rospy.Publisher( "uav0/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    local_vel_pub0 = rospy.Publisher("uav0/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    rospy.wait_for_service("uav0/mavros/cmd/arming")
    arming_client0 = rospy.ServiceProxy("uav0/mavros/cmd/arming", CommandBool)
    param_client0 = rospy.ServiceProxy("mavros_msgs/ParamSet", ParamSet)    
    rospy.wait_for_service("uav0/mavros/set_mode")
    set_mode_client0 = rospy.ServiceProxy("uav0/mavros/set_mode", SetMode)
    
    #Uav1
    state_sub1 = rospy.Subscriber("/uav1/mavros/state", State, callback = state_cb1)
    orientation_sub1 = rospy.Subscriber("/uav1/mavros/local_position/pose",PoseStamped, callback= orien_cb1)
    vel_sub1 = rospy.Subscriber("uav1/mavros/local_position/velocity_body",TwistStamped, callback= vel_cb1)
    local_pos_pub1 = rospy.Publisher( "uav1/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    local_vel_pub1 = rospy.Publisher("uav1/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    rospy.wait_for_service("uav1/mavros/cmd/arming")
    arming_client1 = rospy.ServiceProxy("uav1/mavros/cmd/arming", CommandBool)
    param_client1 = rospy.ServiceProxy("mavros_msgs/ParamSet", ParamSet)    
    rospy.wait_for_service("uav1/mavros/set_mode")
    set_mode_client1 = rospy.ServiceProxy("uav1/mavros/set_mode", SetMode)

    #Uav2 
    state_sub2 = rospy.Subscriber("/uav2/mavros/state", State, callback = state_cb2)
    orientation_sub2 = rospy.Subscriber("/uav2/mavros/local_position/pose",PoseStamped, callback= orien_cb2)
    vel_sub2 = rospy.Subscriber("uav2/mavros/local_position/velocity_body",TwistStamped, callback= vel_cb2)
    local_pos_pub2 = rospy.Publisher( "uav2/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    local_vel_pub2 = rospy.Publisher("uav2/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    rospy.wait_for_service("uav2/mavros/cmd/arming")
    arming_client2 = rospy.ServiceProxy("uav2/mavros/cmd/arming", CommandBool)
    param_client2 = rospy.ServiceProxy("mavros_msgs/ParamSet", ParamSet)    
    rospy.wait_for_service("uav2/mavros/set_mode")
    set_mode_client2 = rospy.ServiceProxy("uav2/mavros/set_mode", SetMode)

    #Image Topic Subscriber
    aruco_detection0 = rospy.Subscriber("/aruco_detection0", Twist, callback=aruco_cb0)
    aruco_detection1 = rospy.Subscriber("/aruco_detection1", Twist, callback=aruco_cb1)
    aruco_detection2 = rospy.Subscriber("/aruco_detection2", Twist, callback=aruco_cb2)


    rate = rospy.Rate(20)
    while(not rospy.is_shutdown() and current_state0.connected):

        # Create TwistStamped messages for velocity control
        vel_msg0 = TwistStamped()
        vel_msg1 = TwistStamped()
        vel_msg2 = TwistStamped()

        # Create TwistStamped messages for velocity control
        param_set_mode0 = ParamSetRequest()
        param_set_mode0.param_id = "NAV_RCL_ACT"
        param_set_mode0.value = 0

        param_set_mode1 = ParamSetRequest()
        param_set_mode1.param_id = "NAV_RCL_ACT"
        param_set_mode1.value = 0

        param_set_mode2 = ParamSetRequest()
        param_set_mode2.param_id = "NAV_RCL_ACT"
        param_set_mode2.value = 0

        # Create SetModeRequest messages to switch to OFFBOARD mode
        offb_set_mode0 = SetModeRequest()
        offb_set_mode1 = SetModeRequest()
        offb_set_mode2 = SetModeRequest()

        offb_set_mode0.custom_mode = 'OFFBOARD'
        offb_set_mode1.custom_mode = 'OFFBOARD'
        offb_set_mode2.custom_mode = 'OFFBOARD'
    
        # Create CommandBoolRequest messages to arm the vehicles
        arm_cmd0 = CommandBoolRequest()
        arm_cmd0.value = True
        arm_cmd1 = CommandBoolRequest()
        arm_cmd1.value = True
        arm_cmd2 = CommandBoolRequest()
        arm_cmd2.value = True

        last_req = rospy.Time.now()
        break

    while(not rospy.is_shutdown()):
        # Control logic and velocity commands go here


        if (reached == False and object_detected == False):
            vel_msg0.header.stamp = rospy.Time.now()
            local_vel_pub0.publish(vel_msg0)
        if (reached == False and object_detected == False):
            vel_msg1.header.stamp = rospy.Time.now()
            local_vel_pub1.publish(vel_msg1)
        if (reached == False and object_detected == False):
            vel_msg2.header.stamp = rospy.Time.now()
            local_vel_pub2.publish(vel_msg2)

        # Check if the current state is not "OFFBOARD"
        if(current_state0.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            # Set the custom mode to "OFFBOARD" using the set_mode_client service
            if(set_mode_client0.call(offb_set_mode0).mode_sent == True):
                rospy.loginfo("OFFBOARD0 enabled")
            if(set_mode_client1.call(offb_set_mode1).mode_sent == True):
                rospy.loginfo("OFFBOARD1 enabled")
            if(set_mode_client2.call(offb_set_mode2).mode_sent == True):
                rospy.loginfo("OFFBOARD2 enabled")
            
            last_req = rospy.Time.now()
        else:
            # Check if the vehicles are not armed
            if(not current_state0.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                # Arm the vehicles using the arming_client service
                if(arming_client0.call(arm_cmd0).success == True):
                    rospy.loginfo("Vehicle armed0")
                if(arming_client1.call(arm_cmd1).success == True):
                    rospy.loginfo("Vehicle armed1")
                if(arming_client2.call(arm_cmd2).success == True):
                    rospy.loginfo("Vehicle armed2")

                last_req = rospy.Time.now()
       
        # Get position and orientation values for each vehicle

        #Uav0
        x_val0 = current_orien0.pose.position.x
        y_val0 = current_orien0.pose.position.y
        z_val0 = current_orien0.pose.position.z
        yaw0 = current_orien0.pose.orientation.w

        #Uav1
        x_val1 = current_orien1.pose.position.x
        y_val1 = current_orien1.pose.position.y
        z_val1 = current_orien1.pose.position.z
        yaw1 = current_orien1.pose.orientation.w

        #Uav2
        x_val2 = current_orien2.pose.position.x
        y_val2 = current_orien2.pose.position.y
        z_val2 = current_orien2.pose.position.z
        yaw2 = current_orien2.pose.orientation.w

        # Check the state variable and perform corresponding actions
        if state == 0 and current_state1.armed and current_state2.armed and current_state0.armed:
        # Takeoff Started

            # Check if the z-value is less than 6 for the third vehicle
            if z_val2 < 6:

                # Increase the linear velocity in the z-axis to ascend
                vel_msg0.twist.linear.z = 0.8
                vel_msg1.twist.linear.z = 0.8
                vel_msg2.twist.linear.z = 0.8

            else:
                # Set the linear velocities in the z-axis to 0
                vel_msg0.twist.linear.z = 0

                # Check the state variable and perform corresponding actions
                offb_set_mode0.custom_mode = "AUTO.LOITER"
                if (set_mode_client0.call(offb_set_mode0).mode_sent == True):
                    rospy.loginfo("AUTO.LOITER mode enable")
                vel_msg1.twist.linear.z = 0
                offb_set_mode1.custom_mode = "AUTO.LOITER"
                if (set_mode_client1.call(offb_set_mode1).mode_sent == True):
                    rospy.loginfo("AUTO.LOITER mode enable")
                vel_msg2.twist.linear.z = 0
                offb_set_mode2.custom_mode = "AUTO.LOITER"
                if (set_mode_client2.call(offb_set_mode2).mode_sent == True):
                    rospy.loginfo("AUTO.LOITER mode enable")
                state = 1
                rospy.loginfo("Height Reached")
                rospy.loginfo("State Changed to 1")
        elif state == 1:
            # Move in a square pattern for 150 seconds

            offb_set_mode0.custom_mode = "OFFBOARD"
            if (set_mode_client0.call(offb_set_mode0).mode_sent == True):
                rospy.loginfo("OFFBOARD mode enable")
            offb_set_mode1.custom_mode = "OFFBOARD"
            if (set_mode_client1.call(offb_set_mode1).mode_sent == True):
                rospy.loginfo("OFFBOARD mode enable")
            offb_set_mode2.custom_mode = "OFFBOARD"
            if (set_mode_client2.call(offb_set_mode2).mode_sent == True):
                rospy.loginfo("OFFBOARD mode enable")
            
            rate = rospy.Rate(20)
            for i in range(150):
                # Move in forward direcrion
                rospy.loginfo("Forward ")
                vel_msg0.twist.linear.x = 1
                local_vel_pub0.publish(vel_msg0)
                vel_msg1.twist.linear.x = 1
                local_vel_pub1.publish(vel_msg1)
                vel_msg2.twist.linear.x = 1
                local_vel_pub2.publish(vel_msg2)
        
                rate.sleep()
            
            for i in range(150):
                # Move in right direcrion
                rospy.loginfo("Right ")
                vel_msg0.twist.linear.x = 0
                vel_msg0.twist.linear.y = 1
                local_vel_pub0.publish(vel_msg0)
                vel_msg1.twist.linear.x = 0
                vel_msg1.twist.linear.y = 1
                local_vel_pub1.publish(vel_msg1)
                vel_msg2.twist.linear.x = 0
                vel_msg2.twist.linear.y = 1
                local_vel_pub2.publish(vel_msg2)
                rate.sleep()

            for i in range(150):
                # Move in backward direcrion
                rospy.loginfo("Backward ")
                vel_msg0.twist.linear.x = -1
                vel_msg0.twist.linear.y = 0
                local_vel_pub0.publish(vel_msg0)
                vel_msg1.twist.linear.x = -1
                vel_msg1.twist.linear.y = 0
                local_vel_pub1.publish(vel_msg1)
                vel_msg2.twist.linear.x = -1
                vel_msg2.twist.linear.y = 0
                local_vel_pub2.publish(vel_msg2)
                rate.sleep()

            for i in range(150):
                # Move in left direcrion
                rospy.loginfo("Left ")
                vel_msg0.twist.linear.x = 0
                vel_msg0.twist.linear.y = -1
                local_vel_pub0.publish(vel_msg0)
                vel_msg1.twist.linear.x = 0
                vel_msg1.twist.linear.y = -1
                local_vel_pub1.publish(vel_msg1)
                vel_msg2.twist.linear.x = 0
                vel_msg2.twist.linear.y = -1
                local_vel_pub2.publish(vel_msg2)
                rate.sleep()

            vel_msg0.twist.linear.x = 0
            vel_msg0.twist.linear.y = 0
            vel_msg1.twist.linear.x = 0
            vel_msg1.twist.linear.y = 0
            vel_msg2.twist.linear.x = 0
            vel_msg2.twist.linear.y = 0
            rospy.loginfo("Square Complete")
            rospy.loginfo("State Changed to 2")
            rospy.loginfo("Landing Started")
            state = 2            

        elif state == 2:
            # Landing started

            # Mode change from "AUTO.LOITER" to "OFFBOARD"
            if offb_set_mode0.custom_mode == "AUTO.LOITER":
                offb_set_mode0.custom_mode = "OFFBOARD"
                if (set_mode_client0.call(offb_set_mode0).mode_sent == True):
                    rospy.loginfo("OFFBOARD mode enable")
            if offb_set_mode1.custom_mode == "AUTO.LOITER":
                offb_set_mode1.custom_mode = "OFFBOARD"
                if (set_mode_client1.call(offb_set_mode1).mode_sent == True):
                    rospy.loginfo("OFFBOARD mode enable")
            if offb_set_mode2.custom_mode == "AUTO.LOITER":
                offb_set_mode2.custom_mode = "OFFBOARD"
                if (set_mode_client2.call(offb_set_mode2).mode_sent == True):
                    rospy.loginfo("OFFBOARD mode enable")
            else:
                #Uav0 Landing
                if z_val0 > 1.1:
                    vel_msg0.twist.linear.z = -0.3
                    vel_msg0.twist.linear.x = y_error0* 0.2
                    vel_msg0.twist.linear.y = -x_error0* 0.15
                else:
                    vel_msg0.twist.linear.z = -0.3
                    vel_msg0.twist.linear.x = 0
                    vel_msg0.twist.linear.y = 0
                    offb_set_mode0.custom_mode = "AUTO.LAND"
                    if (set_mode_client0.call(offb_set_mode0).mode_sent == True):
                        rospy.loginfo("AUTO.LAND mode enable")
                        rospy.loginfo("Landing Completed")
                        rospy.loginfo("Destination Reached")

                #Uav1 Landing
                if z_val1 > 1.1:
                    vel_msg1.twist.linear.z = -0.3
                    vel_msg1.twist.linear.x = y_error1* 0.2
                    vel_msg1.twist.linear.y = -x_error1* 0.15
                else:
                    vel_msg1.twist.linear.z = -0.3
                    vel_msg1.twist.linear.x = 0
                    vel_msg1.twist.linear.y = 0
                    offb_set_mode1.custom_mode = "AUTO.LAND"    # Create TwistStamped messages for velocity control

                #Uav2 Landing
                if z_val2 > 1.1:
                    vel_msg2.twist.linear.z = -0.3
                    vel_msg2.twist.linear.x = y_error2* 0.2
                    vel_msg2.twist.linear.y = -x_error2* 0.15
                else:
                    vel_msg2.twist.linear.z = -0.3
                    vel_msg2.twist.linear.x = 0
                    vel_msg2.twist.linear.y = 0
                    offb_set_mode2.custom_mode = "AUTO.LAND"
                    if (set_mode_client2.call(offb_set_mode2).mode_sent == True):
                        rospy.loginfo("AUTO.LAND mode enable")
                        rospy.loginfo("Landing Completed")
                        rospy.loginfo("Destination Reached")                       
        rate.sleep()