#! /usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import TimeReference, Image
from geometry_msgs.msg import PoseStamped,TwistStamped, Twist
from mavros_msgs.msg import State, Thrust
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, ParamPush,ParamPushRequest,ParamSet,ParamSetRequest,ParamSetResponse
from mavros_msgs.srv import ParamGet, ParamGetRequest, ParamGetResponse

current_state = State()
current_orien = PoseStamped()
current_vel = TwistStamped()
time_stamp = TimeReference()
reached = False
land = False
x_error = 0
y_error = 0
state = 0
object_detected = False
flag_inititate = True
check_tag = False
def img_cb(msg):
    global obj_distance
    bridge = CvBridge()
    try:
      cv_image = bridge.imgmsg_to_cv2(msg,desired_encoding='32FC1')
    except CvBridgeError as e:
      print(e)
    obj_distance = cv_image[cv_image.shape[0]//2, cv_image.shape[1]//2]    

def aruco_cb(msg):
    global x_error, y_error
    x_error = msg.linear.x
    y_error = msg.linear.y
    
def state_cb(msg):
    global current_state
    current_state = msg

def orien_cb(msg):
    global current_orien
    current_orien=  msg

def vel_cb(msg):
    global current_vel
    current_vel=  msg

def time_cb(msg):
    global time_stamp
    time_stamp = msg

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    image_sub = rospy.Subscriber("/iris/camera/depth/image_raw", Image,callback=img_cb)
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    orientation_sub = rospy.Subscriber("mavros/local_position/pose",PoseStamped, callback= orien_cb )
    vel_sub = rospy.Subscriber("mavros/local_position/velocity_body",TwistStamped, callback= vel_cb )
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    param_client = rospy.ServiceProxy("mavros_msgs/ParamSet", ParamSet)    
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    aruco_detection = rospy.Subscriber("/aruco_detection", Twist, callback=aruco_cb)

    rate = rospy.Rate(20)
    while(not rospy.is_shutdown() and current_state.connected):
        rate.sleep()
        vel_msg = TwistStamped()
        vel_msg.header.frame_id = "base_link"
        vel_msg.twist.linear.x = 0.0 # set desired x velocity here
        vel_msg.twist.linear.y = 0.0 # set desired y velocity here
        vel_msg.twist.linear.z = 0.4 # set desired z velocity here
        vel_msg.twist.angular.x = 0.0
        vel_msg.twist.angular.y = 0.0
        vel_msg.twist.angular.z = 0.0
        local_vel_pub.publish(vel_msg)
        rate.sleep()

        param_set_mode = ParamSetRequest()
        param_set_mode.param_id = "NAV_RCL_ACT"
        param_set_mode.value = 0

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'
 
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        last_req = rospy.Time.now()
        break

    while(not rospy.is_shutdown()):
        if (reached == False and object_detected == False):
            vel_msg.header.stamp = rospy.Time.now()
            local_vel_pub.publish(vel_msg)

        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            
                last_req = rospy.Time.now()
       
        x_val = current_orien.pose.position.x
        y_val = current_orien.pose.position.y
        z_val = current_orien.pose.position.z
        yaw = current_orien.pose.orientation.w

        if state == 0:
            if z_val < 6:
                vel_msg.twist.linear.z = 0.8
            else:
                vel_msg.twist.linear.z = 0
                offb_set_mode.custom_mode = "AUTO.LOITER"
                if (set_mode_client.call(offb_set_mode).mode_sent == True):
                    rospy.loginfo("AUTO.LOITER mode enable")
                state = 2
                rospy.loginfo("Height Reached")
        elif state == 2:
            if offb_set_mode.custom_mode == "AUTO.LOITER":
                offb_set_mode.custom_mode = "OFFBOARD"
                if (set_mode_client.call(offb_set_mode).mode_sent == True):
                    rospy.loginfo("OFFBOARD mode enable")
            else:
                print(x_error)
                if z_val > 1.3:
                    vel_msg.twist.linear.z = -0.4
                    vel_msg.twist.linear.x = y_error* 0.1
                    vel_msg.twist.linear.y = -x_error* 0.1
                else:
                    print("Detection Complete")
                    vel_msg.twist.linear.z = -0.4
                    vel_msg.twist.linear.x = 0
                    vel_msg.twist.linear.y = 0
                    # offb_set_mode.custom_mode = "AUTO.LAND"
                    # if (set_mode_client.call(offb_set_mode).mode_sent == True):
                    #     rospy.loginfo("AUTO.LAND mode enable")

                    

        # elif z_val > 6 and x_val < 5 and flag2 == False:
        #     if offb_set_mode.custom_mode == "AUTO.LOITER":
        #         offb_set_mode.custom_mode = 'OFFBOARD'
        #         if(set_mode_client.call(offb_set_mode).mode_sent == True):
        #             rospy.loginfo("OFFBOARD enabled")
        #     elif offb_set_mode.custom_mode == 'OFFBOARD':
        #         vel_msg.twist.linear.z = 0
        #         vel_msg.twist.linear.x = 0.8
        # elif z_val > 6 and y_val < 5 and flag1 == False:
        #     flag2 == True
        #     if offb_set_mode.custom_mode == "AUTO.LOITER":
        #         offb_set_mode.custom_mode = 'OFFBOARD'
        #         if(set_mode_client.call(offb_set_mode).mode_sent == True):
        #             rospy.loginfo("OFFBOARD enabled")
        #     elif offb_set_mode.custom_mode == 'OFFBOARD':
        #         vel_msg.twist.linear.z = 0
        #         vel_msg.twist.linear.x = 0
        #         vel_msg.twist.linear.y = 0.8
        #         flag1 = True
        # elif z_val > 6 and x_val > 0:
        #     flag1 == True

        #     if offb_set_mode.custom_mode == "AUTO.LOITER":
        #         offb_set_mode.custom_mode = 'OFFBOARD'
        #         if(set_mode_client.call(offb_set_mode).mode_sent == True):
        #             rospy.loginfo("OFFBOARD enabled")
        #     elif offb_set_mode.custom_mode == 'OFFBOARD':
        #         vel_msg.twist.linear.z = 0
        #         vel_msg.twist.linear.x = -0.8
        #         vel_msg.twist.linear.y = 0
        # elif z_val > 6 and y_val > 0:
        #     flag2 == True

        #     if offb_set_mode.custom_mode == "AUTO.LOITER":
        #         offb_set_mode.custom_mode = 'OFFBOARD'
        #         if(set_mode_client.call(offb_set_mode).mode_sent == True):
        #             rospy.loginfo("OFFBOARD enabled")
        #     elif offb_set_mode.custom_mode == 'OFFBOARD':
        #         vel_msg.twist.linear.z = 0
        #         vel_msg.twist.linear.x = -0.8
        #         vel_msg.twist.linear.y = 0
            
        




                # while (current_state.mode == "OFFBOARD" and land == True):
                # if (land == True):
                #     offb_set_mode.custom_mode = "AUTO.LAND"
                #     if current_state.mode != "AUTO.LAND":
                #         if (set_mode_client.call(offb_set_mode).mode_sent == True):
                
                #             rospy.loginfo("AUTO.LAND mode enable")
        rate.sleep()