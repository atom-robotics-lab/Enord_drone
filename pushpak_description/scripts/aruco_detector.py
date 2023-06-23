#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from detector import detection
from cv_bridge import CvBridge, CvBridgeError

import cv2
from sensor_msgs.msg import Image


class Robot_Controller:
    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/iris/camera/rgb/image_raw", Image, self.callback)
        self.image_pub = rospy.Publisher("image_topic_2", Image)
        self.aruco_detection = rospy.Publisher("/aruco_detection", Twist, queue_size=10)

        self.aruco_msg = Twist()

        self.id = None
       
        self.detect = detection()
        self.detect.T = 3

    def publish(self, x_error, y_error):
        self.aruco_msg.linear.x = x_error
        self.aruco_msg.linear.y = y_error
        self.aruco_detection.publish(self.aruco_msg)

    def callback(self, data):
        self.cv1_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.control_loop()


    def direction(self, markerID):

        if markerID == 1:
            return 0.3, "Turning Left ", "ID - 1"
        elif markerID == 2:
            return -0.3, "Turning Right", "ID - 2"
        else:
            return 0, "", "Parked" , "ID - 3"

    def control_loop(self):
        
        
        self.Result = self.detect.aruco_detection(self.cv1_image)
        aruco_center, aruco_radius = self.Result[1] , self.Result[2]

        frame_height = self.Result[0].shape[0]
        frame_width  = self.Result[0].shape[1]
        x_error = 0
        y_error = 0

        if aruco_center != None and aruco_radius != None:
            aruco_x = aruco_center[0]
            aruco_y = aruco_center[1]



            if (aruco_x > frame_height + 10):
                print("Go Right")
                x_error = 1
            elif(aruco_x < frame_height - 10):
                print("Go Left")
                x_error = -1
            else:
                x_error = 0
            
            if (aruco_y > 300):
                print("Go Down")
                y_error = -1
            elif (aruco_y < 280):
                print("Go Up")
                y_error = 1
            else:
                y_error = 0
            
            self.publish(x_error, y_error)
        else:
            x_error = 5
            y_error = 0
            self.publish(x_error, y_error)

        cv2.line(self.Result[4] , (frame_height,0),(frame_height,frame_width),(255,0,0),2)
        cv2.line(self.Result[4] , (0, 290),(1000, 290),(255,0,0),2)

            
        #cv2.putText(self.Result[4], self.lt, (300, 800), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 3, cv2.LINE_AA)
        #cv2.putText(self.Result[4], self.at, (350, 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 3, cv2.LINE_AA)

        cv2.imshow("Frame", self.Result[4])
        cv2.waitKey(1)


def main():
    rospy.init_node("aruco_detector_node", anonymous=True)
    of = Robot_Controller()
    try:
        rospy.spin()
    except:
        print("error")
    cv2.destroyAllWindows()

main()