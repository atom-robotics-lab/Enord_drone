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
        self.image_sub = rospy.Subscriber("/iris0/camera/rgb/image_raw", Image, self.callback0)
        self.image_sub = rospy.Subscriber("/iris1/camera/rgb/image_raw", Image, self.callback1)
        self.image_sub = rospy.Subscriber("/iris2/camera/rgb/image_raw", Image, self.callback2)

        self.image_pub = rospy.Publisher("image_topic_2", Image)
        self.aruco_detection0 = rospy.Publisher("/aruco_detection0", Twist, queue_size=10)
        self.aruco_detection1 = rospy.Publisher("/aruco_detection1", Twist, queue_size=10)
        self.aruco_detection2 = rospy.Publisher("/aruco_detection2", Twist, queue_size=10)



        self.aruco_msg0 = Twist()
        self.aruco_msg1 = Twist()
        self.aruco_msg2 = Twist()

        self.id = None
       
        self.detect = detection()
        self.detect.T = 3

    def publish0(self, x_error0, y_error0):
        self.aruco_msg0.linear.x = x_error0
        self.aruco_msg0.linear.y = y_error0
        self.aruco_detection0.publish(self.aruco_msg0)

    def publish1(self, x_error0, y_error0):
        self.aruco_msg1.linear.x = x_error0
        self.aruco_msg1.linear.y = y_error0
        self.aruco_detection1.publish(self.aruco_msg1)

    def publish2(self, x_error0, y_error0):
        self.aruco_msg2.linear.x = x_error0
        self.aruco_msg2.linear.y = y_error0
        self.aruco_detection2.publish(self.aruco_msg2)

    def callback0(self, data):
        self.cv1_image0 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.control_loop()
    def callback1(self, data):
        self.cv1_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.control_loop()
    def callback2(self, data):
        self.cv1_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.control_loop()

    def direction(self, markerID):

        if markerID == 1:
            return 0.3, "Turning Left ", "ID - 1"
        elif markerID == 2:
            return -0.3, "Turning Right", "ID - 2"
        else:
            return 0, "", "Parked" , "ID - 3"

    def control_loop(self):
        
        
        self.Result0 = self.detect.aruco_detection(self.cv1_image0)
        aruco_center0, aruco_radius0 = self.Result0[1] , self.Result0[2]
        self.Result1 = self.detect.aruco_detection(self.cv1_image1)
        aruco_center1, aruco_radius1 = self.Result1[1] , self.Result1[2]
        self.Result2 = self.detect.aruco_detection(self.cv1_image2)
        aruco_center2, aruco_radius2 = self.Result2[1] , self.Result2[2]

        frame_height0 = self.Result0[0].shape[0]
        frame_width0  = self.Result0[0].shape[1]
        frame_height1 = self.Result1[0].shape[0]
        frame_width1  = self.Result1[0].shape[1]
        frame_height2 = self.Result2[0].shape[0]
        frame_width2  = self.Result2[0].shape[1]
        x_error0 = 0
        y_error0 = 0
        x_error1 = 0
        y_error1 = 0
        x_error2 = 0
        y_error2 = 0

        if aruco_center0 != None and aruco_radius0 != None:
            aruco_x0 = aruco_center0[0]
            aruco_y0 = aruco_center0[1]



            if (aruco_x0 > frame_height0 + 10):
                print("Go Right")
                x_error0 = 1
            elif(aruco_x0 < frame_height0 - 10):
                print("Go Left")
                x_error0 = -1
            else:
                x_error0 = 0
            
            if (aruco_y0 > 300):
                print("Go Down")
                y_error = -1
            elif (aruco_y0 < 280):
                print("Go Up")
                y_error0 = 1
            else:
                y_error0 = 0
            
            self.publish0(x_error0, y_error0)
        else:
            x_error = 5
            y_error = 0
            self.publish0(x_error0, y_error0)
        if aruco_center1 != None and aruco_radius1 != None:
            aruco_x1 = aruco_center1[0]
            aruco_y1 = aruco_center1[1]



            if (aruco_x1 > frame_height1 + 10):
                print("Go Right")
                x_error = 1
            elif(aruco_x1 < frame_height1 - 10):
                print("Go Left")
                x_error1 = -1
            else:
                x_error1 = 0
            
            if (aruco_y1 > 300):
                print("Go Down")
                y_error1 = -1
            elif (aruco_y1 < 280):
                print("Go Up")
                y_error1 = 1
            else:
                y_error1 = 0
            
            self.publish1(x_error1, y_error1)
        else:
            x_error1 = 5
            y_error1 = 0
            self.publish1(x_error1, y_error1)

        if aruco_center2 != None and aruco_radius2 != None:
            aruco_x2 = aruco_center2[0]
            aruco_y2 = aruco_center2[1]



            if (aruco_x2 > frame_height2 + 10):
                print("Go Right")
                x_error = 1
            elif(aruco_x2 < frame_height2 - 10):
                print("Go Left")
                x_error = -1
            else:
                x_error = 0
            
            if (aruco_y2 > 300):
                print("Go Down")
                y_error = -1
            elif (aruco_y2 < 280):
                print("Go Up")
                y_error2 = 1
            else:
                y_error2 = 0
            
            self.publish2(x_error2, y_error2)
        else:
            x_error2 = 5
            y_error2 = 0
            self.publish2(x_error2, y_error2)

        cv2.line(self.Result0[4] , (frame_height0,0),(frame_height0,frame_width0),(255,0,0),2)
        cv2.line(self.Result0[4] , (0, 290),(1000, 290),(255,0,0),2)
        cv2.line(self.Result1[4] , (frame_height1,0),(frame_height1,frame_width1),(255,0,0),2)
        cv2.line(self.Result1[4] , (0, 290),(1000, 290),(255,0,0),2)
        cv2.line(self.Result2[4] , (frame_height2,0),(frame_height2,frame_width2),(255,0,0),2)
        cv2.line(self.Result2[4] , (0, 290),(1000, 290),(255,0,0),2)           
        #cv2.putText(self.Result[4], self.lt, (300, 800), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 3, cv2.LINE_AA)
        #cv2.putText(self.Result[4], self.at, (350, 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 3, cv2.LINE_AA)

        cv2.imshow("Frame0", self.Result0[4])
        cv2.imshow("Frame1", self.Result1[4])
        cv2.imshow("Frame2", self.Result2[4])
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