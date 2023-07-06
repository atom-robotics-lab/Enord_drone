#! /usr/bin/env python3

# Import necessary libraries and packages
import rospy
from geometry_msgs.msg import Twist
from detector import detection
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image


'''The Robot_Controller class will receive the camera feed from the drones, detect the aruco markers and publish
the position errors of the aruco markers in a topic'''

class Robot_Controller:
    def __init__(self):
        ''' This is the constructor function where variable and object initialization will take place'''
        
        # Initializing CvBridge() for converting sensor_msgs/msg/Image data to numpy matrices
        self.bridge = CvBridge()

        # Initializing subscribers to obtain camera feed from the drones
        self.image_sub0 = rospy.Subscriber("/iris0/camera/rgb/image_raw", Image, callback=self.callback0)
        self.image_sub1 = rospy.Subscriber("/iris1/camera/rgb/image_raw", Image, callback=self.callback1)
        self.image_sub2 = rospy.Subscriber("/iris2/camera/rgb/image_raw", Image, callback=self.callback2)

        # Initializing publishers to publish position errors
        self.aruco_detection0 = rospy.Publisher("/aruco_detection0", Twist, queue_size=10)
        self.aruco_detection1 = rospy.Publisher("/aruco_detection1", Twist, queue_size=10)
        self.aruco_detection2 = rospy.Publisher("/aruco_detection2", Twist, queue_size=10)

        # Initializing empty Twist() messages
        self.aruco_msg0 = Twist()
        self.aruco_msg1 = Twist()
        self.aruco_msg2 = Twist()

        self.id = None

        # Initialize the aruco detector
        self.detect = detection()

    def publish0(self, x_error0, y_error0):
        ''' This function is used to publish position error for iris0'''

        # Initialize x and y errors
        self.aruco_msg0.linear.x = x_error0
        self.aruco_msg0.linear.y = y_error0

        # Publish data 
        self.aruco_detection0.publish(self.aruco_msg0)

    def publish1(self, x_error1, y_error1):
        ''' This function is used to publish position error for iris1'''

        # Initialize x and y errors
        self.aruco_msg1.linear.x = x_error1
        self.aruco_msg1.linear.y = y_error1

        # Publish data        
        self.aruco_detection1.publish(self.aruco_msg1)

    def publish2(self, x_error2, y_error2):

        ''' This function is used to publish position error for iris1'''

        # Initialize x and y errors
        self.aruco_msg2.linear.x = x_error2
        self.aruco_msg2.linear.y = y_error2

        # Publish data      
        self.aruco_detection2.publish(self.aruco_msg2)

    def callback0(self, data):
        '''Callback function for camera feed from iris1'''

        # Convert sensor_msgs/msg/Image datatype to numpy matrix
        self.cv1_image0 = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def callback1(self, data):
        '''Callback function for camera feed from iris1'''

        # Convert sensor_msgs/msg/Image datatype to numpy matrix
        self.cv1_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.control_loop()

    
    def callback2(self, data):
        '''Callback function for camera feed from iris1'''

        # Convert sensor_msgs/msg/Image datatype to numpy matrix
        self.cv1_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # Execute the control_loop()


    def control_loop(self):
        '''This is the main control loop for the detection of aruco markers
        and calculation of position errors of the aruco markers w.r.t the drones
        '''

        # Get the aruco detection results for iris0
        self.Result0 = self.detect.aruco_detection(self.cv1_image0)
        aruco_center0, aruco_radius0 = self.Result0[1] , self.Result0[2]

        # Get the aruco detection results for iris1
        self.Result1 = self.detect.aruco_detection(self.cv1_image1)
        aruco_center1, aruco_radius1 = self.Result1[1] , self.Result1[2]

        # Get the aruco detection results for iris2
        self.Result2 = self.detect.aruco_detection(self.cv1_image2)
        aruco_center2, aruco_radius2 = self.Result2[1] , self.Result2[2]

        # Store the height and width of camera frames
        frame_height0 = self.Result0[0].shape[0]
        frame_width0  = self.Result0[0].shape[1]
        frame_height1 = self.Result1[0].shape[0]
        frame_width1  = self.Result1[0].shape[1]
        frame_height2 = self.Result2[0].shape[0]
        frame_width2  = self.Result2[0].shape[1]

        # Initialize x and y errors as zero
        x_error0 = 0
        y_error0 = 0
        x_error1 = 0
        y_error1 = 0
        x_error2 = 0
        y_error2 = 0

        # Publish position error for iris0 using values from aruco detector
        if aruco_center0 != None and aruco_radius0 != None:
            
            # Centers of the detected arco markers
            aruco_x0 = aruco_center0[0]
            aruco_y0 = aruco_center0[1]

            # x_error is 1 when aruco is to the right of the vertical center of
            # frame, -1 when aruco is to the left and zero otherwise
            if (aruco_x0 > frame_height0 + 10):
                print("Go Right")
                x_error0 = 1
            elif(aruco_x0 < frame_height0 - 10):
                print("Go Left")
                x_error0 = -1
            else:
                x_error0 = 0
            
            # y_error is 1 when aruco is to the top of the horizontal center of
            # frame, -1 when aruco is below and zero otherwise
            if (aruco_y0 > 300):
                print("Go Down")
                y_error0 = -1
            elif (aruco_y0 < 280):
                print("Go Up")
                y_error0 = 1
            else:
                y_error0 = 0
            
            self.publish0(x_error0, y_error0)
        else:

            # Publish x_error=5 if aruco is not visible
            x_error0 = 5
            y_error0 = 0
            self.publish0(x_error0, y_error0)

        # Publish position error for iris1 using values from aruco detector
        if aruco_center1 != None and aruco_radius1 != None:
            aruco_x1 = aruco_center1[0]
            aruco_y1 = aruco_center1[1]

            # x_error is 1 when aruco is to the right of the vertical center of
            # frame, -1 when aruco is to the left and zero otherwise
            if (aruco_x1 > frame_height1 + 10):
                print("Go Right")
                x_error1 = 1
            elif(aruco_x1 < frame_height1 - 10):
                print("Go Left")
                x_error1 = -1
            else:
                x_error1 = 0
            
            # y_error is 1 when aruco is to the top of the horizontal center of
            # frame, -1 when aruco is below and zero otherwise
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

            # Publish x_error=5 if aruco is not visible
            x_error1 = 5
            y_error1 = 0
            self.publish1(x_error1, y_error1)

        # Publish position error for iris2 using values from aruco detector
        if aruco_center2 != None and aruco_radius2 != None:
            aruco_x2 = aruco_center2[0]
            aruco_y2 = aruco_center2[1]

            # x_error is 1 when aruco is to the right of the vertical center of
            # frame, -1 when aruco is to the left and zero otherwise
            if (aruco_x2 > frame_height2 + 10):
                print("Go Right")
                x_error2 = 1
            elif(aruco_x2 < frame_height2 - 10):
                print("Go Left")
                x_error2 = -1
            else:
                x_error2 = 0
            
            # y_error is 1 when aruco is to the top of the horizontal center of
            # frame, -1 when aruco is below and zero otherwise
            if (aruco_y2 > 300):
                print("Go Down")
                y_error2 = -1
            elif (aruco_y2 < 280):
                print("Go Up")
                y_error2 = 1
            else:
                y_error2 = 0
            
            self.publish2(x_error2, y_error2)
        else:

            # Publish x_error=5 if aruco is not visible
            x_error2 = 5
            y_error2 = 0
            self.publish2(x_error2, y_error2)

        # Draw horizontal and vertical center lines in the frame
        cv2.line(self.Result0[4] , (frame_height0,0),(frame_height0,frame_width0),(255,0,0),2)
        cv2.line(self.Result0[4] , (0, 290),(1000, 290),(255,0,0),2)
        cv2.line(self.Result1[4] , (frame_height1,0),(frame_height1,frame_width1),(255,0,0),2)
        cv2.line(self.Result1[4] , (0, 290),(1000, 290),(255,0,0),2)
        cv2.line(self.Result2[4] , (frame_height2,0),(frame_height2,frame_width2),(255,0,0),2)
        cv2.line(self.Result2[4] , (0, 290),(1000, 290),(255,0,0),2)           

        # Display the camera frames
        cv2.imshow("Frame0", self.Result0[4])
        cv2.imshow("Frame1", self.Result1[4])
        cv2.imshow("Frame2", self.Result2[4])
        cv2.waitKey(1)
        


def main():

    # Initialize the ROS node
    rospy.init_node("aruco_detector_node", anonymous=True)
    
    # Initialize the Robot_Controller class
    of = Robot_Controller()
    try:
        rospy.spin()
    except:
        print("error")


# Start the ROS node
main()