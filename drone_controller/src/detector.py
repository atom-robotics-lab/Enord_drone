# ! /usr/bin/env python3

# Import necessary libraries
import cv2
import math
import imutils

# Intialize the parameters for aruco detection using OpenCV
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
arucoParams = cv2.aruco.DetectorParameters_create()

'''The detection class is a helper class for the aruco_detector_node, used to detect 
aruco markers from input camera feed using OpenCV'''

class detection():
    def __init__(self):
        '''This is the constructor function where the  variables and objects will get initialized'''

        self.center = None
        self.markerID1 = None
        self.radius1 = None

    def aruco_detection(self, image):
        '''This is the main function which receives the image and detects aruco markers'''
        self.image = image

        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Resize the image
        image = imutils.resize(image, width=1000)
        
        # Use the OpenCV to detect the aruco markers and retreive their co-ordinates and ids
        (corners, ids, rejected) = cv2.aruco.detectMarkers(image,
                                                           arucoDict, parameters=arucoParams)

        # Proceed further if aruco markers are detected in the input image       
        if len(corners) > 0:

            # Retreive the individual ids
            ids = ids.flatten()

            # Retreive the co-ordinates and centers of the individual markers 
            for (markerCorner, markerID) in zip(corners, ids):
                
                # Calculate the corners of the aruco markers
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                
                # Calculate the radius of the aruco marker
                radius = int(math.sqrt(
                    (int(topRight[0]) - int(bottomLeft[0])) ** 2 + (int(topRight[1]) - int(bottomLeft[1])) ** 2) / 2)
                
                # Calculate the center co-ordinates of the detected aruco markers
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                
                # Draw centers around the aruco markers
                cv2.circle(image, (cX, cY), radius, (0, 0, 255), 3)
                cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
              
                self.center = (cX, cY)
                self.markerID1 = markerID
                self.radius1 = radius

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            exit()

        # Return aruco detection results
        return [gray, self.center, self.radius1, self.markerID1, image]


