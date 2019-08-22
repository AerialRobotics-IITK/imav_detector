#!/usr/bin/env python
from __future__ import print_function


#import the necessary packages
import cv2
import numpy as np
import sys
import string
import rospy
import roslib
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# #'optional' argument is required for trackbar creation parameters
# def nothing(x):
#     pass

# class HSVTuner:

#     def __init__(self):
        
#         self.bridge = CvBridge()
#         self.sub = rospy.Subscriber("colour_image", Image, self.callback)

#     def callback(self, data):
#         try:
#             self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
#         except CvBridgeError as e:
#             print(e)

#         cv2.namedWindow('Colorbars') # Create a window named 'Colorbars'

#         #assign strings for ease of coding
#         hh='Hue High'
#         hl='Hue Low'
#         sh='Saturation High'
#         sl='Saturation Low'
#         vh='Value High'
#         vl='Value Low'
#         wnd ='Colorbars'
#         #Begin Creating trackbars for each
#         cv2.createTrackbar(hl, wnd,0,179,nothing)
#         cv2.createTrackbar(hh, wnd,0,179,nothing)
#         cv2.createTrackbar(sl, wnd,0,255,nothing)
#         cv2.createTrackbar(sh, wnd,0,255,nothing)
#         cv2.createTrackbar(vl, wnd,0,255,nothing)
#         cv2.createTrackbar(vh, wnd,0,255,nothing)
        
#         #it is common to apply a blur to the frame
#         self.cv_image=cv2.GaussianBlur(self.cv_image,(5,5),0)
#         #convert from a BGR stream to an HSV stream
#         hsv=cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
#         #read trackbar positions for each trackbar
#         hul=cv2.getTrackbarPos(hl, wnd)
#         huh=cv2.getTrackbarPos(hh, wnd)
#         sal=cv2.getTrackbarPos(sl, wnd)
#         sah=cv2.getTrackbarPos(sh, wnd)
#         val=cv2.getTrackbarPos(vl, wnd)
#         vah=cv2.getTrackbarPos(vh, wnd)
#         #make array for final values
#         HSVLOW=np.array([hul,sal,val])
#         HSVHIGH=np.array([huh,sah,vah])
#         #create a mask for that range
#         mask = cv2.inRange(hsv,HSVLOW, HSVHIGH)
#         res = cv2.bitwise_and(self.cv_image,self.cv_image, mask =mask)

#         cv2.imshow('Colorbars', self.cv_image)
#         cv2.waitKey(3)
        

# def main(args):

#     tuner = HSVTuner()
#     rospy.init_node('tuner', anonymous=True)
   
#     try:
#         rospy.spin()
#     except KeyboardInterrupt:
#         print("Shutting down")
#     cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main(sys.argv)

###########################################################################

class ColorFilter:

    def __init__(self, camera=1):
        self.camera = camera
        self.bridge = CvBridge()


    def nothing(self,x):
        pass

    def color_filtering(self, camera_image):

        cv2.namedWindow('Trackbar window')
        height, width = camera_image.shape[:2]
        img = camera_image[0:height, 0:width]

        # create trackbars for color change
        cv2.createTrackbar('H_high', 'Trackbar window', 0, 179, self.nothing)
        cv2.createTrackbar('S_high', 'Trackbar window', 0, 179, self.nothing)
        cv2.createTrackbar('V_high', 'Trackbar window', 0, 255, self.nothing)
        cv2.createTrackbar('H_low', 'Trackbar window', 0, 255, self.nothing)
        cv2.createTrackbar('S_low', 'Trackbar window', 0, 255, self.nothing)
        cv2.createTrackbar('V_low', 'Trackbar window', 0, 255, self.nothing)


        cv2.imshow('Trackbar window', np.zeros((1, 512, 3), np.uint8))

        _f = cv2.GaussianBlur(img, (15, 15), 2)
        _f = cv2.cvtColor(_f, cv2.COLOR_BGR2HSV)  # To HSV

        h_low = cv2.getTrackbarPos('H_low', 'Trackbar window')
        s_low = cv2.getTrackbarPos('S_low', 'Trackbar window')
        v_low = cv2.getTrackbarPos('V_low', 'Trackbar window')
        h_high = cv2.getTrackbarPos('H_high', 'Trackbar window')
        s_high = cv2.getTrackbarPos('S_high', 'Trackbar window')
        v_high = cv2.getTrackbarPos('V_high', 'Trackbar window')

        # define range of color in HSV
        lower_bound = np.array([h_low, s_low, v_low])
        upper_bound = np.array([h_high, s_high, v_high])

        mask = cv2.inRange(_f, lower_bound, upper_bound)
        img = cv2.bitwise_and(img, img, mask=mask)  # Comment this line if you won't show the frame later

        # Comment the one you won't need
        cv2.imshow('Trackbar window', img)
        # cv2.imshow('mask',mask)
        cv2.waitKey(15)

    def callback(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.camera == 1:
            self.color_filtering(cv_image)

    def __del__(self):
        pass


cf = None
cf = ColorFilter()


def callback(data):
    cf.callback(data)


def main(args):
    rospy.init_node('tuner', anonymous=True)
    rospy.Subscriber("/usb_cam/image_raw", Image, callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)