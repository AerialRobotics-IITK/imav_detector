#!/usr/bin/env python

import cv2
import numpy as np
import os, sys
import os.path as path
import roslib
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#frame = np.array([640,480])
#cap = cv2.VideoCapture(1)
#cv2.namedWindow('Image')
minHSV = [-1,-1,-1]
maxHSV = [-1,-1,-1]

class HSVPicker:

    def __init__(self):
        print("Press p to pause/play the image stream.")
        print("Press c to calculate the HSV ranges for stored regions.")
        print("Press q to quit.")
        self.sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback, queue_size=1)
        self.bridge = CvBridge()
        self.key = 0
        self.proc = 0
        self.boxes = []
        self.rois = []
        self.stallImage = False
        self.sigma = 2
        filepath = path.join(path.join(path.join(path.dirname(path.realpath(__file__)),os.pardir),'etc'),'hsv.bin')
        open(filepath, 'w+').close()

    def calcHSV(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        # std deviation non zero fix!
        hZ = abs((h-np.mean(h))/np.std(h))
        sZ = abs((s-np.mean(s))/np.std(s))
        vZ = abs((v-np.mean(v))/np.std(v))
        imZ = frame
        imZ[hZ>self.sigma] = [255,255,255]
        imZ[sZ>self.sigma] = [255,255,255]
        imZ[vZ>self.sigma] = [255,255,255]
        regionH = h[abs(hZ)<=self.sigma]
        regionS = s[abs(sZ)<=self.sigma]
        regionV = v[abs(vZ)<=self.sigma]
        return [np.min(regionH),np.min(regionS),np.min(regionV)], [np.max(regionH),np.max(regionS),np.max(regionV)]

    def on_mouse(self, event, x, y, flags, param):
        # handle bottom to top drag
        if event == cv2.EVENT_LBUTTONDOWN:
            print("Start: "+str(x)+", "+str(y))
            sbox = [x,y]
            self.boxes.append(sbox)
        if event == cv2.EVENT_LBUTTONUP:
            print("End: "+str(x)+", "+str(y))
            ebox = [x,y]
            self.boxes.append(ebox)
            if self.boxes[-1] == self.boxes[-2] :
                print("Click ignored.")
                return
            crop = self.frame[self.boxes[-2][1]:self.boxes[-1][1], self.boxes[-2][0]:self.boxes[-1][0]]
            cv2.imshow('ROI', crop)
            print("Press w to save, x to discard.")
            done = False
            while(not done):
                self.key = cv2.waitKey(1)
                if ord('w') == self.key:
                    self.rois.append(crop)
                    cv2.destroyWindow('ROI')
                    print("Saved")
                    done = True
                    self.key = 0
                if ord('x') == self.key:
                    cv2.destroyWindow('ROI')
                    print("Deleted")
                    done = True
                    self.key = 0

    def callback(self, data):
        try:
            if(not self.stallImage):
                self.frame = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.imshow('Image', self.frame)
        self.key = cv2.waitKey(1)
        cv2.setMouseCallback('Image', self.on_mouse, 0)
        if ord('q') == self.key:
            print("Quitting.")
            rospy.signal_shutdown("Quitting.")
        if ord('p') == self.key:
            if (self.stallImage):
                print("Resuming.")
            else:
                print("Pausing.")
            self.stallImage = not self.stallImage
        if ord('c') == self.key:
            if (self.proc == len(self.rois)):
                print("No new regions to process.")
                return
            print("Calculating HSV ranges for saved ROIs.")
            i = self.proc
            print("Press e to calculate.")
            print("Press r to stop calculating.")
            print("Press n to move to the next saved region.")
            print("Press s to save current region values to file.")
            while(i < len(self.rois)):
                cv2.imshow('Region', self.rois[i])
                self.key = cv2.waitKey(1)
                if self.key == ord('e'):
                    cv2.destroyWindow('Region')
                    minHSV, maxHSV = self.calcHSV(self.rois[i])
                    print("Minimum H,S,V values for this region: ")
                    print(minHSV)
                    print("Maximum H,S,V values for this region: ")
                    print(maxHSV)
                if self.key == ord('r'):
                    cv2.destroyWindow('Region')
                    print("Stopping after " + str(self.proc) + " regions.")
                    break
                if self.key == ord('n'):
                    cv2.destroyWindow('Region')
                    print("Loading next region")
                    i = i + 1
                    self.proc = i
                    continue
                if self.key == ord('s'):
                    cv2.destroyWindow('Region')
                    print("Saving to file.")
                    filepath = path.join(path.join(path.join(path.dirname(path.realpath(__file__)),os.pardir),'etc'),'hsv.bin')
                    data = [minHSV, maxHSV]
                    with open(filepath, 'ab') as f:
                        np.savetxt(f, data, "%d")
                        f.write("\n")
                        f.close()
                    print("Loading next region")
                    i = i + 1
                    self.proc = i
                    continue
            print("All regions done.")
            cv2.destroyWindow('Region')

def main(args):

    picker = HSVPicker()
    rospy.init_node('picker', anonymous=True)
    while not rospy.is_shutdown():
        rospy.spin()
    #cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)