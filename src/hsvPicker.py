#!/usr/bin/env python

import cv2
import numpy as np

proc = 0
key = 0
boxes = []
rois = []

def hsvToRGB(h,s,v):
    r=g=b=0.0
    h = h/60.0
    s = s/255.0
    v = v/255.0
    c = v*s
    x = c*(1.0-abs(h%2-1.0))
    if(h>=0 and h<=1):
        r = c
        g = x
    elif(h>1 and h<=2):
        r = x
        g = c
    elif(h>2 and h<=3):
        g = c
        b = x
    elif(h>3 and h<=4):
        g = x
        b = c
    elif(h>4 and h<=5):
        r = x
        b = c
    elif(h>5 and h<=6):
        r = c
        b = x
    m = v - c
    r = (r + m)*255.0
    g = (g + m)*255.0
    b = (b + m)*255.0
    return int(r),int(g),int(b)

def rgbToHSV(r,g,b):
    h=s=v=0.0
    min_ = min(r, min(g,b))
    max_ = max(r, max(g,b))
    v = max_
    delta = max_ - min_
    if max_ != 0 :
        s = min(delta*255.0/max_, 255.0)
    if r==max_:
        h = 0.0 + (g-b)/delta
    elif  g==max_:
        h = 2.0 + (b-r)/delta
    else:
        h = 4.0 + (r-g)/delta
    h = h*60.0
    if h<0:
        h = h + 360.0
    return int(h),int(s),int(v)

def findRanges(rois, key):
    i = proc
    print("Press r to stop calculating.")
    print("Press n to move to the next saved region.")
    print("Press s to save region values to file.")
    while(i < (len(rois) - proc)):
        cv2.imshow('Region', rois[i])
        #calc and print here
        if key == ord('r'):
            print("Stopping after " + str(proc) + " images.")
            return
        if key == ord('n'):
            print("Loading next region")
            i = i + 1
            continue
        if key == ord('s'):
            # save here
            print("Saving to file.")
    print("All regions done.")
    return

def on_mouse(event, x, y, flags, params):
    if event == cv2.EVENT_LBUTTONDOWN:
        print("Start: "+str(x)+", "+str(y))
        sbox = [x,y]
        boxes.append(sbox)
    if event == cv2.EVENT_LBUTTONUP:
        print("End: "+str(x)+", "+str(y))
        ebox = [x,y]
        boxes.append(ebox)
        crop = frame[boxes[-2][1]:boxes[-1][1], boxes[-2][0]:boxes[-1][0]]
        cv2.imshow('ROI', crop)
        print("Press w to save, x to discard.")
        done = False
        while(not done):
            key = cv2.waitKey(1)
            if ord('w') == key:
                rois.append(crop)
                cv2.destroyWindow('ROI')
                print("Saved")
                done = True
                key = 0
            if ord('x') == key:
                cv2.destroyWindow('ROI')
                print("Deleted")
                done = True
                key = 0

cap = cv2.VideoCapture(1)

print("Press p to pause/play the image stream.")
print("Press c to calculate the HSV ranges for stored regions.")
print("Press q to quit.")

cv2.namedWindow('Image')
stallImage = False

while(True):

    if (not stallImage):
        ret, frame = cap.read()
    cv2.imshow('Image', frame)
    key = cv2.waitKey(1)
    cv2.setMouseCallback('Image', on_mouse, 0)
    if ord('q') == key:
        print("Quitting.")
        break
    if ord('p') == key:
        if (stallImage):
            print("Resuming.")
        else:
            print("Pausing.")
        stallImage = not stallImage
    if ord('c') == key:
        if (proc == len(rois)):
            print("No new regions to process.")
            break
        print("Calculating HSV ranges for saved ROIs.")
        i = proc
        print("Press r to stop calculating.")
        print("Press n to move to the next saved region.")
        print("Press s to save region values to file.")
        while(i < (len(rois) - proc)):
            cv2.imshow('Region', rois[i])
            key = cv2.waitKey(1)
            if key == ord('r'):
                print("Stopping after " + str(proc) + " images.")
                break
            if key == ord('n'):
                print("Loading next region")
                i = i + 1
                proc = i
                continue
            if key == ord('s'):
                print("Saving to file.")
        print("All regions done.")
        cv2.destroyWindow('Region')

cap.release()
cv2.destroyAllWindows()