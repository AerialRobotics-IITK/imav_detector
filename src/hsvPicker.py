#!/usr/bin/env python

import cv2
import numpy as np
import os
import os.path as path

proc = 0
key = 0
boxes = []
rois = []

def calcHSV(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    # std deviation non zero fix!
    hZ = abs((h-np.mean(h))/np.std(h))
    sZ = abs((s-np.mean(s))/np.std(s))
    vZ = abs((v-np.mean(v))/np.std(v))
    imZ = frame
    imZ[hZ>1] = [255,255,255]
    imZ[sZ>1] = [255,255,255]
    imZ[vZ>1] = [255,255,255]
    regionH = h[abs(hZ)<=1]
    regionS = s[abs(sZ)<=1]
    regionV = v[abs(vZ)<=1]
    return [np.min(regionH),np.min(regionS),np.min(regionV)], [np.max(regionH),np.max(regionS),np.max(regionV)]

def on_mouse(event, x, y, flags, params):
    # handle bottom to top drag
    if event == cv2.EVENT_LBUTTONDOWN:
        print("Start: "+str(x)+", "+str(y))
        sbox = [x,y]
        boxes.append(sbox)
    if event == cv2.EVENT_LBUTTONUP:
        print("End: "+str(x)+", "+str(y))
        ebox = [x,y]
        boxes.append(ebox)
        if boxes[-1] == boxes[-2] :
            print("Click ignored.")
            return
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
minHSV = [-1,-1,-1]
maxHSV = [-1,-1,-1]

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
        print("Press e to calculate.")
        print("Press r to stop calculating.")
        print("Press n to move to the next saved region.")
        print("Press s to save current region values to file.")
        while(i < len(rois)):
            cv2.imshow('Region', rois[i])
            key = cv2.waitKey(1)
            if key == ord('e'):
                cv2.destroyWindow('Region')
                minHSV, maxHSV = calcHSV(rois[i])
                print("Minimum H,S,V values for this region: ")
                print(minHSV)
                print("Maximum H,S,V values for this region: ")
                print(maxHSV)
            if key == ord('r'):
                cv2.destroyWindow('Region')
                print("Stopping after " + str(proc) + " regions.")
                break
            if key == ord('n'):
                cv2.destroyWindow('Region')
                print("Loading next region")
                i = i + 1
                proc = i
                continue
            if key == ord('s'):
                cv2.destroyWindow('Region')
                print("Saving to file.")
                filepath = path.join(path.join(os.getcwd(),'etc'),'hsv.bin')
                data = [minHSV, maxHSV]
                with open(filepath, 'ab') as f:
                    np.savetxt(f, data, "%d")
                    f.write("\n")
                    f.close()
                print("Loading next region")
                i = i + 1
                proc = i
                continue
        print("All regions done.")
        cv2.destroyWindow('Region')

cap.release()
cv2.destroyAllWindows()