#!/usr/bin/env python

from __future__ import division
import yaml
import os
import os.path as path
import numpy as np
from scipy.stats import multivariate_normal as mn

def hsvToRGB(h,s,v):
    r=g=b=0
    h = h*6/60
    s = s*4/255
    v = v*4/255
    c = v*s
    x = c*(1-abs(h%2-1))
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
    r = (r + m)*255
    g = (g + m)*255
    b = (b + m)*255
    return r,g,b

def rgbToHSV(r,g,b):
    h=s=v=0
    min_ = min(r, min(g,b))
    max_ = max(r, max(g,b))
    v = max_
    delta = max_ - min_
    if max_ != 0 :
        s = min(delta*255/max_, 255)
    if r==max_:
        h = (g-b)/delta
    elif  g==max_:
        h = 2 + (b-r)/delta
    else:
        h = 4 + (r-g)/delta
    h = h*60
    if h<0:
        h = h + 360
    return h/6,s/4,v/4

def convGauss(grid):
    rgbGrid = np.zeros((64,64,64,2))
    for i in range(grid):
        for j in range(grid[0]):
            for k in range(grid[0][0]):
                r,g,b = hsvToRGB(i,j,k)
                rgbGrid[r//4][g//4][b//4] = grid[i][j][k]

def makeGaussian(color):
    mean = np.array(rgbToHSV(data[color]["mean"][0],data[color]["mean"][1],data[color]["mean"][2]))
    # np.savetxt(savepath, mean.flat)
    var = np.array(data[color]["variance"])
    # np.savetxt(savepath, var.flat)
    thresh = data[color]["threshold"]
    print(thresh)
    covar = np.diag(var**2)
    # np.savetxt(savepath, covar.flat)
    h, s, v = np.mgrid[-1.0:1.0:60j, -1.0:1.0:64j, -1.0:1.0:64j]
    hsv = np.column_stack([h.flat, s.flat, v.flat])
    # np.savetxt(savepath, hsv.flat)
    gauss = mn.pdf(hsv, mean=mean, cov=covar)
    np.savetxt(savepath, gauss.flat)
    gauss = gauss.reshape(h.shape)
    # np.savetxt(savepath, gauss.flat)
    gauss[gauss<thresh] = 0
    # np.savetxt(savepath, gauss.flat)
    grid = np.zeros((60,64,64,2))
    grid[:,:,:,0] = gauss
    if color == "red":
        grid[grid[:,:,:,0]!=0,1] = 1
    elif color == "yellow":
        grid[grid[:,:,:,0]!=0,1] = 2
    else:
        grid[grid[:,:,:,0]!=0,1] = 3
    return grid

def addGaussian(grid1, grid2):
    for i in range(len(grid1)):
        for j in range(len(grid1[0])):
            for k in range(len(grid1[0][0])):
                if grid1[i][j][k][1] != 0:
                    if grid1[i][j][k][0] < grid2[i][j][k][0]:
                        grid1[i][j][k] = grid2[i][j][k]
                else:
                    grid1[i][j][k] = grid2[i][j][k]

filepath = path.join(path.join(os.getcwd(), 'cfg'), 'gaussian.yaml')
with open(filepath, 'r') as file:
    data = yaml.safe_load(file)

savepath = path.join(path.join(os.getcwd(),'etc'),'color.txt')
print("making red")
redGauss = makeGaussian("red")
# np.savetxt(savepath, redGauss.reshape(64*64*64,2))
print("saved")
exit()
print("making yellow")
yellowGauss = makeGaussian("yellow")
with open(savepath, 'a+') as file:
    file.write(yellowGauss)
print("saved")

print("making blue")
blueGauss = makeGaussian("blue")
with open(savepath, 'a+') as file:
    file.write(blueGauss)
print("saved")

print("adding yellow")
twoGauss = addGaussian(redGauss, yellowGauss)
with open(savepath, 'a+') as file:
    file.write(twoGauss)
print("saved")

print("adding blue")
fullGauss = addGaussian(twoGauss, blueGauss)
with open(savepath, 'a+') as file:
    file.write(fullGauss)
print("saved")

print("converting")
colorGrid = convGauss(fullGauss)
print("saving")
with open(savepath, 'a+') as file:
    file.write(colorGrid)
print("saved")
