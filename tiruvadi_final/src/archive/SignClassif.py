#!/usr/bin/env python
#Vineet Tiruvadi
#adapted from class-wide example_knn.py provided for Final Project

import cv2

import sys
import csv
import numpy as np

import pdb

import sklearn
from sklearn import mixture

import skimage
import skimage.filter
from skimage.filter.rank import entropy
from skimage.morphology import disk

from scipy import stats
import matplotlib.pyplot as plt

from collections import defaultdict

print(cv2.__version__)

def extr_feat(img):
    #imfilt = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    imfilt = img
    imLap = cv2.Laplacian(img,cv2.CV_64F)
    #hist_vect,bins = np.histogram(imLap.flatten(),bins=50,range=(0,200))


        
    #entropy
    entr = np.sum(entropy(imfilt,disk(5))[:])

    #SIFTing
    #sift = cv2.xfeatures2d.SIFT_create()
    #kp, des = sift.detectAndCompute(imfilt,None)    

    #corners
    corns = cv2.cornerHarris(imfilt,2,3,0.04)
    numcorns = np.sum((corns>0.1 * corns.max()).astype(int))
    
    #more edge pixels on left or right
    im_siz = imLap.shape
    left_weight = np.sum(imLap[:,0:im_siz[1]/2])
    right_weight = np.sum(imLap[:,im_siz[1]/2:])
    
    which_side = left_weight/(left_weight + right_weight)
    
    
    f_vect = np.hstack((entr,numcorns,which_side))
    return f_vect

#display the first image
def crop_img(stack,num,plot=False):
    #plt.figure()
    #ify ou want to pre-crop - 
    img = stack[num,30:200,30:300,1]
    #if not
    #img = stack[num,:,:,0]
    #cimg = stack[num,:,:,:]

    #find the color mask
    #mask = np.abs(cv2.inRange(cimg,np.array([105,100,110]),np.array([130,130,140])))
    
    #plt.figure()
    #plt.imshow(mask)
    #whiteidx = np.where(red.any(axis=1) > 100 and green.any(axis=1) < 120)
    #whiteidx = np.where(np.logical_and(red > 100,red < 120) and np.logical_and(green > 100, green < 120) and np.logical_and(blue > 100, blue < 120))  
    
    img = cv2.resize(img,(60,60),interpolation=cv2.INTER_CUBIC)    
    #img_filt = cv2.GaussianBlur(img,(3,3),0)
    img_filt = cv2.medianBlur(img,5)
    
    
    #different approach with adaptive thresholding    
    th3 = cv2.adaptiveThreshold(img_filt,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,11,3)    
    #th3 = abs(th3 - 255)    
    
    #conts,heirs = cv2.findContours(th3,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    #THIS WORKSconts,heirs = cv2.findContours(th3,5,5)
    _,conts,heirs = cv2.findContours(th3,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

    #just take the top contour??    
    #cnt = conts[0]
    #cycle into contours
    cnts = sorted(conts,key=cv2.contourArea,reverse=True)[:10]

    boxcnt = 0
    for c in cnts:
        peri = cv2.arcLength(c,True)
        approx = cv2.approxPolyDP(c,0.02*peri,True)
        
        if len(approx) == 4:
            boxcnt = approx
            
    if len(cnts) > 0:
        hull = cv2.convexHull(cnts[0])
    
        x,y,w,h = cv2.boundingRect(hull)
        crop = img[y:y+h,x:x+w]
    else:
#        edges = cv2.Canny(img_filt,100,200)
#        ct2,heirs2 = cv2.findContours(edges,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
#        cnts2 = sorted(ct2,key=cv2.contourArea,reverse=True)[:10]
#        hull2 = cv2.convexHull(cnts2[0])
#        for c in cnts2:
#            peri = cv2.arcLength(c,True)
#            approx = cv2.approxPolyDP(c,0.02*peri,True)
#            if len(approx) == 4:
#                boxcnt2 = approx
#                hull2 = cv2.convexHull(boxcnt2)
#                
#        x,y,w,h = cv2.boundingRect(hull2)
        y = img.shape[1]/4
        x = img.shape[0]/4
        
        crop = img[y:y+25,x+20]            
        
    
    if plot:
        plt.figure()
        plt.subplot(311)
        plt.imshow(img)
        plt.subplot(312)
        plt.imshow(crop,cmap='gray',interpolation='bicubic')
        plt.subplot(313)
        plt.imshow(th3)
        plt.colorbar()
        #cv2.imshow('Test',fulltrain_rgb[0,:,:,cc])
            
        
        plt.show()
        
    #RETURN THE CROPPED PICTURE!!!
    #maybe even return a flag for the feature?
    #flag the ones that are likely nonsense
    #nonsense = False
    #if crop.shape != (25,20):
    #    nonsense = True
    sumred = np.sum(crop.flatten())
    return crop, sumred
        
def trainKNN():
    root_dir = '/home/rosu/catkin_ws/src/tiruvadi_final/src/imgs/'
    
    with open(root_dir + 'train.txt', 'rb') as f:
        reader = csv.reader(f)
        lines = list(reader)
    
    # this line reads in all images listed in the file in GRAYSCALE, and resizes them to 33x25 pixels
    fulltrain_rgb = np.array( [np.array(cv2.imread(root_dir+lines[i][0]+".png")) for i in range(len(lines))])
    
    train_labels = np.array([np.int32(lines[i][1]) for i in range(len(lines))])
    
    train_gray = []
    train_labs = []
    train_files = []
    #go through each image and crop
    for ii in range(fulltrain_rgb.shape[0]):
        croppedimg,redsum = crop_img(fulltrain_rgb,ii,plot=False)
        croppedimg = cv2.resize(croppedimg,(25,20))
        
        train_gray.append(croppedimg)
        train_labs.append(train_labels[ii])
        train_files.append(ii)
            
        print(str(ii) + ': ')
    
    train_thumbs=train_gray
    
       
    
    f = []
    for ii in range(len(train_thumbs)):
        f.append(extr_feat(train_thumbs[ii]))
        
    
    #Now we have a set of data, and we have the label space
    #Let's do a simple feature extraction
    train_X = np.array(train_thumbs) #34x3
    train_X = train_X.flatten().reshape(len(train_thumbs),25*20)
    train_X = train_X.astype(np.float32)
    train_Y = np.array(train_labs)
    
    #let's try KNN
    knn = cv2.ml.KNearest_create()
    knn.train(train_X,cv2.ml.ROW_SAMPLE,train_Y)
    
    return knn