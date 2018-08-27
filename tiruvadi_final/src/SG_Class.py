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
def crop_img(stack,num=-1,plot=False):
    #plt.figure()
    #ify ou want to pre-crop -
    frame = ((30,200),(30,300))
    
    if num == -1:
        #img = stack[30:200,30:300,1]
        img = stack[:,:,1]
    else:
        #img = stack[num,30:200,30:300,1]
        img = stack[num,:,:,1]
    
    #if not
    #img = stack[num,:,:,0]
    #cimg = stack[num,:,:,:]

    #find the color mask
    #mask = np.abs(cv2.inRange(cimg,np.array([105,100,110]),np.array([130,130,140])))
    
    #plt.figure()
    #plt.imshow(mask)
    #whiteidx = np.where(red.any(axis=1) > 100 and green.any(axis=1) < 120)
    #whiteidx = np.where(np.logical_and(red > 100,red < 120) and np.logical_and(green > 100, green < 120) and np.logical_and(blue > 100, blue < 120))  
    
    img = cv2.resize(img,(100,100),interpolation=cv2.INTER_CUBIC)    
    img_filt = cv2.GaussianBlur(img,(5,5),0)
    #img_filt = cv2.medianBlur(img,3)
    
    
    #different approach with adaptive thresholding    
    #9,3 below gives 86 percent performance!!
    th3 = cv2.adaptiveThreshold(img_filt,205,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,9,3)    
    #th3 = abs(th3 - 255)    
    
    #conts,heirs = cv2.findContours(th3,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    #THIS WORKSconts,heirs = cv2.findContours(th3,5,5)
    _,conts,heirs = cv2.findContours(th3,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    
    #conts,heirs = cv2.findContours(th3,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    

    #just take the top contour??    
    #cnt = conts[0]
    #cycle into contours
    cnts = sorted(conts,key=cv2.contourArea,reverse=True)[:10]

    boxcnt = 0
    for c in cnts:
        peri = cv2.arcLength(c,True)
        #0.02 below gives 86%
        approx = cv2.approxPolyDP(c,0.16*peri,True)
        
        if len(approx) == 4:
            boxcnt = approx
            
    if len(cnts) > 0:
        hull = cv2.convexHull(cnts[0])
    
        x,y,w,h = cv2.boundingRect(hull)
        crop = img[y:y+h,x:x+w]
    else:
        y = img.shape[1]/4
        x = img.shape[0]/4
        
        crop = img[y-5:y+20,x-5:x+20]       
        
    
    if plot:
        #plt.figure()
        #plt.subplot(311)
        #plt.imshow(img)
        #plt.subplot(312)
        #plt.imshow(crop,cmap='gray',interpolation='bicubic')
        #plt.subplot(313)
        #plt.imshow(th3)
        #plt.colorbar()
        cv2.imshow('Inp',img)
        #cv2.imshow('Cropped',crop)
        cv2.waitKey(1)
        #plt.show()
        
    #RETURN THE CROPPED PICTURE!!!
    #maybe even return a flag for the feature?
    #flag the ones that are likely nonsense
    #nonsense = False
    #if crop.shape != (25,20):
    #    nonsense = True
    sumred = np.sum(crop.flatten())
    
    #ratio for left vs right
    #print(crop.shape)
    
    ln = crop.shape[1]
    un = crop.shape[0]
    #lrratio = np.sum(crop[0:un/2,0:ln/2].flatten()).astype(np.float32) / (np.sum(crop[0:un,0:ln].flatten())).astype(np.float32) - 0.25
    lrratio = np.sum(crop[:,0:ln/2].flatten()).astype(np.float32) / np.sum(crop.flatten()).astype(np.float32)
    #print(lrratio)
    
    return crop, lrratio
        
def knn_train():
    
    print(cv2.__version__)

    root_dir = '/home/rosu/catkin_ws/src/tiruvadi_final/src/imgs/'
    
    with open(root_dir + 'train.txt', 'rb') as f:
        reader = csv.reader(f)
        lines = list(reader)
    
    # this line reads in all images listed in the file in GRAYSCALE, and resizes them to 33x25 pixels
    fulltrain_rgb = np.array( [np.array(cv2.imread(root_dir+lines[i][0]+".png")) for i in range(len(lines))])
    fulltrain_g = np.array( [np.array(cv2.imread(root_dir+lines[i][0]+".png",0)) for i in range(len(lines))])
    
    train_labels = np.array([np.int32(lines[i][1]) for i in range(len(lines))])
    
    train_gray = []
    train_labs = []
    train_files = []
    lrfeat = []
    #go through each image and crop
    for ii in range(fulltrain_rgb.shape[0]):
        croppedimg,lrratio = crop_img(fulltrain_rgb,ii,plot=False)
        croppedimg = cv2.resize(croppedimg,(25,20))
        
        train_gray.append(croppedimg)
        train_labs.append(train_labels[ii])
        train_files.append(ii)
        lrfeat.append(lrratio)
            
        #print(str(ii) + ': ')
    
    train_thumbs=train_gray
    
    assert len(train_gray) == len(train_labs)
    
    
    f = []
    for ii in range(len(train_thumbs)):
        f.append(extr_feat(train_thumbs[ii]))
        
    
    #Now we have a set of data, and we have the label space
    #Let's do a simple feature extraction
    train_X = np.array(train_thumbs) #34x3
    train_X = train_X.flatten().reshape(len(train_thumbs),25*20)
    
    #go in and add the lrratio feature for each
    augtrain_X = []
    for tt in range(len(train_thumbs)):    
        augtrain_X.append(np.hstack((train_X[tt,:],lrfeat[tt])))
    #if we want to use augmented design matrix
    train_X = np.array(augtrain_X)
    
    train_X = train_X.astype(np.float32)
    train_Y = np.array(train_labs)
    
    #let's try KNN
    knn = cv2.ml.KNearest_create()
    knn.train(train_X,cv2.ml.ROW_SAMPLE,train_Y)
    
    return knn
#%%

def knn_test(knn):
    ### Run test imagess
    root_dir = '/home/rosu/catkin_ws/src/tiruvadi_final/src/imgs/'
    with open(root_dir + 'test.txt', 'rb') as f:
        reader = csv.reader(f)
        lines = list(reader)
    
    correct = 0.0
    confusion_matrix = np.zeros((6,6))
    
    print('Doing testing now')
    
    #make our test set just like our normal set
    fulltest_rgb = np.array( [np.array(cv2.imread(root_dir+lines[i][0]+".png")) for i in range(len(lines))])
    test_gray = []
    test_labs = []
    test_files = []
    
    num_skipped = 0
    
    for ii in range(fulltest_rgb.shape[0]):
        croppedimg,lrratio = crop_img(fulltest_rgb,ii,plot=True)
        croppedimg = cv2.resize(croppedimg,(25,20)).flatten().reshape(1,25*20).astype(np.float32)
    
        
        augci = np.hstack((croppedimg,np.array([lrratio]).reshape(-1,1))).astype(np.float32)
        #test_gray.append(croppedimg)
            
        ret,ress,neig,dist = knn.findNearest(augci,3)
        #gmmret = clf.predict(croppedimg)
        #ret = gmmret    
        
        test_label = np.int32(lines[ii][1])    
        if test_label == ret:
            print(lines[ii][0], " Correct, ", ret)
            correct += 1
            confusion_matrix[np.int32(ret),np.int32(ret)] += 1
        else:
            confusion_matrix[test_label,np.int32(ret)] += 1
            
            print(lines[ii][0], " Wrong, ", test_label, " classified as ", ret)
           
            
    
    
    
    print("\n\nTotal accuracy: ", correct/len(lines))
    print(confusion_matrix)
    print("Num Skipped: " + str(num_skipped))        


if __name__ == '__main__':
    knnC = knn_train()
    knn_test(knnC)
    