#!/usr/bin/env python3
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

### Load training images and labels


#display the first image
def crop_img(stack,num,plot=False):
    plt.figure()
    #ify ou want to pre-crop - 
    #img = stack[num,30:200,30:300,1]
    #if not
    img = stack[num,:,:,0]
    #cimg = stack[num,:,:,:]

    #find the color mask
    #mask = np.abs(cv2.inRange(cimg,np.array([105,100,110]),np.array([130,130,140])))
    
    #plt.figure()
    #plt.imshow(mask)
    #whiteidx = np.where(red.any(axis=1) > 100 and green.any(axis=1) < 120)
    #whiteidx = np.where(np.logical_and(red > 100,red < 120) and np.logical_and(green > 100, green < 120) and np.logical_and(blue > 100, blue < 120))  
    
    img = cv2.resize(img,(50,50),interpolation=cv2.INTER_CUBIC)    
    img_filt = cv2.GaussianBlur(img,(5,5),0)
    
    
    #different approach with adaptive thresholding    
    th3 = cv2.adaptiveThreshold(img_filt,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,11,2)    
    #th3 = abs(th3 - 255)    
    
    #conts,heirs = cv2.findContours(th3,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    #THIS WORKSconts,heirs = cv2.findContours(th3,5,5)
    conts,heirs = cv2.findContours(th3,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

    #just take the top contour??    
    #cnt = conts[0]
    #cycle into contours
    cnts = sorted(conts,key=cv2.contourArea,reverse=True)[:10]

    boxcnt = 0
    for c in cnts:
        peri = cv2.arcLength(c,True)
        approx = cv2.approxPolyDP(c,0.02*peri,True)
        
        if len(approx) == 4:
            print('Found box!')
            boxcnt = approx
            
    hull = cv2.convexHull(cnts[0])
    x,y,w,h = cv2.boundingRect(hull)
    crop = img[y:y+100,x:x+100]
    
    if plot:
        plt.figure()
        plt.subplot(211)
        plt.imshow(crop,cmap='gray',interpolation='bicubic')
        plt.subplot(212)
        plt.imshow(th3)
        plt.colorbar()
        #cv2.imshow('Test',fulltrain_rgb[0,:,:,cc])
            
        
        plt.show()
    return crop
        

root_dir = './robo_maze_imgs/'

with open(root_dir + 'train.txt', 'rb') as f:
    reader = csv.reader(f)
    lines = list(reader)

# this line reads in all images listed in the file in GRAYSCALE, and resizes them to 33x25 pixels
fulltrain_rgb = np.array( [np.array(cv2.imread(root_dir+lines[i][0]+".png")) for i in range(len(lines))])
fulltrain_g = np.array( [np.array(cv2.imread(root_dir+lines[i][0]+".png",0)) for i in range(len(lines))])

train_gray = []
#go through each image and crop
for ii in range(fulltrain_rgb.shape[0]):
    train_gray.append(crop_img(fulltrain_rgb,ii,plot=True))
    print(str(ii) + ': ' + str(train_gray[ii].shape))


#train_gray = np.array( [np.array(cv2.resize(cv2.imread(root_dir+lines[i][0]+".png",0)[335/3:335*2/3,252/3:252*2/3],(33,25))) for i in range(len(lines))])

train=train_gray

def extr_feat(img):
    imfilt = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    imLap = cv2.Laplacian(imfilt,cv2.CV_64F)
    #hist_vect,bins = np.histogram(imLap.flatten(),bins=50,range=(0,200))

    #for rgb find the mode
    med_col = np.array([0,0,0])
    
    for cc in range(3):
        med_col[cc] = np.array(np.mean(img[:,:,cc]))
    
    
    mostcol = np.argmax(med_col)
    
    #crop the image    
    
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
    
    
    f_vect = np.hstack((entr,med_col[0]/med_col[1],numcorns,which_side))
    return f_vect
    
#feature extraction should happen here
#train has the images
train_data = [None] * 47

feat_extr = 1
#take every image and make it a histogram
if feat_extr:
    for ii in range(train.shape[0]):        
        train_data[ii] = extr_feat(train[ii,:,:])

else:
    # here we reshape each image into a long vector and ensure the data type is a float (which is what KNN wants)
    train_data = train.flatten().reshape(len(lines), 33*25)
    train_data = train_data.astype(np.float32)


# read in training labels
train_labels = np.array([np.int32(lines[i][1]) for i in range(len(lines))])

train_data = np.array(train_data)

### Train classifier
#knn = cv2.KNearest()
#knn.train(train_data, cv2.ROW_SAMPLE, train_labels)

#do GMM instead
clf = mixture.GMM(n_components=6,covariance_type='full')
clf.fit(train_data)

label_map = defaultdict(dict)
#find the new labels
big_map = []
for ii in range(len(lines)):
    train_label = np.int32(lines[ii][1])
    ret = clf.predict(extr_feat(train[ii,:,:]).reshape(-1,1).T)
    
    #print('Tlab' + str(train_label))
    #print(ret)
    #need a new mapping for these
    big_map.append([ret[0],train_label])
    
big_map = np.array(big_map)

for ll in range(6):
    label_map[ll] = stats.mode(big_map[big_map[:,0] == ll][:,1])[0][0]
    
#label_map[ret[0]] = train_label

### Run test imagess
with open('./imgs/test.txt', 'rb') as f:
    reader = csv.reader(f)
    lines = list(reader)

correct = 0.0
confusion_matrix = np.zeros((6,6))

print('Doing testing now')

for ii in range(len(lines)):
    test_img = np.array(cv2.resize(cv2.imread("./imgs/"+lines[ii][0]+".png")[335/3:335*2/3,252/3:252*2/3],(33,25)))
    #test_img = test_img.flatten().reshape(1, 33*25)
    #test_img = test_img.astype(np.float32)

    test_label = np.int32(lines[ii][1])

    ret = clf.predict(extr_feat(test_img).reshape(-1,1).T)
    model_label = label_map[ret[0]]    
    
    if test_label == model_label:
        print(lines[ii][0], " Correct, ", model_label)
        correct += 1
        confusion_matrix[np.int32(model_label),np.int32(model_label)] += 1
    else:
        confusion_matrix[test_label,np.int32(ret)] += 1
        
        print(lines[ii][0], " Wrong, ", test_label, " classified as ", model_label)
        #print "\tneighbours: ", neighbours
        #print "\tdistances: ", dist



print("\n\nTotal accuracy: ", correct/len(lines))
print(confusion_matrix)
