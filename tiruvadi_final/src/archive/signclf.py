# -*- coding: utf-8 -*-
"""
Created on Sun Dec  3 18:49:50 2017

@author: rosu
"""

def cam_callback(img):
    #First, we should 

    #now classify it
    pass

def import_signs_KNN():
    
    
    return knn

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
    crop = cv2.reshape(crop,(25,20))
    return crop, sumred
