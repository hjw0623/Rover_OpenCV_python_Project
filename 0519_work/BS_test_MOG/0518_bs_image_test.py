'''
Created on 2016. 5. 18.

@author: blanc
'''
import numpy as np
import os
import argparse
import cv2, time
from distutils.core import setup, Extension


if __name__ == "__main__":
    #IMAGE_PATHS = ["1.png", "2.png", "3.png", "4.png"]
    start = 146357636244
    end = 146357636506
    IMAGE_PATHS = []
    
  
    for iter in xrange(start, end):
        try:
            f_iter = 0.01 * iter
            filename = "%.2f" % f_iter 
            filename = "cropped_img//"+filename + ".jpg"
            if os.path.isfile(filename):
                #print filename
                IMAGE_PATHS.append(filename)
        except:
            pass
    
    fgbg = cv2.createBackgroundSubtractorMOG2()
    print len(IMAGE_PATHS)
    for iter in xrange(len(IMAGE_PATHS)):        
        frame = cv2.imread(IMAGE_PATHS[iter])        
        cv2.imshow('frame', frame)
        fgmask = fgbg.apply(frame)
        
        cv2.imshow('frame', fgmask)
        time.sleep(0.05)
        #print np.count_nonzero(fgmask)
        if np.count_nonzero(fgmask) >2500:
            print 'alarm'
        k = cv2.waitKey(30) & 0xff
        
        if k ==27:
            break
        
    #cap.release()
    cv2.destroyAllWindows()
