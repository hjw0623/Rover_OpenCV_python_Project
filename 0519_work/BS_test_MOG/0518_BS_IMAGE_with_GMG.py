'''
Created on 2016. 5. 18.

@author: blanc
'''
import numpy as np
import cv2
import os
import time
if __name__ == "__main__":
    #IMAGE_PATHS = ["1.png", "2.png", "3.png", "4.png"]
    start = 820
    end = 826
    IMAGE_PATHS = []
    for iter in xrange(start, end):
        try:            
            filename = iter
            filename = "%d" %iter
            filename = filename + ".png"
            if os.path.isfile(filename):
                #print filename
                IMAGE_PATHS.append(filename)
        except:
            pass
    print "img paths: ", IMAGE_PATHS
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
   # fgbg = cv2.createBackgroundSubtractorGMG()
    print len(IMAGE_PATHS)
    for iter in xrange(len(IMAGE_PATHS)):        
        frame = cv2.imread(IMAGE_PATHS[iter])        
        cv2.imshow('frame', frame)
        fgmask = fgbg.apply(frame)
        
        cv2.imshow('frame', fgmask)
        time.sleep(0.05)
        k = cv2.waitKey(30) & 0xff
        
        if k ==27:
            break
        
    #cap.release()
    cv2.destroyAllWindows()
