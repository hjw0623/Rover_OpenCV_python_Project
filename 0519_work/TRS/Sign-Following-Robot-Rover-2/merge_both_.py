#!/usr/bin/env python

"""
Sign Following Rover 2.0
This module defines RoverSignFollower class based on Rover 2.0

"""
import time
from rover import Rover20
import time
import sys
import signal
import cv2
import numpy as np
from PIL import Image
from pytesseract import *
import math
from turtle import Turtle
import os
import traceback

import imutils
from matplotlib import pyplot as plt
import operator

# Avoid button bounce by enforcing lag between button events
MIN_BUTTON_LAG_SEC = 0.5

# Avoid close-to-zero values on axis
MIN_AXIS_ABSVAL    = 0.01
ANGULAR_VELOCITY = 33.5 * 360/ 60  # 33.5 for full battery (>75 %); 22.5 for 60 % battery; 25.0 for less than 60 %.
VELOCITY = 1 #(meter/sec)
TURN_VALUE = 0.25 # proportional value of error (PID Control)


def _signal_handler(signal, frame):
    frame.f_locals['rover'].close()
    sys.exit(0)


class RoverSignFollower(Rover20):
    def __init__(self):
        """Sign follower initiation."""

        # Set up basics for Odometer
        self.state = (0,0,0) #x,y,theta
        self.path = [] #list will contain points of previous positions)
        Rover20.__init__(self)
        self.wname = 'Rover 2.0: Hit ESC to quit'
        self.quit = False
        self.fileIndex = 0
        self.fname = 'filename'


        # Defaults on start-up: lights off, ordinary camera
        self.lightsAreOn = False
        self.stealthIsOn = False

        # Initiate My Turtle graphics to be used as a map. 

        self.pcmfile = open('rover20.pcm', 'w')
        self.myturtle = Turtle()
        self.myturtle.speed(0)
        self.myturtle.pencolor("blue")
        self.nocommand_counter = 0
    
    def turn(self, angle):
        
        timer = angle / ANGULAR_VELOCITY
        time_0 = time.time()
        while time.time() - time_0 < abs(timer):
            if angle > 0:
                self.setTreads(-1,+1)
            else:
                self.setTreads(+1,-1)
        self.setTreads(0,0)
        
        #update odometer
        self.updateState(0,0,angle)
        self.myturtle.left(angle)
        
    def moveForward(self,t, speed = 1):
        time_0 = time.time()
        while time.time() - time_0 < t:
            self.setTreads(speed,speed)
        self.setTreads(0,0)
      
        #update state for odometry
        distance = VELOCITY * t * speed
        x , y, theta = self.state
        x_change = distance * math.cos(math.radians(theta))
        y_change = distance * math.sin(math.radians(theta))
        
        self.updateState(x_change, y_change, 0)
        self.myturtle.forward(t * 300 * speed)

    def dance(self):
        
        self.moveForward(0.3,-1)
        time.sleep(0.1) # Needs a small delay in between functions.
        
        self.turn(45)
        time.sleep(0.1)

        self.turn(-90) 
        time.sleep(0.1)

        self.turn(360 + 45)
        time.sleep(0.1)
        
        self.moveForward(0.3,1)
        os._exit(0)

    def processVideo(self, jpegbytes, timestamp_10msec):
       
        self.fname = 'tmp.jpg'
        fd = open(self.fname, 'w')
        fd.write(jpegbytes)
        fd.close()
    
    def getImageName(self):

        return self.fname

    def detect_blue_sign(self, image_name):
        image=cv2.imread(image_name)
        cv2.imwrite('tmp0.jpg', image)
    
    # Initialize the lower and upper boundary for BLUE.
    
        blueBoundary=[([30, 0, 0], [120, 255, 255])] # purple
        #blueBoundary=[([105, 50, 50], [130, 255, 255])]
    
  # Turn the boundaries into Numpy arrays
    
        for (lower,upper) in blueBoundary:
            lower_np=np.array(lower,dtype="uint8")
            upper_np=np.array(upper,dtype="uint8")
            
    # Mask the image to keep only blue. Blue area becomes white
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask=cv2.inRange(hsv,lower_np,upper_np)
        cv2.imwrite('mask.jpg', mask)
        output=cv2.bitwise_and(image, image, mask=mask) #Returns only blue areas
        cv2.imwrite('output.jpg', output)
        imgray=cv2.cvtColor(output, cv2.COLOR_BGR2GRAY) #Turn blue into gray. Needed for binarizing.
        cv2.imwrite('imgray.jpg', imgray)

    # Find contours bordering the blue areas only.
        _, cnt, h= cv2.findContours(imgray.copy(), cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cnt = sorted(cnt, key=cv2.contourArea, reverse=True) [:-1]
    
    # Draw a rectangle around the inner contour. x,y,w,h are the corner coordinates (pixels).
    
        x,y,w,h=cv2.boundingRect(cnt[1]) 
    
    # Drawing the contours
        #cv2.drawContours(image,cnt,-1,(128,255,0),2)
    
    # Cropping the inner rectangle and saving it (to be used for OCR).

    
        return x,y,w,h, image, cnt
    
    
    def read_sign(self, sign_name):
        image = Image.open(sign_name)
        text = image_to_string(image) 
        text = text.lower().replace(" ","").replace("\n","")# Get rid of all whitespaces
        print("Command found: " + text)
        return text
    
    #def execute_sign(self, cropped_image_name):
    def execute_sign(self, command):
    # Open in PIL and do OCR
    
        #command = self.read_sign(cropped_image_name)

    # Compare command and execute accordingly.
    
        if command=="right":
            self.turn(-90)
            time.sleep(1.0)
        elif command=="left":
            self.turn(90)
            time.sleep(1.0)
        elif command=="turnaround" or command == "goback":
            self.turn(180)
            time.sleep(1.0)
        elif command=="dance":
            self.dance()
        elif command=="stop":
            os._exit(0)
        else:
            print("No command detected")
            time.sleep(0.1)
    # If no command detected for more than 3 times due to skewed image, the robot goes back in a randomly chosen angle.
        if self.nocommand_counter < 3: 
          self.nocommand_counter += 1
        else:
          self.nocommand_counter = 0
          self.turn(random.choice([-1,1]) * 15)
          self.moveForward(0.5,-1)        
                
    def compare_centroids(self,image_name,x,y,w,h):
      image = cv2.imread(image_name)
      image_x,image_y = image.shape[:2]
      img_centr_x = image_x//2
      x_centroid = (x+w)/2
      gap=img_centr_x-x_centroid
            
      return gap
    def updateState(self,x_change, y_change, theta_change):
      x , y, theta = self.state
      self.path.append(self.state)
      self.state = (x + x_change, y + y_change, theta + theta_change)
        
class point:
    def __init__(self, xy, center_point):
        self.x = xy[0]
        self.y = xy[1]
        self.center_x = center_point[0]
        self.center_y = center_point[1]
        self.distance = math.sqrt(math.pow(self.x*1.0-self.center_x*1.0, 2)+math.pow(self.y*1.0 - self.center_y*1.0, 2))  
          
    def getDistance(self):
        return self.distance
    
    def __getitem__(self):
        return self.distance
    
    def __repr__(self):
        return '(' + str(self.x) + ',' + str(self.y) + ')'
    
    def __str__(self):
        return '(' + str(self.x) + ',' + str(self.y) + ')'

def action(x, y, w, h, image, cnt):
    #myrover.execute_sign(cropped)
    ts=time.time()
    center_point = (x+w/2, y+h/2)
    cropped_cnt = [[],[],[],[]]
    for cn in cnt:
        for c in cn:
            for b in c:
                if b[0] > x and b[0] < x+w and b[1] > y and b[1] < y+h:
                    if b[0] < center_point[0] and b[1] < center_point[1]:
                        cropped_cnt[0].append(point(b,center_point))
                    elif b[0] < center_point[0] and b[1] > center_point[1]:
                        cropped_cnt[1].append(point(b,center_point))
                    elif b[0] > center_point[0] and b[1] > center_point[1]:
                        cropped_cnt[2].append(point(b,center_point))
                    else:
                        cropped_cnt[3].append(point(b,center_point))
    for cr in cropped_cnt:
        cr.sort(key=lambda point: point.distance, reverse = True)
        
    #put the function here
    markers_mat = []
    frame = image.copy()
    #convert the frame to grayscale, blur it, and detect Edges
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #blurred = cv2.medianBlur(gray,5)
    blurred = cv2.GaussianBlur(gray, (7,7), 0)
    th3 = cv2.adaptiveThreshold(blurred,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                cv2.THRESH_BINARY,11,3)

    #cv2.imshow("Frame", th3)
    edged = cv2.Canny(th3, 60, 150)
    #cv2.imshow("edge", edged)
    #find contours in the edge map
    (_, cnts, _) = cv2.findContours(edged, 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )     
    for c in cnts:     
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.01*peri, True) 
        #insure that the approximated contour is roughly rectangular
        if len(approx) >=4 and len(approx) <=60:
            #compute the bounding box of the approximated contour and 
            #use the bounding box to compute the aspect ratio
            
            (x,y,w,h) = cv2.boundingRect(approx)
            aspectRatio = w/float(h)
            
            #compute the solidity of the original contour
            area = cv2.contourArea(c)
            hullArea = cv2.contourArea(cv2.convexHull(c))
            solidity = area/float(hullArea)
            
            #compute wheter or not the width and height, solidity, and
            #aspect ratio of the contour falls within appropriate bounds
            
            keepDims= w>15 and h>15
            keepSolidity = solidity >0.7
            keepAspectRatio = aspectRatio >= 0.7 and aspectRatio<=1.3
            
            #ensure that the contour passes all our tests
            if keepDims and keepSolidity :#and keepAspectRatio:
                #draw an outline around the target and update the status
                #text 
                cv2.drawContours(frame, [approx], -1, (0,0,255),4)
                status = "Target(s) Acquired"
                
                #compute the center of the contour region and draw the 
                #crosshairs
                #print "c: " , c
                M = cv2.moments(approx)
                #print "M:",M
                (cX, cY) = (int(M["m10"] /M["m00"])), int(M["m01"]/M["m00"])
                (startX, endX) = (int(cX - (w * 0.15)), int(cX + (w * 0.15)))
                (startY, endY) = (int(cY - (h * 0.15)), int(cY + (h * 0.15)))
                #cv2.line(frame, (startX, cY), (endX, cY), (0, 255, 0), 3)
                #cv2.line(frame, (cX, startY), (cX, endY), (0, 255, 0), 3)
                #print "cx, cy", (cX,cY)
                markers_mat.append(c)                
                        
    markers_mat2 = []
    markers_points = []
    for i in xrange(len(markers_mat[:][0:100][0])):
        markers_mat2.append(markers_mat[0][i][0])
        
    #print markers_mat2[i][0]
        
    markers_mat3 = [[0 for _ in xrange(3)] for _ in xrange(len(markers_mat2))]
                    
    
    for i in xrange(len(markers_mat2)):
        markers_mat3[i][0] = markers_mat2[i][0]
        markers_mat3[i][1] = markers_mat2[i][1]
        markers_mat3[i][2] = markers_mat2[i][0]+markers_mat2[i][1]
    #print markers_mat3[i][2]
    
    markers_mat3.sort(key = lambda x:x[2])
    #delete 3rd column
    for i in xrange(len(markers_mat3)):
        del markers_mat3[i][2:3]      
    markers_points.append(markers_mat3[0])
    markers_points.append(markers_mat3[-1])                    
    approx_center_point = [(markers_points[0][0]+markers_points[1][0])/2,(markers_points[0][1]+markers_points[1][1])/2]                    
    section = [[],[],[],[]]
    
    for mmm in markers_mat:
        for mm in mmm:
            for m in mm:
                if m[0] < approx_center_point[0] and m[1] < approx_center_point[1]:
                    section[0].append(point(m, approx_center_point))
                elif m[0] < approx_center_point[0] and m[1] > approx_center_point[1]:
                    section[1].append(point(m, approx_center_point))
                elif m[0] > approx_center_point[0] and m[1] > approx_center_point[1]:
                    section[2].append(point(m, approx_center_point))
                else:
                    section[3].append(point(m, approx_center_point))
    for cr in section:
        cr.sort(key = lambda point:point.distance, reverse = True)  
    k=4    
    
    markers_points2=[]
    for cr in section:    
        markers_points2.append([cr[0].x,cr[0].y])
    

    #======== sorting ===============
    #insertion sorting becasue markers are just 4
    for i in xrange(1, len(markers_points2)): 
        current_xvalue = markers_points2[i][0]
        current_yvalue = markers_points2[i][1]
        
        position = i
        while position >0 and markers_points2[position-1][0]>current_xvalue:
            markers_points2[position][0] = markers_points2[position-1][0]
            markers_points2[position][1] = markers_points2[position-1][1]
            position = position-1
        markers_points2[position][0] = current_xvalue
        markers_points2[position][1] = current_yvalue
    #print "x_sorted:", markers_mat   
    
    #y sorting under the condition of x sorting priority   
    if markers_points2[0][1] > markers_points2[1][1] :
        markers_points2[0][0], markers_points2[1][0] = markers_points2[1][0], markers_points2[0][0]                                                                 
        markers_points2[0][1], markers_points2[1][1] = markers_points2[1][1], markers_points2[0][1]         
                                                          
    if markers_points2[2][1] > markers_points2[3][1] :
            markers_points2[2][0], markers_points2[3][0] = markers_points2[3][0], markers_points2[2][0]                                                                 
            markers_points2[2][1], markers_points2[3][1] = markers_points2[3][1], markers_points2[2][1]
    print "sorted markers coord:", markers_points2
    
    
    #to be masked screen representing by lines
    cv2.line(frame, (markers_points2[0][0], markers_points2[0][1]), 
             (markers_points2[1][0], markers_points2[1][1]),(255,0,0),3)
    cv2.line(frame, (markers_points2[1][0], markers_points2[1][1]), 
             (markers_points2[3][0], markers_points2[3][1]),(255,0,0),3)    
    cv2.line(frame, (markers_points2[0][0], markers_points2[0][1]), 
             (markers_points2[2][0], markers_points2[2][1]),(255,0,0),3)          
    cv2.line(frame, (markers_points2[2][0], markers_points2[2][1]), 
             (markers_points2[3][0], markers_points2[3][1]),(255,0,0),3)
               
    # draw the status text on the frame
    
    # show the frame and record if a key is pressed
    #cv2.imshow("Frame", frame)
    
    #0512 work : perspective Transformation
    pts1 = np.float32([[markers_points2[0][0], markers_points2[0][1]], [markers_points2[2][0], markers_points2[2][1]],
                       [markers_points2[1][0], markers_points2[1][1]], [markers_points2[3][0], markers_points2[3][1]]])
    
    print 'pts1:', pts1    
    pts2 = np.float32([[0,0], [350,0], [0,300],[350, 300]])
    
    
    
    homography_target = frame.copy()
    
    Perspective_Transformed_Mat = cv2.getPerspectiveTransform(pts1, pts2)
    dst = cv2.warpPerspective(homography_target, Perspective_Transformed_Mat, (350,300))
     
    #plt.subplot(121),plt.imshow(Perspective_Transformed_Mat),plt.title('Input')
    #plt.subplot(122),plt.imshow(dst),plt.title('Output')
    #plt.show()
    cv2.imshow('homography',dst)
    cv2.imwrite("cropped_img//" + str(ts) + ".jpg", dst)
    #cv2.imshow('target_to compare', homography_target)
    key = cv2.waitKey(0)
    
#------------------------------MAIN FUNCTION-----------------------------
def main():
    myrover = RoverSignFollower()
    myrover.turnLightsOn()
    signal.signal(signal.SIGINT, _signal_handler)
    print("Battery at " + str(myrover.getBatteryPercentage())+"%") #Check battery status

    time.sleep(2)
    sign_counter = 0 
    while True:
        try:
            name = myrover.getImageName()
            x,y,w,h, image, cnt = myrover.detect_blue_sign(name)
            gap = myrover.compare_centroids('tmp.jpg',x,y,w,h)
            if abs(gap) > 10: # Orient towards the center of the sign and move towards it.
                if gap <0:
                    myrover.turn(gap * TURN_VALUE )
                if gap > 0:
                    myrover.turn(gap * TURN_VALUE )
            if w*h < 29000:  # Keep moving till the sign is big enough to read the command.
                myrover.moveForward(0.2 , 0.7)
                time.sleep(0.1)
            else:
                print 'sh'
        
                name = myrover.getImageName()
                x,y,w,h, action_image, action_cnt = myrover.detect_blue_sign(name)
                gap = myrover.compare_centroids('tmp.jpg',x,y,w,h)
                if abs(gap) <= 10 and w*h >= 29000:
                    action(x, y, w, h, action_image, action_cnt)
                
        
        
        
#        try:
#            name = myrover.getImageName()
#            x,y,w,h, image, cnt = myrover.detect_blue_sign(name)
#            gap = myrover.compare_centroids('tmp.jpg',x,y,w,h)
#            if abs(gap) > 10: # Orient towards the center of the sign and move towards it.
#                if gap <0:
#                    myrover.turn(gap * TURN_VALUE )
#                    if gap > 0:
##                        myrover.turn(gap * TURN_VALUE )
#           if w*h < 29000:  # Keep moving till the sign is big enough to read the command.
#                myrover.moveForward(0.2 , 0.7)
#                time.sleep(0.1)
#            else:
#                for _ in xrange(7):
#                    print 'sh'
#                    time.sleep(1)
#                    name = myrover.getImageName()
#                    x,y,w,h, action_image, action_cnt = myrover.detect_blue_sign(name)
#                    gap = myrover.compare_centroids('tmp.jpg',x,y,w,h)
#                    if abs(gap) <= 10 and w*h >= 29000:
#                        action(x, y, w, h, action_image, action_cnt)
     #####################################################################               
                print 'shot'
                myrover.turn(90)
                time.sleep(1)
        except:
            traceback.print_exc()
            print("No blue sign detected")
            myrover.moveForward(0.2,-0.2) # If no blue sign detected, move forward a little and look again.
            sign_counter += 1
            if sign_counter > 10:
                sign_counter = 0
                myrover.moveForward(0.5,-0.5)

    
if __name__ == '__main__':

    main()
    





#====================================================================================
