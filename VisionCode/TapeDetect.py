import libjevois as jevois
import cv2
import numpy as np
import json
import math

class TapeDetect:
    # ###################################################################################################
    ## Constructor
    
    
    def __init__(self):
        # Instantiate a JeVois Timer to measure our processing framerate:
        self.timer = jevois.Timer("processing timer", 100, jevois.LOG_INFO)
        
    # ###################################################################################################
    ## Process function with USB output
    def process(self, inframe, outframe):
        # Get the next camera image (may block until it is captured) and here convert it to OpenCV BGR. If you need a
        # grayscale image, just use getCvGRAY() instead of getCvBGR(). Also supported are getCvRGB() and getCvRGBA():
        runCount = 0
        inimg = inframe.getCvBGR()
        
        # Start measuring image processing time (NOTE: does not account for input conversion time):
        self.timer.start()
    
        hsv = cv2.cvtColor(inimg, cv2.COLOR_BGR2HSV)
        outimg = hsv
        
        
        
        lowerThreshold = np.array([65, 0, 0])
        upperThreshold = np.array([70, 255, 255])
        mask = cv2.inRange(hsv, lowerThreshold, upperThreshold)
        blur = cv2.GaussianBlur(mask,(5,5),0)
        
        
        ret,thresh = cv2.threshold(blur,127,255,0)
        countours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        #cv2.drawContours(inimg, countours, -1, (0, 255, 0),-1)
        
		#cnt = 0
        for countour in countours:
            peri = cv2.arcLength(countour, True)
            approx = cv2.approxPolyDP(countour, 0.04 * peri, True)
            if len(approx) == 4:
                areas = [cv2.contourArea(c) for c in countours]
                max_index = np.argmax(areas)
                cnt=countours[max_index]
                x,y,w,h = cv2.boundingRect(cnt)
                cv2.rectangle(inimg,(x,y),(x+w,y+h),(0,255,0),1)
                
        areas = float(cv2.contourArea(cnt))
        distance = math.log(areas/2710, 0.9719)
        centerX = x/2
        centerY = y/2
        x = x - 160
        y = y - 120
        distance = 3686.42
        distance = distance/y
        centerXinches = 2 * distance * math.tan((65/2)*(math.pi/180))
        centerXinches = centerXinches/320
        centerXinches = centerXinches * x
        Angle = math.atan(centerXinches/distance)
        Angle = Angle * 180
        Angle = Angle/math.pi
        #Angle = Angle - 45
        #distance = distance *105.26
        #distance = 27.823 * math.sin(0.661 * centerY + 0.5461) +77.5627
        #max_index = np.argmax(areas)
        #cnt=contours[max_index]

        Dist_and_Angle = {"Distance" : distance, "Angle" : Angle}
        #x_y = {"X" : x, "Y" : y}
        #jevois.sendSerial(str(distance).format() + " 25.0".format());
        #jevois.sendSerial(str(areas).format())
        jevois.sendSerial(str(distance).format + str(Angle).format)
        outimg = inimg
        # Write a title:
        cv2.putText(outimg, "JeVois TapeDetectTrialOne", (3, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255))
        
        
        
        
        
        # Write frames/s info from our timer into the edge map (NOTE: does not account for output conversion time):
        fps = self.timer.stop()
        height = outimg.shape[0]
        width = outimg.shape[1]
        cv2.putText(outimg, fps, (3, height - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255))
        
        # Convert our output image to video output format and send to host over USB:
        outframe.sendCv(outimg)
        
    def processNoUSB(self, inframe):
	    # Get the next camera image (may block until it is captured) and here convert it to OpenCV BGR. If you need a
        # grayscale image, just use getCvGRAY() instead of getCvBGR(). Also supported are getCvRGB() and getCvRGBA():
        runCount = 0
        inimg = inframe.getCvBGR()
        
        # Start measuring image processing time (NOTE: does not account for input conversion time):
        self.timer.start()
    
        hsv = cv2.cvtColor(inimg, cv2.COLOR_BGR2HSV)
        outimg = hsv
        
        
        
        lowerThreshold = np.array([65, 0, 0])
        upperThreshold = np.array([70, 255, 255])
        mask = cv2.inRange(hsv, lowerThreshold, upperThreshold)
        blur = cv2.GaussianBlur(mask,(5,5),0)
        
        
        ret,thresh = cv2.threshold(blur,127,255,0)
        countours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        #cv2.drawContours(inimg, countours, -1, (0, 255, 0),-1)
        
        for countour in countours:
            peri = cv2.arcLength(countour, True)
            approx = cv2.approxPolyDP(countour, 0.04 * peri, True)
            if len(approx) == 4:
                areas = [cv2.contourArea(c) for c in countours]
                max_index = np.argmax(areas)
                cnt=countours[max_index]
                x,y,w,h = cv2.boundingRect(cnt)
                cv2.rectangle(inimg,(x,y),(x+w,y+h),(0,255,0),1)
                
        areas = float(cv2.contourArea(cnt))
        distance = math.log(areas/2710, 0.9719)
        centerX = x/2
        centerY = y/2
        x = x - 160
        y = y - 120
        distance = 3686.42
        distance = distance/y
        centerXinches = 2 * distance * math.tan((65/2)*(math.pi/180))
        centerXinches = centerXinches/320
        centerXinches = centerXinches * x
        Angle = math.atan(centerXinches/distance)
        Angle = Angle * 180
        Angle = Angle/math.pi
        #Angle = Angle - 45
        #distance = distance *105.26
        #distance = 27.823 * math.sin(0.661 * centerY + 0.5461) +77.5627
        #max_index = np.argmax(areas)
        #cnt=contours[max_index]

        #Dist_and_Angle = {"Distance" : distance, "Angle" : Angle}
        #x_y = {"X" : x, "Y" : y}
        #jevois.sendSerial(str(distance).format() + " 25.0".format());
        #jevois.sendSerial(str(areas).format())
        jevois.sendSerial(str(Distance).format + str(Angle).format)
        outimg = inimg
        # Write a title:
        cv2.putText(outimg, "JeVois TapeDetectTrialOne", (3, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255))
        
        
        
        
        
        # Write frames/s info from our timer into the edge map (NOTE: does not account for output conversion time):
        fps = self.timer.stop()
        height = outimg.shape[0]
        width = outimg.shape[1]
        cv2.putText(outimg, fps, (3, height - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255))
        
        # Convert our output image to video output format and send to host over USB:
        outframe.sendCv(outimg)
        

