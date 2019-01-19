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
		jevois.sendSerial("Hello World Process")
        
	def processNoUSB(self, inframe):
		runcount = 0
		#get image
		inimg = inframe.getCvBGR()
		
		self.timer.start()
		
		#change to hsv
		hsv = cv2.cvtColor(inimg, cv2.COLOR_BGR2HSV)
		outimg = hsv
		
		
		#threshold colors to detect
		lowerThreshold = np.array([60, 0, 0])
		upperThreshold = np.array([75, 255, 255])
		
		#check if color in range
		mask = cv2.inRange(hsv, lowerThreshold, upperThreshold)
		
		#create blur on image to reduce noise
		blur = cv2.GaussianBlur(mask,(5,5),0)
		
		#find contours
		ret,thresh = cv2.threshold(blur,127,255,0)
		countours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
		
		#sift through contours to find two biggest 4 sided contours
		for countour in countours:
			peri = cv2.arcLength(countour, True)
			approx = cv2.approxPolyDP(countour, 0.04 * peri, True)
			if len(approx) == 4:
				areas = [cv2.contourArea(c) for c in countours]
				max_index = np.argmax(areas)
				cnt=countours[max_index]
				rotatedRect = cv2.minAreaRect(cnt)
				box = cv2.boxPoints(rotatedRect)
				box = np.int0(box)
				angleOne = rotatedRect[2]
				lowestCoordinate = box[0]
				
				#cv2.rectangle(inimg,(x,y),(x+w,y+h),(0,255,0),1)
				areas = [cv2.contourArea(c) for c in countours]
				index2 = np.argmax(areas)
				cnt2 = countours[index2 - 1]
				rotatedRect2 = cv2.minAreaRect(cnt2)
				box2 = cv2.boxPoints(rotatedRect2)
				box = np.int0(box)
				angleTwo = rotatedRect2[2]
				#cv2.rectangle(inimg, (x, y), (x+w, y+h), (0,255,0), 1)
			
			
			yesNo = " "
		
		#areas = float(cv2.contourArea(cnt))
		#find center (x,y) of contours
		#centerX = x + w/2
		#centerY = y + h/2
		#centerX2 = x2 + w2/2
		#centerY2 = y2 + h2/2
		#x = x - 160
		#y = y - 120
		
		#determine whether we are detecting the correct contour based on if the points at the top of the contours are closer than the points at the bottoms of the contours
		if angleOne == angleTwo:
			yesNo = "yes"
		else:
			yesNo = "no"
		
		#bottomPointOne = 
		
		#bottomPointsDist = 
		
		#change vals to strings to send over serial
		#centerX = str(centerX)
		#centerY = str(centerY)
		#centerY2 = str(centerY2)
		#centerX2 = str(centerX2)
		#twoPointsDist = str(twoPointsDist)
		#yUpPointsDist = str(yUpPointsDist)
		angleOne = str(angleOne)
		angleTwo = str(angleTwo)
		lowestCoordinate = str(lowestCoordinate)
		
		#JSON not fully working yet
		#X_Y = {"X" : centerX, "Y" : centerY, "X2" : centerX2, "Y2" : centerY2}
		
		#jevois.sendSerial(str(distance).format + str(Angle).format)
		
		#jevois.sendSerial(str(X_Y).format)
		
		#send vals through serial
		#jevois.sendSerial(centerX)
		#jevois.sendSerial(centerY)
		#jevois.sendSerial(centerX2)
		#jevois.sendSerial(centerY2)
		#jevois.sendSerial(yUpPointsDist)
		#jevois.sendSerial(twoPointsDist)
		jevois.sendSerial(angleOne + " " + angleTwo + " " + yesNo)
		#jevois.sendSerial(lowestCoordinate)
		
		
		#jevois.sendSerial(centerX, centerY, centerX2, centerY2, yUpPointsDist, twoPointsDist)
		
		fps = self.timer.stop()
		height = outimg.shape[0]
		width = outimg.shape[1]
		#jevois.sendSerial(X_Y)
		
		#outframe.sendCv(outimg)
		
		#jevois.sendSerial("Hello Simon")
		pass