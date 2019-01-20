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
		jevois.sendSerial("Hello World")
        
	def processNoUSB(self, inframe):
		#jevois.sendSerial("Hello World")
		runcount = 0
		#get image
		inimg = inframe.getCvBGR()
		
		self.timer.start()
		
		#change to hsv
		hsv = cv2.cvtColor(inimg, cv2.COLOR_BGR2HSV)
		outimg = hsv
		
		errPass = " "
		
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
				areas = np.sort(areas)
				max_index = np.argmax(areas)
				
				cnt= countours[max_index]
				rotatedRect = cv2.minAreaRect(cnt)
				box = cv2.boxPoints(rotatedRect)
				box = np.int0(box)
				angleOne = rotatedRect[2]
				lowestCoordinate = box[0]
				
				#cv2.rectangle(inimg,(x,y),(x+w,y+h),(0,255,0),1)
				
				cnt2 = countours[max_index - 1]
				rotatedRect2 = cv2.minAreaRect(cnt2)
				box2 = cv2.boxPoints(rotatedRect2)
				box2 = np.int0(box2)
				angleTwo = rotatedRect2[2]
				lowestCoordinate2 = box2[0]
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
		#if angleOne == angleTwo:
		#	yesNo = "yes"
		#else:
		#	yesNo = "no"
		
	
		
		x = lowestCoordinate[0]
		y = lowestCoordinate[1]
		x2 = lowestCoordinate2[0]
		y2 = lowestCoordinate2[1]
		centerX = x/2
		centerY = y/2
		
		if x < x2:
			leftCoordinate = lowestCoordinate
			rightCoordinate = lowestCoordinate2
			leftAngle = angleOne
			rightAngle = angleTwo
		else:
			leftCoordinate = lowestCoordinate2
			rightCoordinate = lowestCoordinate
			leftAngle = angleTwo
			rightAngle = angleTwo
			
		
		distance = -0.0033508576 * y**3
		distance = distance + 0.8429538777 * y**2
		distance = distance - 68.64070128 * x
		distance = distance + 1882.753427
		distance = distance/y
		
		leftCoordinateY = leftCoordinate[1]
		leftCoordinateX = leftCoordinate[0]
		rightCoordinateY = rightCoordinate[1]
		rightCoordinateX = rightCoordinate[0]
		
		
		#change vals to strings to send over serial
		angleOne = str(angleOne)
		#angleTwo = str(angleTwo)
		lowestCoordinate = str(lowestCoordinate)
		x = str(x)
		distance = str(distance)
		centerY = str(centerY)
		leftCoordinateY = str(leftCoordinateY)	
			
		#JSON not fully working yet
		#X_Y = {"X" : centerX, "Y" : centerY, "X2" : centerX2, "Y2" : centerY2}
		
		
		
		#send vals through serial

		jevois.sendSerial(leftCoordinateY +  " " + yesNo)

		
		
		#jevois.sendSerial(centerX, centerY, centerX2, centerY2, yUpPointsDist, twoPointsDist)
		
		fps = self.timer.stop()
		height = outimg.shape[0]
		width = outimg.shape[1]
		#jevois.sendSerial(X_Y)
		
		#outframe.sendCv(outimg)
		
		#jevois.sendSerial("Hello Simon")
		pass