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
		
		self.draw = True
        
    # ###################################################################################################
    ## Process function with USB output
	def process(self, inframe, outframe):
		out = self.UniversalProcess(inframe)
		outframe.sendCv(out)
		

	def processNoUSB(self, inframe):
		out = self.UniversalProcess(inframe)
		
		
	def UniversalProcess(self, inframe):
		#jevois.sendSerial("Hello World")
		#jevois.sendSerial("Hello World")
		runcount = 0
		#get image
		inimg = inframe.getCvBGR()
		outimg = inimg
		
		
		#change to hsv
		hsv = cv2.cvtColor(inimg, cv2.COLOR_BGR2HSV)
		outimg = hsv
		
		
		#threshold colors to detect
		lowerThreshold = np.array([65, 0, 20])
		upperThreshold = np.array([75, 255, 255])
		
		#check if color in range
		mask = cv2.inRange(hsv, lowerThreshold, upperThreshold)
		
		#create blur on image to reduce noise
		blur = cv2.GaussianBlur(mask,(5,5),0)
		
		#find contours
		ret,thresh = cv2.threshold(blur,127,255,0)
		countours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
		
		cntArray = []
		yesNo = " "
		
		
		#sift through contours and add 4 sided contours to cntArray
		for countour in countours:
			peri = cv2.arcLength(countour, True)
			approx = cv2.approxPolyDP(countour, 0.04 * peri, True)
			if len(approx) == 4 and cv2.contourArea(countour) > 25:
				cntArray.append(countour)
			
		#sort cntArray
		sortedArray = sorted(cntArray, key=cv2.contourArea)
				
		#check if there are more than one contours in sortedArray then find the two biggest and draw a rectangle around them
		arraySize = len(sortedArray)
		
		if arraySize > 1:
			max_index = arraySize - 1
			cnt = sortedArray[max_index]
			rotatedRect = cv2.minAreaRect(cnt)
			box = cv2.boxPoints(rotatedRect)
			bigBoxArray	= np.int0(box)
		
			cv2.drawContours(outimg,[bigBoxArray],0,(0, 0, 255),2)
			
			
			cnt2 = sortedArray[max_index - 1]
			rotatedRect2 = cv2.minAreaRect(cnt2)
			box2 = cv2.boxPoints(rotatedRect2)
			secondBoxArray = np.int0(box2)
			
			cv2.drawContours(outimg,[secondBoxArray],0,(0, 0, 255),2)
			
			#center coordinates of first rectangle
			centerY = bigBoxArray[0][1]
			centerX = bigBoxArray[0][0]
			centerY2 = secondBoxArray[0][1]
			centerX2 = secondBoxArray[0][0]
			
			#angle of robot relative to the target
			yawAngle = (centerX - 155.5)
			yawAngle = yawAngle * 0.203125
			
			# angle at which the target is angled
			targetAngle_A = bigBoxArray[2]
			targetAngle_B = secondBoxArray[2]

			#determine which box is on the left or right
			if centerX < centerX2:
				leftY = centerY
				leftX = centerX
				rightY = centerY2
				rightX = centerX2
				leftAngle = targetAngle_A
				rightAngle = targetAngle_B
			else:
				leftY = centerY2
				leftX = centerX2
				rightY = centerY
				rightX = centerX
				leftAngle = targetAngle_B
				rightAngle = targetAngle_A
			

			# decide whether we are detecting the correct targets
			#if leftAngle == rightAngle:
			#	yesNo = "Yes"
			
			#else:
			#	yesNo = "No"
			
			distance = -0.0013216275 * rightY**3 + 0.3319141337 * rightY**2 + -27.11835717 * rightY + 763.1474953
			
			#change vals to string to send over serial
			yawAngle = str(yawAngle)
			rightY = str(rightY)
			distance = str(distance)
			
			#send values over serial
			#jevois.sendSerial(yawAngle)
			#jevois.sendSerial(yesNo)
			#jevois.sendSerial(rightY)
			jevois.sendSerial(distance)
			#return an image if over usb
		outimg = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)		
		return outimg