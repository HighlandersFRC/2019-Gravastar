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
		
		
		#threshold colors to detect - Green: First value decides color, second val determines intensity, third val decides brightness
		lowerThreshold = np.array([60, 0, 20])
		upperThreshold = np.array([80, 255, 255])
		
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
			cntArea = cv2.contourArea(countour)
			if len(approx) == 4 and cntArea > 100 and cntArea < 800:
				cntArea = str(cntArea)
				jevois.sendSerial(cntArea)
				cntArray.append(countour)
		
		for cnt in cntArray:
			rotatedRect = cv2.minAreaRect(cnt)
			box = cv2.boxPoints(rotatedRect)
			boxArray = np.int0(box)
			boxColor = (0,0,255)
			targetAngle = rotatedRect[2]
			
			if targetAngle >= -90 and targetAngle <= -55:
				boxColor = (155, 120, 240)
			elif targetAngle >= -35 and targetAngle <= -0:
				boxColor = (50, 120, 240)
			else:
				cntArray.remove(cnt)
			targetAngle = str(targetAngle)
			centerX = str(rotatedRect[0][0])
			jevois.sendSerial("CenterX: " + centerX + " " + "TargetAngle: " + targetAngle)
			cv2.drawContours(outimg,[boxArray],0,boxColor,2)
			
			
		#sort cntArray
		sortedArray = sorted(cntArray, key=cv2.contourArea)
				
		#check if there is more than one contour in sortedArray; then find the two biggest and draw a rectangle around them
		arraySize = len(sortedArray)
		
		if arraySize > 1:
			max_index = arraySize - 1
			
			#First rectangle
			cnt = sortedArray[max_index]
			rotatedRect = cv2.minAreaRect(cnt)
			box = cv2.boxPoints(rotatedRect)
			bigBoxArray	= np.int0(box)
		
			cv2.drawContours(outimg,[bigBoxArray],0,(0, 0, 255),2)
			
			#Second rectangle
			cnt2 = sortedArray[max_index - 1]
			rotatedRect2 = cv2.minAreaRect(cnt2)
			box2 = cv2.boxPoints(rotatedRect2)
			secondBoxArray = np.int0(box2)
			
			cv2.drawContours(outimg,[secondBoxArray],0,(0, 0, 255),2)
			
			#center coordinates of first rectangle
			centerY = bigBoxArray[0][1]
			centerX = bigBoxArray[0][0]
			#center coordinates of second rectangle
			centerY2 = secondBoxArray[0][1]
			centerX2 = secondBoxArray[0][0]
			
			#angle of robot relative to the target
			yawAngle = (centerX - 155.5)
			yawAngle = yawAngle * 0.203125
			
			# slope (or angle) of the target
			targetAngle_A = rotatedRect[2]
			targetAngle_B = rotatedRect2[2]

			#determine which box is on the left and which is on the right
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
			#if leftAngle >= -90 and leftAngle <= -65:
			#	if rightAngle >= -30 and rightAngle <= -5:
			#		if dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
			#		yesNo = "Yes"
			#	else:
			#		yesNo = "No"
			#else:
			#	yesNo = "No"
			
			
			
			#calculate the distance to the target. The values were calculated based on trial and error
			distance = -0.0013216275 * rightY**3 + 0.3319141337 * rightY**2 + -27.11835717 * rightY + 763.1474953
			
			#change vals to string to send over serial
			yawAngle = str(yawAngle)
			distance = str(distance)
			leftAngle = str(leftAngle)
			rightAngle = str(rightAngle)
			
			#send values over serial
			#jevois.sendSerial(yawAngle)
			#jevois.sendSerial(distance)
			#jevois.sendSerial(leftAngle)
			#jevois.sendSerial(rightAngle)
			#jevois.sendSerial(yesNo)
			
			#return an image if over usb
		outimg = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)		
		return outimg