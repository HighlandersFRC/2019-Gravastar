import libjevois as jevois
import cv2
import numpy as np
import json
import math
import time

#MOVE TARGET ANGLE TO FIRST FOR LOOP
#Draw contours in first for loop
#

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
		
		
		
	def sortContours(self, cntArray):
		arraySize = len(cntArray)
		if arraySize == 0:
			return []
			

		sortedArray = [cntArray[0]]
		for i in range(1, arraySize):
			
			rectangle = cv2.minAreaRect(cntArray[i])
			
			for j in range(len(sortedArray)):
				sortedRect = cv2.minAreaRect(sortedArray[j])
				if rectangle[0][0] <= sortedRect[0][0]:
					sortedArray.insert(j, cntArray[i])
					break
				if j == (len(sortedArray) - 1):
					sortedArray.append(cntArray[i])
		
		
		return sortedArray
		
		
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
		
		oKernel = np.ones((13, 13), np.uint8)
		cKernel = np.array([
		[0, 0, 1, 0, 0],
		[0, 1, 1, 0, 0],
		[1, 1, 1, 1, 0],
		[0, 1, 1, 1, 0],
		[0, 1, 1, 1, 1],
		[0, 0, 1, 1, 0],
		[0, 0, 1, 0, 0]], dtype = np.uint8)
		
		
		#threshold colors to detect - Green: First value decides color, second val determines intensity, third val decides brightness
		lowerThreshold = np.array([50, 50, 45])
		upperThreshold = np.array([85, 255, 255])
		
		#check if color in range
		mask = cv2.inRange(hsv, lowerThreshold, upperThreshold)
		
		#create blur on image to reduce noise
		blur = cv2.GaussianBlur(mask,(5,5),0)
		
		#closes of noise from inside object
		closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cKernel)
		
		#takes away noise from outside object
		opening = cv2.morphologyEx(closing, cv2.MORPH_OPEN, oKernel)
		
		#find contours
		#ret,thresh = cv2.threshold(closing,127,255,0)
		countours, _ = cv2.findContours(opening, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
		
		cntArray = []
		yesNo = " "
		pairsList = []
		
		#sift through contours and add 4 sided contours to cntArray
		for countour in countours:
			peri = cv2.arcLength(countour, True)
			approx = cv2.approxPolyDP(countour, 0.04 * peri, True)
			cntArea = cv2.contourArea(countour)
			rotatedRect = cv2.minAreaRect(countour)
			box = cv2.boxPoints(rotatedRect)
			boxArray = np.int0(box)
			boxColor = (0,0,255)
			if len(approx) == 4 and cntArea > 100 and cntArea < 800:

				targetAngle = rotatedRect[2]
				if targetAngle >= -80 and targetAngle <= -65:
					cntArray.append(countour)
					boxColor = (155, 200, 240)
				elif targetAngle >= -25 and targetAngle <= -5:
					cntArray.append(countour)
					boxColor = (50, 200, 240)
			
				targetAngle = str(targetAngle)
				centerX = rotatedRect[0][0]
				centerX = str(centerX)
				jevois.sendSerial("CenterX: " + centerX + " " + "TargetAngle: " + targetAngle)
				cv2.drawContours(outimg,[boxArray],0,boxColor,2)
				#jevois.sendSerial(" ")
		
		sortedArray = self.sortContours(cntArray)
				
		
		for cnt in sortedArray:
			rect = cv2.minAreaRect(cnt)
			centerX = rect[0][0]
			centerX = str(centerX)
			jevois.sendSerial(centerX)
		
		jevois.sendSerial(" ")		

		
		#index = len(cntArray)
		#index = str(index)
		#jevois.sendSerial(index)
			
		#sort cntArray
		#sortedArray = sorted(cntArray, key=cv2.contourArea)
				
		#check if there is more than one contour in sortedArray; then find the two biggest and draw a rectangle around them
		#arraySize = len(sortedArray)
		
		#if arraySize > 1:
		#	max_index = arraySize - 1
			
		#	#First rectangle
		#	cnt = sortedArray[max_index]
		#	rotatedRect = cv2.minAreaRect(cnt)
		#	box = cv2.boxPoints(rotatedRect)
		#	bigBoxArray	= np.int0(box)
		#
		#	cv2.drawContours(outimg,[bigBoxArray],0,(0, 0, 255),2)
			
			#Second rectangle
		#	cnt2 = sortedArray[max_index - 1]
		#	rotatedRect2 = cv2.minAreaRect(cnt2)
		#	box2 = cv2.boxPoints(rotatedRect2)
		#	secondBoxArray = np.int0(box2)
		
		#	cv2.drawContours(outimg,[secondBoxArray],0,(0, 0, 255),2)
			
			#center coordinates of first rectangle
		##	centerY = bigBoxArray[0][1]
			#centerX = bigBoxArray[0][0]
			#center coordinates of second rectangle
			#centerY2 = secondBoxArray[0][1]
			#centerX2 = secondBoxArray[0][0]
			
			#angle of robot relative to the target
		#	yawAngle = (centerX - 155.5)
		#	yawAngle = yawAngle * 0.203125
			
			# slope (or angle) of the target
		#	targetAngle_A = rotatedRect[2]
		#	targetAngle_B = rotatedRect2[2]

			#determine which box is on the left and which is on the right
		#	if centerX < centerX2:
		#		leftY = centerY
		#		leftX = centerX
		#		rightY = centerY2
		#		rightX = centerX2
		#		leftAngle = targetAngle_A
		#		rightAngle = targetAngle_B
		#	else:
		#		leftY = centerY2
		#		leftX = centerX2
		#		rightY = centerY
		#		rightX = centerX
		#		leftAngle = targetAngle_B
		#		rightAngle = targetAngle_A
			
			#dist = math.sqrt((leftX - rightX)**2 + (leftY - rightY)**2)
			
			# decide whether we are detecting the correct targets
			#if leftAngle >= -90 and leftAngle <= -65:
			#	if rightAngle >= -30 and rightAngle <= -5:
			#		yesNo = "Yes"
			#	else:
			#		yesNo = "No"
			#else:
			#	yesNo = "No"
						
			
			#calculate the distance to the target. The values were calculated based on trial and error
		#	distance = -0.0013216275 * rightY**3 + 0.3319141337 * rightY**2 + -27.11835717 * rightY + 763.1474953
			
			#change vals to string to send over serial
			#yawAngle = str(yawAngle)
			#distance = str(distance)
			#leftAngle = str(leftAngle)
			#rightAngle = str(rightAngle)
			#dist = str(dist)
		#	centerY = str(centerY)
			
			#send values over serial
			#jevois.sendSerial(yawAngle)
			#jevois.sendSerial(distance)
			#jevois.sendSerial(leftAngle)
			#jevois.sendSerial(rightAngle)
			#jevois.sendSerial(yesNo)
		#	jevois.sendSerial(centerY)
			
			#return an image if over usb
		outimg = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)		
		#opening2 = cv2.cvtColor(blur, cv2.COLOR_GRAY2BGR)
		#outimg = opening2
		#time.sleep(0.25)
		return outimg