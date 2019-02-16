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
	
	def isLeft(self, contour, hsv):
		rows,cols = hsv.shape[:2]
		[vx,vy,x,y] = cv2.fitLine(contour, cv2.DIST_L2,0,0.01,0.01)
		lefty = int((-x*vy/vx) + y)
		righty = int(((cols-x)*vy/vx)+y)
		rectangle = cv2.fitLine(contour, cv2.DIST_L2,0,0.01,0.01)
		slope = math.fabs(120 - y/160 - x)
		#slope = -slope
		slope = slope * 3.14
		slope = slope/180
		angle = math.atan(slope)
		angle = angle * 180
		angle = angle/3.14
		boxColor = (0, 0, 255)
		if angle >= 0 and angle <= 45:
			boxColor = (0, 0, 255)
		elif angle >= 46 and angle <= 90:
			boxColor = (153, 255, 255)
		else:
			boxColor = (255,0, 0)
		cv2.line(hsv,(cols-1,righty),(0,lefty), boxColor,2)
		
		#angle = math.atan((vy - y)/(vx - x))
		
		angle = str(angle)
		jevois.sendSerial("Angle One" + angle)
		
		#if targetAngle >= -80 and targetAngle <= -65:
		#	return True
		#else:
		#	return False
		return True
		
	def isRight(self, contour, hsv):
		rows2,cols2 = hsv.shape[:2]
		[vx2,vy2,x2,y2] = cv2.fitLine(contour, cv2.DIST_L2,0,0.01,0.01)
		lefty2 = int((-x2*vy2/vx2) + y2)
		righty2 = int(((cols2-x2)*vy2/vx2)+y2)
		rectangle2 = cv2.fitLine(contour, cv2.DIST_L2,0,0.001,0.001)
		slope2 = math.fabs(120 - y2/160 - x2)
		#slope2 = -slope2
		slope2 = slope2 * 3.14
		slope2 = slope2/180
		angle2 = math.atan(slope2)
		angle2 = angle2 * 180
		angle2 = angle2/3.14
		boxColor = (0, 0, 255)
		if angle2 >= 0 and angle2 <= 45:
			boxColor = (0, 0, 255)
		elif angle2 >= 46 and angle2 <= 90:
			boxColor = (153, 255, 255)
		else:
			boxColor = (255,0, 0)
		cv2.line(hsv,(cols2-1,righty2),(0,lefty2),boxColor,2)
		#angle2 = math.atan((vy2 - y2)/(vx2 - x2))
		angle2 = str(angle2)
		jevois.sendSerial("Angle Two" + angle2)
	
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
		
		oKernel = np.ones((7, 7), np.uint8)
		cKernel = np.ones((7, 7), np.uint8)
		#cKernel = np.array([
		#[0, 0, 1, 0, 0],
		#[0, 1, 1, 0, 0],
		#[1, 1, 1, 1, 0],
		#[0, 1, 1, 1, 0],
		#[0, 1, 1, 1, 1],
		#[0, 0, 1, 1, 0],
		#[0, 0, 1, 0, 0]], dtype = np.uint8)
		
		
		#threshold colors to detect - Green: First value decides color, second val determines intensity, third val decides brightness
		lowerThreshold = np.array([55, 50, 15])
		upperThreshold = np.array([85, 255, 255])
		
		#check if color in range
		mask = cv2.inRange(hsv, lowerThreshold, upperThreshold)
		
		#create blur on image to reduce noise
		blur = cv2.GaussianBlur(mask,(5,5),0)
		
		ret,thresh = cv2.threshold(blur,180,255,0)
		
		#closes of noise from inside object
		closing = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, cKernel)
		
		#takes away noise from outside object
		opening = cv2.morphologyEx(closing, cv2.MORPH_OPEN, oKernel)
		
		#find contours
		
		countours, _ = cv2.findContours(closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
		
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
			boxColor = (255,0,0)
			if len(approx) == 4 and cntArea > 100 and cntArea < 800:
				targetAngle = rotatedRect[2]
				#if targetAngle >= -80 and targetAngle <= -65:
				cntArray.append(countour)
				#boxColor = (155, 200, 240)
				#elif targetAngle >= -25 and targetAngle <= -5:
				cntArray.append(countour)
				#boxColor = (50, 200, 240)

				targetAngle = str(targetAngle)
				centerX = rotatedRect[0][0]
				centerX = str(centerX)
				#jevois.sendSerial("CenterX: " + centerX + " " + "TargetAngle: " + targetAngle)
				cv2.drawContours(outimg,[boxArray],0,boxColor,2)
		#jevois.sendSerial(" ")
		
		sortedArray = self.sortContours(cntArray)
				
		
		#outimg = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)		
		opening2 = cv2.cvtColor(opening, cv2.COLOR_GRAY2BGR)
		outimg = opening2
		#time.sleep(0.25)


		if len(sortedArray) > 0:
			if self.isRight(sortedArray[0], hsv):
				sortedArray.pop(0)
		if len(sortedArray) > 0:
			if self.isLeft(sortedArray[len(sortedArray) - 1], hsv):
				sortedArray.pop(len(sortedArray) - 1)
		
		if len(sortedArray) < 2 or (len(sortedArray) % 2) != 0:
			outimg = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
			#jevois.sendSerial("Hello")
			return outimg
		
		#boxColor = (200,50, 180)
		
		xArray = []
		
		for index in range(0, len(sortedArray), 2):
			leftRect = cv2.minAreaRect(sortedArray[index])
			rightRect = cv2.minAreaRect(sortedArray[index + 1])
			points_A = cv2.boxPoints(leftRect)
			points_1 = np.int0(points_A)
			points_B = cv2.boxPoints(leftRect)
			points_2 = np.int0(points_B)
			leftX = leftRect[0][0]
			rightX = rightRect[0][0]
			cv2.drawContours(outimg, [points_1], 0, boxColor, 2)
			cv2.drawContours(outimg, [points_2], 0, boxColor, 2)
			centerX = (leftX + rightX)/2
			centerX = math.fabs(centerX - 160)
			xArray.append(centerX)
		
		minX = np.argmin(xArray)
		
		#jevois.sendSerial("Hello")
		
		centerPairLeft = sortedArray[minX * 2]
		centerPairRight = sortedArray[(minX * 2) + 1]
		
		rightRect = cv2.minAreaRect(centerPairRight)
		leftRect = cv2.minAreaRect(centerPairLeft)
		
		leftY = leftRect[0][1]
		rightY = rightRect[0][1]
		leftX = leftRect[0][0]
		rightX = rightRect[0][0]
		
		
		centerY = (leftY + rightY)/2
		centerX = (leftX + rightX)/2
		yawAngle = (centerX - 159.5) * 0.203125
		distance = -0.00002750028278 * centerY **3 + 0.0110106527 * centerY ** 2 -0.7826513252 * centerY + 51.55036834
		
		
		realWorldPointY = (centerX - 159.5)/251
		realWorldPointY = realWorldPointY * distance
		
		
		#change vals to string to send over serial
		centerY = str(centerY)
		distance = str(distance)
		leftY = str(leftY)
		rightY = str(rightY)
		realWorldPointY = str(realWorldPointY)
		centerX = str(centerX)
		
		
		#send vals over serial
		jevois.sendSerial("RealWorldPoint: " + realWorldPointY + "centerX: " + centerX)
		#jevois.sendSerial("Hello World")
		return outimg