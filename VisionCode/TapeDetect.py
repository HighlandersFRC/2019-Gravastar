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
		
		errPass = " "
		
		#threshold colors to detect
		lowerThreshold = np.array([65, 0, 0])
		upperThreshold = np.array([75, 255, 255])
		
		#check if color in range
		mask = cv2.inRange(hsv, lowerThreshold, upperThreshold)
		
		#create blur on image to reduce noise
		blur = cv2.GaussianBlur(mask,(5,5),0)
		
		#find contours
		ret,thresh = cv2.threshold(blur,127,255,0)
		countours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
		
		#sift through contours to find the two biggest 4 sided contours
		
		##### change logic to have loop that identifies only four sided contours and copy them to another array
		##### outside loop, sort array to find the biggest two contours
		
		#areas = [cv2.contourArea(c) for c in countours]
		#areas = np.sort(areas)
		cntArray = []
		
		
		for countour in countours:
			peri = cv2.arcLength(countour, True)
			approx = cv2.approxPolyDP(countour, 0.04 * peri, True)
			if len(approx) == 4:
				cntArray.append(countour)
				#max_index = np.argmax(areas)
			
		
		sortedArray = sorted(cntArray, key=cv2.contourArea)
		#areas = [cv2.contourArea(c) for c in cntArray]
		
		
		#sortedArray = areas.sort()
		
		
		max_index = len(sortedArray)
		if max_index > 0:
			cnt = sortedArray[max_index - 1]
			rotatedRect = cv2.minAreaRect(cnt)
			box = cv2.boxPoints(rotatedRect)
			box_A = np.int0(box)
		
			cv2.drawContours(outimg,[box_A],0,(0, 0, 255),2)
		#angleOne = box_A.angle
		#lowestCoordinate = box[0]
		
		#cv2.rectangle(inimg, lowestCoordinate,(lowestCoordinate[0] + box_A[2][0],lowestCoordinate[1] + box_A[2][1]),(0,255,0),1)
		
		#cnt2 = sortedArray[max_index - 2]
		#rotatedRect2 = cv2.minAreaRect(cnt2)
		#box2 = cv2.boxPoints(rotatedRect2)
		#box2_B = np.int0(box2)
		#angleTwo = rotatedRect2.angle
		#lowestCoordinate2 = box2[0]
		
		#cv2.rectangle(inimg, (x, y), (x+w, y+h), (0,255,0), 1)
				
		outimg = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
		#lowestCoordinate = str(lowestCoordinate)
		#jevois.sendSerial(lowestCoordinate)
		#max_index = str(max_index)
		#box = str(box)
		#angleOne = str(angleOne)
		#jevois.sendSerial(angleOne)
		#cntArray = str(cntArray)
		#jevois.sendSerial(max_index)
		#jevois.sendSerial(box)
		return outimg