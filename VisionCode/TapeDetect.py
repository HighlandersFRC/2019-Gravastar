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
		inimg = inframe.getCvBGR()
		
		self.timer.start()
		
		hsv = cv2.cvtColor(inimg, cv2.COLOR_BGR2HSV)
		outimg = hsv
		
		lowerThreshold = np.array([65, 0, 0])
		upperThreshold = np.array([70, 255, 255])
		mask = cv2.inRange(hsv, lowerThreshold, upperThreshold)
		blur = cv2.GaussianBlur(mask,(5,5),0)
		
		ret,thresh = cv2.threshold(blur,127,255,0)
		countours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
		
		for countour in countours:
			peri = cv2.arcLength(countour, True)
			approx = cv2.approxPolyDP(countour, 0.04 * peri, True)
			if len(approx) == 4:
				areas = [cv2.contourArea(c) for c in countours]
				max_index = np.argmax(areas)
				cnt=countours[max_index]
				x,y,w,h = cv2.boundingRect(cnt)
				cv2.rectangle(inimg,(x,y),(x+w,y+h),(0,255,0),1)
				areas = [cv2.contourArea(c) for c in countours]
				index2 = np.argmax(areas)
				cnt2 = countours[index2 - 1]
				x2, y2, w2, h2 = cv2.boundingRect(cnt2)
				cv2.rectangle(inimg, (x, y), (x+w, y+h), (0,255,0), 1)
			
			
		
		#areas = float(cv2.contourArea(cnt))
		centerX = x + w/2
		centerY = y + h/2
		centerX2 = x2 + w2/2
		centerY2 = y2 + h2/2
		x = x - 160
		y = y - 120
		distance = -0.0033508576 * y**3
		distance = distance + 0.8429538777 * y**2
		distance = distance - 68.64070128 * x
		distance = distance + 1882.753427
		distance = distance/y
		centerXinches = 2 * distance * math.tan((65/2)*(math.pi/180))
		centerXinches = centerXinches/320
		centerXinches = centerXinches * x
		Angle = math.atan(centerXinches/distance)
		Angle = Angle * 180
		Angle = Angle/math.pi
		
		Dist_and_Angle = '{"Distance" : distance, "Angle" : angle}'
		
		#jevois.sendSerial(str(distance).format + str(Angle).format)
		
		jevois.sendSerial(str(Dist_and_Angle))
		
		fps = self.timer.stop()
		height = outimg.shape[0]
		width = outimg.shape[1]
		jevois.sendSerial(str(centerX) + " " + str(centerY) + " " + str(centerX2) + " " + str(centerY2))
		
		#outframe.sendCv(outimg)
		
		#jevois.sendSerial("Hello Simon")
		pass