import numpy as np
import config as cfg
import cv2
import imutils
from imutils.object_detection import non_max_suppression
from picamera import PiCamera 
from time import sleep, time

REAL_DISTANCE = 23 # cm
REAL_WIDTH = 12 # cm
PIXEL_WIDTH_AT_REAL_DIST = 257 # pixels
FOCAL_LENGTH = (PIXEL_WIDTH_AT_REAL_DIST * REAL_DISTANCE) / REAL_WIDTH

##
## Finds distance to an object in centimeters, needs calibration values to determine focal length 
##
## :param      pixelWidth:  The pixel width
##
## :returns:   { real distance to an object }
##
def findCameraDistance(pixelWidth) :
	return (REAL_WIDTH * FOCAL_LENGTH) / pixelWidth



##
## Finds a distance to device sending signal of given strength, uses logarithmic distance path loss model found in paper: 
## https://file.scirp.org/pdf/CN_2013071010352139.pdf
##
## :param      rssi:  The rssi [dB]
## :type       rssi:  int
##
## :returns:   distance to device in meters
## :rtype:     double
##
def findRssiDistance(rssi) :
	# Rssi Distance calibration values
	d_0 = 1.0 #distance at which calibration rssi was measured
	rssi_0 = -30.0	# signal strength at distance d_0
	eta = 4.0 	# signal attenuation factor typically value between 2 and 6
	base = 10.0
	exponent = (rssi_0 - rssi)/(10 * eta)
	return d_0*pow(base, exponent)


def rectArea(width, height) :
	return width * height

##
## calculates moving average of measured values, works as low pass filter
##
## :param      measurement:        The measurement
## :type       measurement:        float
## :param      windowSize:         The window size
## :type       windowSize:         integer
## :param      measurementsArray:  The measurements array
## :type       measurementsArray:  array
##
## :returns:   moving average
## :rtype:     float
##
def movingAverage(measurement, measurementsArray, windowSize=5) :

	if measurement != None:
		measurementsArray.append(measurement)
	arrayLength = len(measurementsArray)
	if arrayLength > windowSize :
		measurementsArray = measurementsArray[(arrayLength - windowSize): (arrayLength - 1)]
	return np.average(measurementsArray)

def horizontalPositionControl_PID() :

	# global previousError, integralError, loopFreq, measurement, setpoint, Kp, Kd, Ki, threadStopper

	loopPeriod = 1 / cfg.horizontal_loopFreq

	try :
		while not cfg.threadStopper.is_set() :
			start = time()

			error = cfg.horizontal_setpoint - cfg.horizontal_measurement #calculating current error
			# print("cfg.horizontal_setpoint", cfg.horizontal_setpoint, "cfg.horizontal_measurement", cfg.horizontal_measurement)

			proportional_output = cfg.horizontal_Kp * error #Output of proportional controller

			diff_error = (error-cfg.horizontal_previousError) #dividing by timestep is not necessary since this can be compensated for by tuning Kd 
			cfg.horizontal_previousError = error
			# print("error: ", error, "preverror: ", cfg.horizontal_previousError)

			diff_output = cfg.horizontal_Kd * diff_error #output of differential controller
			# print("cfg.horizontal_Kd: ", cfg.horizontal_Kd, "diff_error: ", diff_error)

			if cfg.horizontal_Ki < 0.00001 and cfg.horizontal_Ki > -0.00001 :
				cfg.horizontal_integralError = 0.0 #calculating the integral, again timestep is not necessary
			else :
				cfg.horizontal_integralError += error

			integral_output = cfg.horizontal_Ki * cfg.horizontal_integralError  #integral controller output
			# print("cfg.horizontal_Ki", cfg.horizontal_Ki, cfg.horizontal_integralError)

			cfg.horizontal_correction = proportional_output + diff_output + integral_output #check if the output needs to be negative or not
			# print("proportional out: %.3f, differential out: %.3f, integral out: %.3f" % (proportional_output, diff_output, integral_output))
			# print(" IN UTILS:: correction: ", cfg.horizontal_correction, "cfg.measurement: ", cfg.horizontal_measurement, "cfg.setpoint", cfg.horizontal_setpoint)
			# make sure loop frequency is fairly constant
			end = time()
			delayDiff = end - start
			if loopPeriod - delayDiff > 0 :
				sleep(loopPeriod - delayDiff)
	except Exception as err:
		print(str(err))
		cfg.threadStopper.set()
	finally:
		cfg.GPG.stop()

def distanceControl_PID() :

	# global previousError, integralError, loopFreq, measurement, setpoint, Kp, Kd, Ki, threadStopper

	loopPeriod = 1 / cfg.distance_loopFreq

	try :
		# print("begin: ", time())
		# print("isthreadstopped: ", cfg.threadStopper.is_set())
		while not cfg.threadStopper.is_set() :
			start = time()
			preverror = cfg.distance_previousError # save old error
			# integral_error = cfg.distance_integralError

			error = cfg.distance_setpoint - cfg.distance_measurement #calculating current error
			#print ("distance_error: ", error)

			proportional_output = cfg.distance_Kp * error #Output of proportional controller

			diff_error = (error-preverror) #dividing by timestep is not necessary since this can be compensated for by tuning Kd 

			diff_output = cfg.distance_Kd * diff_error #output of differential controller

			if cfg.distance_Ki < 0.0001 and cfg.distance_Ki > -0.0001 :
				cfg.distance_integralError = 0.0 #calculating the integral, again timestep is not necessary
			else :
				cfg.distance_integralError += error

			integral_output = cfg.distance_Ki * cfg.distance_integralError  #integral controller output

			cfg.distance_correction = proportional_output + diff_output + integral_output #check if the output needs to be negative or not

			end = time()
			delayDiff = end - start
			if loopPeriod - delayDiff > 0 :
				sleep(loopPeriod - delayDiff)
	except Exception as err:
		print(str(err))
		cfg.threadStopper.set()
	finally:
		cfg.GPG.stop()


# Initialization methods

def cameraInit() :
    camera = PiCamera()
    camera.resolution = (cfg.FRAME_WIDTH, cfg.FRAME_HEIGHT)
    camera.framerate = 20
    camera.rotation = 180
    return camera

def getHSVColorLimitsFromBGR(blue, green, red, lowerSaturation=100,  lowerValue=50, upperSaturation=255,  upperValue=255) :
	        #Set limits for color filter
    color = np.uint8([[[blue, green, red]]])
    hsvColor = cv2.cvtColor(color,cv2.COLOR_BGR2HSV)
    lowerLimit = np.uint8([hsvColor[0][0][0]-10, lowerSaturation,lowerValue])
    upperLimit = np.uint8([hsvColor[0][0][0]+10, upperSaturation, upperValue])
    return lowerLimit, upperLimit

def getFilteredColorMask(hsvImage, lowerColorLimit ,upperColorLimit, useMorphology=True) :
    mask = cv2.inRange(hsvImage, lowerColorLimit, upperColorLimit)
    if useMorphology :
	    kernel = np.ones((4,4),np.uint8)
	    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
	    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask

def getAreaSortedContours(mask) :
	maxArea = 0.0
	edged = cv2.Canny(mask, 100, 200)
	contours = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
	contours = imutils.grab_contours(contours)
	contoursSorted = sorted(contours, key=cv2.contourArea, reverse=True)
	# cv2.imshow("edged: ", edged)
	if len(contoursSorted) > 1 :
		contoursSorted = contoursSorted[:1]
	# for i in range(len(contoursSorted)):
	# 	maxArea = cv2.contourArea(contoursSorted[0])
		# print(a)
		# print(cv2.contourArea(contoursSorted[0]))
		# print(i, cv2.contourArea(contoursSorted[i]))
	# print("utils.py contours len: ", len(contoursSorted))
	# a = cv2.contourArea(contoursSorted[0])
	# maxArea = cv2.contourArea(contoursSorted[0])
	# if contours != None and maxArea > 0.5:
	return contoursSorted

def drawBoxes(image, boxCoordinates) :
	if len(boxCoordinates) > 0:
		for (startX, startY, endX, endY) in boxCoordinates:
			cv2.rectangle(image, (startX, startY), (endX, endY), (0, 255, 0), 2)

def getBoundingBoxes(contours) :
	rects = []
	if contours != None and len(contours) != 0 :
		for contour in contours:
			x,y,w,h = cv2.boundingRect(contour)
			rects.append([x,y,x+w,y+h])
			rects = np.array(rects)
			# return non_max_suppression(rects, probs=None, overlapThresh=0.45)
			return rects
	else :
		# print("No contours found...")
		return []

def drawObjectCoordinates(image, objects) :
	if bool(objects) :
		for (objectID, centroid) in objects.items():
			centerX, centerY, area, width = centroid
			text = "ID {}".format(objectID)
			cv2.putText(image, text, (centerX - 10, centerY - 10),
			cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
			cv2.circle(image, (centerX, centerY), 4, (0, 255, 0), -1)

def findCenterOfBiggestBox(objects) :
	maxAreaBboxID = 0
	prevArea = 0
	for (objectID, centroid) in objects.items():
		centerX, centerY, area, width = centroid
		# if area > 100 :
		if prevArea < area :
			maxAreaBboxID = objectID
			prevArea = area
	if maxAreaBboxID in objects :
		trackedCentroid = objects[maxAreaBboxID]
		return maxAreaBboxID, trackedCentroid[0]


# class Path():
# 	def __init__(self, path=1) :
# 		self.pink = 1
# 		self.yellow = 2
# 		self.blue = 3
# 		self.red = 4
# 		# self.defaultPath = self.pink
# 		self.path = []

# 	def setPath(self, path) :
# 		self.path.clear()
# 		for color in path :
# 			self.addToPath(color)

# 	def addToPath(self, pathColor) :
# 		self.path.append(pathColor)

# 	def printPath(self) :
# 		print(self.path)
