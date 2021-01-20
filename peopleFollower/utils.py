import numpy as np
import config as cfg
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
	measurementsArray.append(measurement)
	arrayLength = len(measurementsArray)
	if arrayLength > windowSize :
		# print("windowsize: ", windowSize, arrayLength)
		measurementsArray = measurementsArray[arrayLength - windowSize - 1: arrayLength - 1]
		# print(measurementsArray)
		# print ("NEEEEEEEEEEEEEEEEEEEEEEXT")
	return np.average(measurementsArray)

def horizontalPositionControl_PID() :

	# global previousError, integralError, loopFreq, measurement, setpoint, Kp, Kd, Ki, threadStopper

	loopPeriod = 1 / cfg.horizontal_loopFreq

	try :
		# print("begin: ", time())
		# print("isthreadstopped: ", cfg.threadStopper.is_set())
		while not cfg.threadStopper.is_set() :
			start = time()

			preverror = cfg.horizontal_previousError # save old error
			# integral_error = cfg.horizontal_integralError

			error = cfg.horizontal_setpoint - cfg.horizontal_measurement #calculating current error
			cfg.horizontal_previousError = error
			proportional_output = cfg.horizontal_Kp * error #Output of proportional controller

			diff_error = (error-preverror) #dividing by timestep is not necessary since this can be compensated for by tuning Kd 

			diff_output = cfg.horizontal_Kd * diff_error #output of differential controller

			if cfg.horizontal_Ki < 0.00001 and cfg.horizontal_Ki > -0.00001 :
				cfg.horizontal_integralError = 0.0 #calculating the integral, again timestep is not necessary
			else :
				cfg.horizontal_integralError += error

			integral_output = cfg.horizontal_Ki * cfg.horizontal_integralError  #integral controller output

			cfg.horizontal_correction = proportional_output + diff_output + integral_output #check if the output needs to be negative or not
			# print("proportional out: %.3f, differential out: %.3f, integral out: %.3f" % (proportional_output, diff_output, integral_output))
			# print(" IN UTILS:: correction: ", cfg.correction, "cfg.measurement: ", cfg.measurement, "cfg.setpoint", cfg.setpoint)
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
			cfg.distance_previousError = error
			proportional_output = cfg.distance_Kp * error #Output of proportional controller

			diff_error = (error-preverror) #dividing by timestep is not necessary since this can be compensated for by tuning Kd 

			diff_output = cfg.distance_Kd * diff_error #output of differential controller

			if cfg.distance_Ki < 0.0001 and cfg.distance_Ki > -0.0001 :
				cfg.distance_integralError = 0.0 #calculating the integral, again timestep is not necessary
			else :
				cfg.distance_integralError += error

			integral_output = cfg.distance_Ki * cfg.distance_integralError  #integral controller output

			cfg.distance_correction = proportional_output + diff_output + integral_output #check if the output needs to be negative or not
			# print(" IN UTILS:: correction: ", cfg.correction, "cfg.measurement: ", cfg.measurement, "cfg.setpoint", cfg.setpoint)
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


# Initialization methods

def cameraInit() :
    camera = PiCamera()
    camera.resolution = (cfg.FRAME_WIDTH, cfg.FRAME_HEIGHT)
    camera.framerate = 20
    camera.rotation = 180
    return camera

