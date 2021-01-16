import numpy as np
import config as cfg
from time import sleep, time

REAL_DISTANCE = 23 # cm
REAL_WIDTH = 12 # cm
PIXEL_WIDTH_AT_REAL_DIST = 257 # pixels
FOCAL_LENGTH = (PIXEL_WIDTH_AT_REAL_DIST * REAL_DISTANCE) / REAL_WIDTH

##
## Finds distance to an object in centimeters
##
## :param      pixelWidth:  The pixel width
##
## :returns:   { real distance to an object }
##
def findCameraDistance(pixelWidth) :
	return (REAL_WIDTH * FOCAL_LENGTH) / pixelWidth

def movingAverage(measurement) :
	cfg.distMeasurements.append(measurement)
	if len(cfg.distMeasurements) > cfg.movingAverageWindowSize :
		cfg.distMeasurements = cfg.distMeasurements[1:len(cfg.distMeasurements)]
	return np.average(cfg.distMeasurements)

def positionControl_PID() :

	# global previousError, integralError, loopFreq, measurement, setpoint, Kp, Kd, Ki, threadStopper

	loopPeriod = 1 / cfg.loopFreq

	try :
		# print("begin: ", time())
		# print("isthreadstopped: ", cfg.threadStopper.is_set())
		while not cfg.threadStopper.is_set() :
			start = time()
			preverror = cfg.previousError # save old error
			integral_error = cfg.integralError

			error = cfg.setpoint - cfg.measurement #calculating current error
			previousError = error
			proportional_output = cfg.Kp * error #Output of proportional controller

			diff_error = (error-preverror) #dividing by timestep is not necessary since this can be compensated for by tuning Kd 

			diff_output = cfg.Kd * diff_error #output of differential controller

			if cfg.Ki < 0.0001 and cfg.Ki > -0.0001 :
				integral_error = 0.0 #calculating the integral, again timestep is not necessary
			else :
				integral_error += error

			integral_output = cfg.Ki * integral_error  #integral controller output

			cfg.correction = proportional_output + diff_output + integral_output #check if the output needs to be negative or not
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

	# 	print("somerror: ", str(err))
		
# MAX_SPEED = 100
# TIME_STEP = 1/30

# def positionControl_PID(previousError, integralError, loopFreq, measurement, setpoint, Kp, Kd, Ki) :

# 	preverror = previousError # save old error
# 	integral_error = integralError

# 	error = setpoint - measurement #calculating current error
# 	proportional_output = Kp * error #Output of proportional controller

# 	diff_error = (error-preverror)/TIME_STEP #dividing by timestep is not necessary since this can be compensated for by tuning Kd 

# 	diff_output = Kd * diff_error #output of differential controller

# 	integral_error = (integral_error + error*TIME_STEP) #calculating the integral, again timestep is not necessary
# 	integral_output = Ki * integral_error  #integral controller output

# 	correction = int(proportional_output + diff_output + integral_output) #check if the output needs to be negative or not
# 	return error, integral_error, correction
	# print("PID OUTPUT: ", output, "ERROR: ", error, "proportional_output", proportional_output, "diff_output: ", diff_output, "integral_output: ", integral_output)

	# if output < 0 :
	# 	if output < -MAX_SPEED :
	# 		output = 100
	# 	else:
	# 		output = np.abs(output)
	# 	return error, integral_error, MAX_SPEED, output
	# else :
	# 	if output > MAX_SPEED :
	# 		output = 100
	# 	else:
	# 		output = np.abs(output)
	# 	return error, integral_error, output, MAX_SPEED


# stopper = Event()
# stepSize = 0.1
# loopFreq = 30.0
# setPoint = 0.5
# motorSpeed = 300
# leftMotorSpeed = 0
# rightMotorSpeed = 0
# stopMotors = True
# Kp = 0.0
# Ki = 0.0
# Kd = 0.0

# integralArea = 0.0


# def controller():
#     global stopper, gpg, lf, stepSize, loopFreq, setPoint, motorSpeed, leftMotorSpeed, rightMotorSpeed,stopMotors, Kp, Ki, Kd
#     global integralArea
#     loopPeriod = 1 / loopFreq
    
#     integralArea = 0.0
#     previousError = 0.0

#     try:
#         while not stopper.is_set():
#             start = time()

#             # <0.5 when line is on the left
#             # >0.5 when line is on the right
#             current, _ = lf.read('weighted-avg')

#             # calculate correction
#             error = current - setPoint
#             if Ki < 0.0001 and Ki > -0.0001:
#                 integralArea = 0.0
#             else:
#                 integralArea += error
#             correction = Kp * error + Ki * integralArea + Kd * (error - previousError) 
#             # print(Kp * error, Ki * integralArea, Kd * (error - previousError))
#             previousError = error

#             # calculate motor speedss
#             leftMotorSpeed = int(motorSpeed + correction)
#             rightMotorSpeed = int(motorSpeed - correction)

#             if leftMotorSpeed == 0: leftMotorSpeed = 1
#             if rightMotorSpeed == 0: rightMotorSpeed = 1
#             # if leftMotorSpeed >= 300: leftMotorSpeed = 299
#             # if rightMotorSpeed >= 300: rightMotorSpeed = 299

#             # update motor speeds
#             if stopMotors is False:
#                 gpg.set_motor_dps(gpg.MOTOR_LEFT, dps=leftMotorSpeed)
#                 gpg.set_motor_dps(gpg.MOTOR_RIGHT, dps=rightMotorSpeed)

#             # make the loop work at a given frequency
#             end = time()
#             delayDiff = end - start
#             if loopPeriod - delayDiff > 0:
#                 sleep(loopPeriod - delayDiff)
#     except Exception as err:
#         print(str(err))
#         stopper.set()
#     finally:
#         gpg.stop()   