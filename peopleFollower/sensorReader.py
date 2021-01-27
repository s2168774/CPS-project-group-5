import smbus
import time
import config as cfg
import math
import threading
import numpy as np
from utils import movingAverage


class SensorReader(threading.Thread) :
	
	def __init__(self, probingFreq=100) :
		self.probingFreq = probingFreq
		threading.Thread.__init__(self)

	def run(self) :
		self.readIMUData(self.probingFreq)
		# self.sensorThread(target=self.readIMUData, args=(probingFreq), deamon=True)
		# self.sensorThread.start()


	def readIMUData(self, probingFreq) :
		bus = smbus.SMBus(1)


		#Test MMA8452Q address after connecting it with (it shows in the table):
		#sudo i2cdetect -y 1
		#For Velleman VMA208 accelerometer it should be 0x1D, for others other addresses
		MMA8452Q= 0x1D
		#Setup:
		# Select Control register, 0x2A(42)
		#       0x00(00)    StandBy mode
		bus.write_byte_data(MMA8452Q, 0x2A, 0x00)
		# MMA8452Q address, 0x1C(28)
		# Select Control register, 0x2A(42)
		#       0x01(01)    Active mode
		bus.write_byte_data(MMA8452Q, 0x2A, 0x01)
		# MMA8452Q address, 0x1C(28)
		# Select Configuration register, 0x0E(14)
		#       0x00(00)    Set range to +/- 2g
		bus.write_byte_data(MMA8452Q, 0x0E, 0x00)
		time.sleep(0.1)
		xAccelerationMeasurementsArray = []
		zAccelerationMeasurementsArray = []
		xTiltCompensation = 0.0
		yTiltCompensation = 0.0
		zTiltCompensation = 0.0

		xCalibrationValues = []
		yCalibrationValues = []
		zCalibrationValues = []
		i = 0
		# calibration
		while len(xCalibrationValues) < 300 :
			data = bus.read_i2c_block_data(MMA8452Q, 0x00, 7)
			xAccl = (data[1] * 256 + data[2]) / 16
			if xAccl > 2047 :
				xAccl -= 4096

			yAccl = (data[3] * 256 + data[4]) / 16
			if yAccl > 2047 :
				yAccl -= 4096

			zAccl = (data[5] * 256 + data[6]) / 16
			if zAccl > 2047 :
				zAccl -= 4096

			# print("Acceleration in X-Axis : ", xAccl)
			# print("Acceleration in Y-Axis : ", yAccl)
			# print("Acceleration in Z-Axis : ", zAccl)

			xCalibrationValues.append(xAccl)
			yCalibrationValues.append(yAccl)
			zCalibrationValues.append(zAccl)
			i += 1
			print(i)

		xTiltCompensation = np.average(xCalibrationValues)
		yTiltCompensation = np.average(yCalibrationValues)
		zTiltCompensation = np.average(zCalibrationValues)
		# print("Tilt in X-Axis : ", xTiltCompensation)
		# print("Tilt in Y-Axis : ", yTiltCompensation)
		# print("Tilt in Z-Axis : ", zTiltCompensation)

		# try :
		while not cfg.threadStopper.is_set() :
		# MMA8452Q address, 0x1C(28)
		# Read data back from 0x00(0), 7 bytes
		# Status register, X-Axis MSB, X-Axis LSB, Y-Axis MSB, Y-Axis LSB, Z-Axis MSB, Z-Axis LSB
			data = bus.read_i2c_block_data(MMA8452Q, 0x00, 7)

		# Convert the data
			xAccl = (data[1] * 256 + data[2]) / 16
			if xAccl > 2047 :
				xAccl -= 4096

			yAccl = (data[3] * 256 + data[4]) / 16
			if yAccl > 2047 :
				yAccl -= 4096

			zAccl = (data[5] * 256 + data[6]) / 16
			if zAccl > 2047 :
				zAccl -= 4096

			# cfg.xAccl = movingAverage(xAccl, xAccelerationMeasurementsArray, windowSize=20) - xTiltCompensation
			# cfg.zAccl = movingAverage(zAccl, xAccelerationMeasurementsArray, windowSize=20) - zTiltCompensation
			cfg.xAccl = xAccl - xTiltCompensation
			cfg.zAccl = zAccl - zTiltCompensation

			spatialAccel = math.sqrt(math.pow(cfg.zAccl, 2.0) + math.pow(cfg.xAccl, 2.0))
			cfg.spatialVelocity += (1/probingFreq) * spatialAccel
			# time.sleep(1/probingFreq)		
			# print("Acceleration in X-Axis : ", cfg.xAccl, "before calibration: ", xAccl, " : ", xTiltCompensation)
			# print("Acceleration in Y-Axis : ", cfg.yAccl, "before calibration: ", yAccl)
			# print("Acceleration in Z-Axis : ", cfg.zAccl, "before calibration: ", zAccl)
			# print("spatialVelocity : ", cfg.spatialVelocity)
		# # except Exception as err:
		# 	print(str(err))
		# 	cfg.threadStopper.set()
		# finally:
		# 	cfg.GPG.stop()

		# Output data to screen
		# print("Acceleration in X-Axis ", cfg.xAccl)
		#print("Acceleration in Y-Axis : %d", yAccl)
		#print("Acceleration in Z-Axis : %d", zAccl)