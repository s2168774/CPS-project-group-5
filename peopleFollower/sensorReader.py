import smbus
import time
import config as cfg

class sensorReader() :
	
	def __init__() :


	def readIMUData() :
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

		try :
			while not cfg.threadStopper.is_set() :
			# MMA8452Q address, 0x1C(28)
			# Read data back from 0x00(0), 7 bytes
			# Status register, X-Axis MSB, X-Axis LSB, Y-Axis MSB, Y-Axis LSB, Z-Axis MSB, Z-Axis LSB
			    data = bus.read_i2c_block_data(MMA8452Q, 0x00, 7)

			# Convert the data
			    cfg.xAccl = (data[1] * 256 + data[2]) / 16
			    if cfg.xAccl > 2047 :
			        cfg.xAccl -= 4096

			    cfg.yAccl = (data[3] * 256 + data[4]) / 16
			    if cfg.yAccl > 2047 :
			        cfg.yAccl -= 4096

			    cfg.zAccl = (data[5] * 256 + data[6]) / 16
			    if cfg.zAccl > 2047 :
			        cfg.zAccl -= 4096
		except Exception as err:
			print(str(err))
			cfg.threadStopper.set()
		finally:
			cfg.GPG.stop()

		# Output data to screen
		# print("Acceleration in X-Axis ", xAccl)
		#print("Acceleration in Y-Axis : %d", yAccl)
		#print("Acceleration in Z-Axis : %d", zAccl)