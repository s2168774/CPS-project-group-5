from lineFollower import lineFollower
from peopleDetection import personFollower
import config as cfg

mode = 1# getBluetoothCommand()

MODE_LINE_FOLLOWER = 0
MODE_PERSON_FOLLOWER = 1


if mode == MODE_LINE_FOLLOWER :
	cfg.horizontal_Kp =0.23 #0.23
	cfg.horizontal_Kd = 12.0#12.0
	cfg.horizontal_Ki = 0.0001#0.000022
	lineFollower()
elif mode == MODE_PERSON_FOLLOWER :
	print("START following person")
	# parameters of the PID controller
	cfg.MAX_SPEED = 400
	cfg.horizontal_Kp = 0.24
	cfg.horizontal_Kd = 0.3
	cfg.horizontal_Ki = 0.0#02#001#0.000022
	cfg.distance_setpoint = 60
	cfg.distance_Kp = 4.0
	cfg.distance_Kd = 8.0
	cfg.distance_Ki = 0.0#1
	personFollower()
else :
	print("Avilable modes:")
	print("\tLINE FOLLOWER (MODE_LINE_FOLLOWER = 0")
	print("\tLINE FOLLOWER (MODE_PERSON_FOLLOWER = 1")
