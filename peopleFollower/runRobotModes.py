from lineFollower import LineFollower
from peopleDetection import personFollower
from threading import Thread
import config as cfg

mode = 1# getBluetoothCommand()

isLineFollowerRunning = False
isPeopleDetectionRunning = False
MODE_EXIT = 0
MODE_LINE_FOLLOWER = 1
MODE_PERSON_FOLLOWER = 2
STOP_CURRENT_MODE = 3

lineFollowerThread = Thread()
personFollowerThread = Thread()

lineFollower = LineFollower()

while True : #not cfg.threadStopper.is_set() :
##
## SCAN FOR BLUETOOTH INPUT
##
	if mode == MODE_LINE_FOLLOWER and not isLineFollowerRunning :
		myPath = [cfg.color_PINK, cfg.color_YELLOW, cfg.color_PINK, cfg.color_YELLOW, cfg.color_PINK]
		print("START line following")
		cfg.horizontal_loopFreq = 8
		cfg.MAX_SPEED = 150
		cfg.horizontal_Kp =0.15 #0.23
		cfg.horizontal_Kd = 0.3#12.0
		cfg.horizontal_Ki = 0.0001#0.000022
		isLineFollowerRunning=True
		lineFollowerThread(target=lineFollower.run(myPath))
		lineFollowerThread.start()
	elif mode == MODE_PERSON_FOLLOWER and not isPeopleDetectionRunning:
		print("START following person")
		# parameters of the PID controller
		cfg.horizontal_loopFreq = 9
		cfg.MAX_SPEED = 400
		cfg.horizontal_Kp = 0.7
		cfg.horizontal_Kd = 0.7
		cfg.horizontal_Ki = 0.0001#02#001#0.000022
		cfg.distance_setpoint = 130
		cfg.distance_Kp = 8.0
		cfg.distance_Kd = 12.0
		cfg.distance_Ki = 0.0#1
		personFollowerThread(target=personFollower())
		personFollowerThread.start()
	elif mode == STOP_CURRENT_MODE:
		cfg.threadStopper.set()
		isLineFollowerRunning = False
		isPeopleDetectionRunning = False
		personFollowerThread.join()
		lineFollowerThread.join()
	elif mode == MODE_EXIT:
		cfg.threadStopper.set()
		if personFollowerThread.is_alive() :
			personFollowerThread.join()
		if lineFollowerThread.is_alive() :
			lineFollowerThread.join()
		break;
	else :
		print("Avilable modes:")
		print("\tLINE FOLLOWER (MODE_LINE_FOLLOWER = 0")
		print("\tLINE FOLLOWER (MODE_PERSON_FOLLOWER = 1")
