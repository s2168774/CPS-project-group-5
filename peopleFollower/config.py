from threading import Thread, Event
from utils import getColorLimitsFromBGR
from collections import OrderedDict
import easygopigo3

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

MAX_SPEED = 200 #150

# containers for moving average
distanceMeasurements = []
horizontalPositions = []
bBoxWidths = []

##
## Horizontal position PID controller parameters START ##
##
horizontal_previousError = 0.0
horizontal_integralError = 0.0
horizontal_loopFreq = 9
horizontal_measurement =  FRAME_WIDTH / 2
horizontal_setpoint = FRAME_WIDTH / 2
# LeftMotorSpeed = 0
# rightMotorSpeed = 0
horizontal_Kp =0.23 #0.23
horizontal_Kd = 12.0#12.0
horizontal_Ki = 0.0001#0.000022
horizontal_correction = 0.0
##
## Horizontal position PID controller parameters END ##
##

##
## Distance PID controller parameters START ##
##
distance_previousError = 0.0
distance_integralError = 0.0
distance_loopFreq = 40
distance_measurement = 0
distance_setpoint = 100
# leftMotorSpeed = 0
# rightMotorSpeed = 0
distance_Kp = 10
distance_Kd = 0.5
distance_Ki = 0.0#1
distance_correction = 0.0
##
## Distance PID controller parameters END ##
##

##
## Line follower params
##
color_PINK = 0 
color_YELLOW = 1
color_BLUE = 2
color_RED = 3

##
## Color limits BGR
##

lowerLimitPink, upperLimitPink = getColorLimitsFromBGR(50, 10, 200)
lowerLimitYellow, upperLimitYellow = getColorLimitsFromBGR(77, 147, 192)
lowerLimitBlue, upperLimitBlue = getColorLimitsFromBGR(200, 0, 0)
lowerLimitRed, upperLimitRed = getColorLimitsFromBGR(0, 0, 200)

class ColorLimitsDict() :
	def __init__(self) :
		self.colorLimitsDict = OrderedDict()
		self.colorLimitsDict[color_PINK] = [lowerLimitPink, upperLimitPink]
		self.colorLimitsDict[color_YELLOW] = [lowerLimitYellow, upperLimitYellow]
		self.colorLimitsDict[color_BLUE] = [lowerLimitBlue, upperLimitBlue]
		self.colorLimitsDict[color_RED] = [lowerLimitRed, upperLimitRed]
		
	def upper(self, color) :
		return self.colorLimitsDict[color][1]

	def lower(self, color) :
		return self.colorLimitsDict[color][0]

colorLimitsDict = ColorLimitsDict()
## Distance from RSSI parameters START ##
scanTime = float("inf")

rssi = 0
## Distance from RSSI parameters END ##


threadStopper = Event()

GPG = easygopigo3.EasyGoPiGo3() # Create an instance of the GoPiGo3 class. GPG will be the GoPiGo3 object.
GPG.set_speed(MAX_SPEED)