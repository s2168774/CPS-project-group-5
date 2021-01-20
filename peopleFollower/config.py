from threading import Thread, Event
import easygopigo3

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

MAX_SPEED = 300

# containers for moving average
distanceMeasurements = []
horizontalPositions = []
bBoxWidths = []

## horizontal position PID controller parameters START ##
horizontal_previousError = 0.0
horizontal_integralError = 0.0
horizontal_loopFreq = 100
horizontal_measurement =  FRAME_WIDTH / 2
horizontal_setpoint = FRAME_WIDTH / 2
# LeftMotorSpeed = 0
# rightMotorSpeed = 0
horizontal_Kp = 0.3
horizontal_Kd = 0.2
horizontal_Ki = 0.002
horizontal_correction = 0.0
## horizontal position PID controller parameters END ##

## distance PID controller parameters START ##
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
## distance PID controller parameters END ##


## Distance from RSSI parameters START ##
scanTime = float("inf")

rssi = 0
## Distance from RSSI parameters END ##


threadStopper = Event()

GPG = easygopigo3.EasyGoPiGo3() # Create an instance of the GoPiGo3 class. GPG will be the GoPiGo3 object.
GPG.set_speed(MAX_SPEED)