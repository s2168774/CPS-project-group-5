from threading import Thread, Event
import easygopigo3

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

MAX_SPEED = 600

movingAverageWindowSize = 10
distMeasurements = []

# PID controller parameters START
previousError = 0.0
integralError = 0.0
loopFreq = 40
measurement = 0
setpoint = int(FRAME_WIDTH -  FRAME_WIDTH / 6)
leftMotorSpeed = 0
rightMotorSpeed = 0
Kp = 0.2
Kd = 0.01
Ki = 0.1
correction = 0.0
# PID controller parameters END

threadStopper = Event()

GPG = easygopigo3.EasyGoPiGo3() # Create an instance of the GoPiGo3 class. GPG will be the GoPiGo3 object.
GPG.set_speed(MAX_SPEED)