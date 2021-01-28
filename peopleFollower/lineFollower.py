import cv2
import numpy as np 
import imutils
import time
import easygopigo3
import config as cfg
import bleScanner as ble
import sensorReader

# from bleCommunication.bleScanner import DeviceScanner
from threading import Thread
from collections import OrderedDict
from picamera.array import PiRGBArray

from utils import findCameraDistance, horizontalPositionControl_PID, distanceControl_PID, findRssiDistance, rectArea, cameraInit, movingAverage, getHSVColorLimitsFromBGR, getFilteredColorMask, getAreaSortedContours, drawBoxes, getBoundingBoxes, drawObjectCoordinates, findCenterOfBiggestBox

from centroidTracker import CentroidTracker

from wifiScanner import WifiScanner

class LineFollower() :

    def __init__(self) :
        # self.currentPath = np.array(cfg.color_PINK)
        self.currentColors = []
        self.step = 0
        self.path = []
        self.isNextStep = False
        self.isNearCrossroads = False
        self.markerPresentFrameCount = 0
        self.markerPresentFrameLimit = 3
        self.sensorReader = sensorReader.SensorReader(probingFreq=100)

    def updateStep(self) :
        print("len(self.path:", len(self.path))
        if len(self.path) > 2 and self.step < len(self.path) - 2 :
            self.step += 1
            self.currentColors = np.array(self.path[self.step:self.step+2])
            print(".currentColors: ", self.currentColors, "path", self.path)
            return True
        else :
            return False

    def run(self, path=[cfg.color_PINK, cfg.color_YELLOW]) :
        self.path = path
        self.currentColors = path[:2]
        # Initialize PiCamera
        camera = cameraInit()

        #initialize peopleTracker
        firstColorTracker = CentroidTracker(maxDisappeared=8)
        secondColorTracker = CentroidTracker(maxDisappeared=14)
        centroidX = 0
        width = 1
        # objects = OrderedDict()

        # Create and start PID controller thread
        horizontalPositionControlThread = Thread(target = horizontalPositionControl_PID)
        horizontalPositionControlThread.start()
        print("horizontal control thread started")

        rawCapture = PiRGBArray(camera, size=(cfg.FRAME_WIDTH, cfg.FRAME_HEIGHT))

        self.sensorReader.start()
        def nothing(x):
            pass
         

        #loop through frames continuously
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            image = frame.array
            startTime = time.time()
            
            print(cfg.mode)

            blurred = cv2.GaussianBlur(image, (5, 5), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV) # convert picture

            
            
            maskFirst = getFilteredColorMask(hsv, cfg.colorLimitsDict.lower(self.currentColors[0]), cfg.colorLimitsDict.upper(self.currentColors[0]), useMorphology=False)
            firstContoursSorted = getAreaSortedContours(maskFirst)
            firstBoundingBoxes = getBoundingBoxes(firstContoursSorted)
            drawBoxes(image, firstBoundingBoxes)
            firstColorObjects = firstColorTracker.update(firstBoundingBoxes)
            drawObjectCoordinates(image, firstColorObjects)

            maskSecond = getFilteredColorMask(hsv, cfg.colorLimitsDict.lower(self.currentColors[1]), cfg.colorLimitsDict.upper(self.currentColors[1]), useMorphology=False)
            secondContoursSorted = getAreaSortedContours(maskSecond)
            secondBoundingBoxes = getBoundingBoxes(secondContoursSorted)
            drawBoxes(image, secondBoundingBoxes)
            secondColorObjects = secondColorTracker.update(secondBoundingBoxes)
            drawObjectCoordinates(image, secondColorObjects)


            if not (self.step + 2 == len(self.path)):

                if bool(firstColorObjects) and bool(secondColorObjects) :
                    self.isNextStep = False
                    print("both colors seen")
                    _, secondColorCenterX = findCenterOfBiggestBox(secondColorObjects)
                    cfg.horizontal_measurement = movingAverage(secondColorCenterX, cfg.horizontalPositions, windowSize=2)

                    cfg.GPG.set_motor_dps(cfg.GPG.MOTOR_LEFT, dps=cfg.MAX_SPEED - int(cfg.horizontal_correction))
                    cfg.GPG.set_motor_dps(cfg.GPG.MOTOR_RIGHT, dps=cfg.MAX_SPEED + int(cfg.horizontal_correction))
                
                elif bool(firstColorObjects) :
                    print("first color seen")
                    self.isNextStep = False
                    _, firstColorCenterX = findCenterOfBiggestBox(firstColorObjects)
                    cfg.horizontal_measurement = movingAverage(firstColorCenterX, cfg.horizontalPositions, windowSize=2)

                    cfg.GPG.set_motor_dps(cfg.GPG.MOTOR_LEFT, dps=cfg.MAX_SPEED - int(cfg.horizontal_correction))
                    cfg.GPG.set_motor_dps(cfg.GPG.MOTOR_RIGHT, dps=cfg.MAX_SPEED + int(cfg.horizontal_correction))

                elif bool(secondColorObjects) :
                    print("second color seen")
                    if not self.isNextStep :
                        self.isNextStep = self.updateStep()

                else :
                    cfg.GPG.set_motor_dps(cfg.GPG.MOTOR_LEFT, dps=1)
                    cfg.GPG.set_motor_dps(cfg.GPG.MOTOR_RIGHT, dps=1)
                    print("STOP!")
            else:
                print("last step")
                _, secondColorCenterX = findCenterOfBiggestBox(secondColorObjects)
                cfg.horizontal_measurement = movingAverage(secondColorCenterX, cfg.horizontalPositions, windowSize=2)


                cv2.imshow("outputImage", image)
                endTime = time.time()
                # print("loopTime: ", endTime - startTime)
                # Exit if 'esc' is clicked
                # cleanup hardware
                key = cv2.waitKey(1)
                rawCapture.truncate(0)
                if key == 27:
                    cfg.threadStopper.set()
                    horizontalPositionControlThread.join()
                    # distanceControlThread.join()
                    # deviceScannerThread.join()
                    cfg.GPG.reset_all()
                    camera.close()
                    cfg.GPG.stop()
                    break


        cv2.destroyAllWindows()

def findMarker() :

    return True

def main () :
    lineFollower()

if __name__== "__main__":
    main()

