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
        self.isNearCrossroads = True
        self.markerPresentFrameCount = 0
        self.markerPresentFrameLimit = 7
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
        # print("path: ", path)
        # print("###############################")

        # Initialize PiCamera
        camera = cameraInit()

        #initialize peopleTracker
        firstColorTracker = CentroidTracker(maxDisappeared=20)
        secondColorTracker = CentroidTracker(maxDisappeared=20)
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
         
        # cv2.namedWindow("Trackbars")
         
        # cv2.createTrackbar("B", "Trackbars", 0, 255, nothing)
        # cv2.createTrackbar("G", "Trackbars", 0, 255, nothing)
        # cv2.createTrackbar("R", "Trackbars", 0, 255, nothing)

        #loop through frames continuously
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            image = frame.array
            startTime = time.time()

            blurred = cv2.GaussianBlur(image, (5, 5), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV) # convert picture

             # blurred = cv2.GaussianBlur(image, (1, 1), 0)
            # hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # convert picture
            # B = 50  #blue 0..255
            # G = 10  #green 0..255
            # R = 200 #red 0..255

            # Optional trackbars left for determining threshold 'live' if current is not working
            # B = cv2.getTrackbarPos("B", "Trackbars")
            # G = cv2.getTrackbarPos("G", "Trackbars")
            # R = cv2.getTrackbarPos("R", "Trackbars")

            
            
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
            # drawObjectCoordinates(image, secondColorObjects)

            # lowerLimitMarker, upperLimitMarker = getHSVColorLimitsFromBGR(B,G,R)

            # # maskMarker = getFilteredColorMask(hsv, cfg.colorLimitsDict.lower(cfg.color_MARKER), cfg.colorLimitsDict.upper(cfg.color_MARKER))
            # maskMarker = getFilteredColorMask(hsv, lowerLimitMarker, upperLimitMarker)
            # cv2.imshow("mask", maskMarker)
            # markerContoursSorted = getAreaSortedContours(maskMarker)
            # markerBoundingBoxes = getBoundingBoxes(markerContoursSorted)
            # drawBoxes(image, markerBoundingBoxes)


            # if len(markerBoundingBoxes) != 0 :
            #     self.markerPresentFrameCount += 1

            #     if self. markerPresentFrameCount >= self.markerPresentFrameLimit :
            #         self.isNearCrossroads = True

            #     ## broadcast info that marker is reached
            #     ## check if you received info from other cars that reached the intersection
            #     ## compare speeds of cars and stop one of the cars
            #     print("I AM ON INTERSECTION, WATCH OUT!")

            if bool(firstColorObjects) and bool(secondColorObjects) :
                if not self.isNextStep :
                    self.isNextStep = self.updateStep()
                    _, secondColorCenterX = findCenterOfBiggestBox(secondColorObjects)
                    cfg.horizontal_measurement = movingAverage(secondColorCenterX, cfg.horizontalPositions, windowSize=2) # horizontal 
                else:
                    _, firstColorCenterX = findCenterOfBiggestBox(firstColorObjects)
                    cfg.horizontal_measurement = movingAverage(firstColorCenterX, cfg.horizontalPositions, windowSize=2) # horizontal 
                
                cfg.GPG.set_motor_dps(cfg.GPG.MOTOR_LEFT, dps=int(cfg.MAX_SPEED /2) - int(cfg.horizontal_correction))
                cfg.GPG.set_motor_dps(cfg.GPG.MOTOR_RIGHT, dps=int(cfg.MAX_SPEED / 2) + int(cfg.horizontal_correction))
             
                # print("Both colors detected!!")

            elif bool(firstColorObjects) and not bool(secondColorObjects) :
                if self.isNextStep :
                    self.isNextStep = False
                _, firstColorCenterX = findCenterOfBiggestBox(firstColorObjects)
                cfg.horizontal_measurement = movingAverage(firstColorCenterX, cfg.horizontalPositions, windowSize=2) # horizontal position 
                cfg.GPG.set_motor_dps(cfg.GPG.MOTOR_LEFT, dps=cfg.MAX_SPEED - int(cfg.horizontal_correction))
                cfg.GPG.set_motor_dps(cfg.GPG.MOTOR_RIGHT, dps=cfg.MAX_SPEED + int(cfg.horizontal_correction))
            elif bool(secondColorObjects) :
                _, secondColorCenterX = findCenterOfBiggestBox(secondColorObjects)
                cfg.horizontal_measurement = movingAverage(secondColorCenterX, cfg.horizontalPositions, windowSize=2) # horizontal position 
                cfg.GPG.set_motor_dps(cfg.GPG.MOTOR_LEFT, dps=cfg.MAX_SPEED - int(cfg.horizontal_correction))
                cfg.GPG.set_motor_dps(cfg.GPG.MOTOR_RIGHT, dps=cfg.MAX_SPEED + int(cfg.horizontal_correction))
                
            elif not bool(firstColorObjects) and not bool(secondColorObjects) :
                cfg.GPG.set_motor_dps(cfg.GPG.MOTOR_LEFT, dps=1)
                cfg.GPG.set_motor_dps(cfg.GPG.MOTOR_RIGHT, dps=1)
                print("STOP!")
            
            # print("distance correction: ", cfg.distance_correction)
            # print("horizontal_correction: ", cfg.horizontal_correction)

            cv2.imshow("outputImage", image)
            endTime = time.time()
            print("loopTime: ", endTime - startTime)
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

