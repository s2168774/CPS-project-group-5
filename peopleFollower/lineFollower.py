import cv2
import numpy as np 
import imutils
import time
import easygopigo3
import config as cfg
import bleScanner as ble

# from bleCommunication.bleScanner import DeviceScanner
from threading import Thread
from collections import OrderedDict
from picamera.array import PiRGBArray

from utils import findCameraDistance, horizontalPositionControl_PID, distanceControl_PID, findRssiDistance, rectArea, cameraInit, movingAverage, getColorLimitsFromBGR, getFilteredColorMask, getAreaSortedContours, drawBoxes, getBoundingBoxes, drawObjectCoordinates, findCenterOfBiggestBox

from centroidTracker import CentroidTracker

from wifiScanner import WifiScanner



class LineFollower() :

    def __init__(self) :
        # self.currentPath = np.array(cfg.color_PINK)
        self.currentColors = []
        self.step = 0
        self.path = []
        self.isNextStep = False

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

        # def nothing(x):
        #     pass
         
        # cv2.namedWindow("Trackbars")
         
        # cv2.createTrackbar("B", "Trackbars", 0, 255, nothing)
        # cv2.createTrackbar("G", "Trackbars", 0, 255, nothing)
        # cv2.createTrackbar("R", "Trackbars", 0, 255, nothing)
        # objectTrackers = []
        # for x in range(4):
        #     objectTrackers.append(CentroidTracker(maxDisappeared=5))
        #     pass

        # coloredObjects = OrderedDict()
        # for x in range(4):
        #     coloredObjects[x] = OrderedDict()
        #     pass
        # colorLimits = np.zeros((4, 3), dtype="uint8")


        #loop through frames continuously
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            image = frame.array
            startTime = time.time()

            blurred = cv2.GaussianBlur(image, (5, 5), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV) # convert picture from BGR to HSV color format
            # hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # convert picture 

            # for i in range(len(coloredObjects)):
            #     coloredObjects[self.currentColors[i]] = objectTrackers[self.currentColors[i]].update()
                # find pink line


            # B = 50  #blue 0..255
            # G = 10  #green 0..255
            # R = 200 #red 0..255

            # Optional trackbars left for determining threshold 'live' if current is not working
            # B = cv2.getTrackbarPos("B", "Trackbars")
            # G = cv2.getTrackbarPos("G", "Trackbars")
            # R = cv2.getTrackbarPos("R", "Trackbars")
            # lowerLimitPink, upperLimitPink = getColorLimitsFromBGR(50, 10, 200)
            # lowerLimitYellow, upperLimitYellow = getColorLimitsFromBGR(77, 147, 192)
            
            
            maskFirst = getFilteredColorMask(hsv, cfg.colorLimitsDict.lower(self.currentColors[0]), cfg.colorLimitsDict.upper(self.currentColors[0]))
            maskSecond = getFilteredColorMask(hsv, cfg.colorLimitsDict.lower(self.currentColors[1]), cfg.colorLimitsDict.upper(self.currentColors[1]))
            # Apply Filters END
            # cv2.imshow("mask pink", maskFirst)
            # pinkContoursSorted = getAreaSortedContours(maskPink)
            # yellowContoursSorted = getAreaSortedContours(maskYellow)

            firstContoursSorted = getAreaSortedContours(maskFirst)
            secondContoursSorted = getAreaSortedContours(maskSecond)
            # print("lineFollower.py: contours len: ", len(firstContoursSorted))

            # cv2.drawContours(image, firstContoursSorted, -1, (0, 255, 0), 3)
            # cv2.imshow("with contours", image)

            # cv2.imshow("contours pink", firstContoursSorted)

            # pinkBoundingBoxes = getBoundingBoxes(pinkContoursSorted)
            # yellowBoundingBoxes = getBoundingBoxes(yellowContoursSorted)
            firstBoundingBoxes = getBoundingBoxes(firstContoursSorted)
            secondBoundingBoxes = getBoundingBoxes(secondContoursSorted)

            drawBoxes(image, firstBoundingBoxes)
            drawBoxes(image, secondBoundingBoxes)
            firstColorObjects = firstColorTracker.update(firstBoundingBoxes)
            secondColorObjects = secondColorTracker.update(secondBoundingBoxes)
            

            drawObjectCoordinates(image, firstColorObjects)
            drawObjectCoordinates(image, secondColorObjects)

            #process pink Objects
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
             
                print("Both colors detected!!")

            elif bool(firstColorObjects) and not bool(secondColorObjects) :
                if self.isNextStep :
                    self.isNextStep = False
                _, firstColorCenterX = findCenterOfBiggestBox(firstColorObjects)
                cfg.horizontal_measurement = movingAverage(firstColorCenterX, cfg.horizontalPositions, windowSize=2) # horizontal position 
                cfg.GPG.set_motor_dps(cfg.GPG.MOTOR_LEFT, dps=cfg.MAX_SPEED - int(cfg.horizontal_correction))
                cfg.GPG.set_motor_dps(cfg.GPG.MOTOR_RIGHT, dps=cfg.MAX_SPEED + int(cfg.horizontal_correction))
            # elif bool(firstColorObjects) and bool(secondColorObjects) :
            #     self.step += 1
            #     cfg.GPG.set_motor_dps(cfg.GPG.MOTOR_LEFT, dps=0)
            #     cfg.GPG.set_motor_dps(cfg.GPG.MOTOR_RIGHT, dps=0)
            #     print("STOP!")
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

                # ###### HANDLE ROBOT MOVEMENT_ END #####
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

def main () :
    lineFollower()

if __name__== "__main__":
    main()
