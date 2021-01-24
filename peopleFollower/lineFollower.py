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



def lineFollower() :

    # Initialize PiCamera
    camera = cameraInit()

    #initialize peopleTracker
    yellowTracker = CentroidTracker(maxDisappeared=5)
    pinkTracker = CentroidTracker(maxDisappeared=5)
    centroidX = 0
    width = 1
    objects = OrderedDict()

    # Create and start PID controller thread
    horizontalPositionControlThread = Thread(target = horizontalPositionControl_PID)
    horizontalPositionControlThread.start()
    print("horizontal control thread started")

    rawCapture = PiRGBArray(camera, size=(cfg.FRAME_WIDTH, cfg.FRAME_HEIGHT))

    def nothing(x):
        pass
     
    # cv2.namedWindow("Trackbars")
     
    # cv2.createTrackbar("B", "Trackbars", 0, 255, nothing)
    # cv2.createTrackbar("G", "Trackbars", 0, 255, nothing)
    # cv2.createTrackbar("R", "Trackbars", 0, 255, nothing)

    print ("TRACKBARS CREATED")
    #loop through frames continuously
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        startTime = time.time()

        # search for another color

        blurred = cv2.GaussianBlur(image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV) # convert picture from BGR to HSV color format

        # B = 50  #blue 0..255
        # G = 10  #green 0..255
        # R = 200 #red 0..255

        # Optional trackbars left for determining threshold 'live' if current is not working
        # B = cv2.getTrackbarPos("B", "Trackbars")
        # G = cv2.getTrackbarPos("G", "Trackbars")
        # R = cv2.getTrackbarPos("R", "Trackbars")
        lowerLimitPink, upperLimitPink = getColorLimitsFromBGR(50, 10, 200)
        lowerLimitYellow, upperLimitYellow = getColorLimitsFromBGR(77, 147, 192)

        maskPink = getFilteredColorMask(hsv, lowerLimitPink, upperLimitPink)
        maskYellow = getFilteredColorMask(hsv, lowerLimitYellow, upperLimitYellow)
        # Apply Filters END

        pinkContoursSorted = getAreaSortedContours(maskPink)
        yellowContoursSorted = getAreaSortedContours(maskYellow)

        pinkBoundingBoxes = getBoundingBoxes(pinkContoursSorted)
        yellowBoundingBoxes = getBoundingBoxes(yellowContoursSorted)

        drawBoxes(image, pinkBoundingBoxes)
        drawBoxes(image, yellowBoundingBoxes)
        pinkObjects = pinkTracker.update(pinkBoundingBoxes)
        yellowObjects = yellowTracker.update(yellowBoundingBoxes)

        #process pink Objects
        if bool(pinkObjects):

            drawObjectCoordinates(image, pinkObjects)
            pinkCenterX = findCenterOfBiggestBox(pinkObjects)
            cfg.horizontal_measurement = movingAverage(pinkCenterX, cfg.horizontalPositions, windowSize=2) # horizontal position 

        if bool(yellowObjects):
            drawObjectCoordinates(image, yellowObjects)
            yellowCenterX = findCenterOfBiggestBox(yellowObjects) 
            cfg.horizontal_measurement = movingAverage(yellowCenterX, cfg.horizontalPositions, windowSize=2) # horizontal position 

        cfg.GPG.set_motor_dps(cfg.GPG.MOTOR_LEFT, dps=cfg.MAX_SPEED - int(cfg.horizontal_correction))
        cfg.GPG.set_motor_dps(cfg.GPG.MOTOR_RIGHT, dps=cfg.MAX_SPEED + int(cfg.horizontal_correction))
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