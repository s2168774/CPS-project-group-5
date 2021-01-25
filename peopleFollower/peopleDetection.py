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
from imutils.object_detection import non_max_suppression
from wifiScanner import WifiScanner
## TODO: 
def personFollower() :
    # Initialize PiCamera
    camera = cameraInit()

    #initialize peopleTracker
    peopleTracker = CentroidTracker(maxDisappeared=10)

    # def nothing(x):
    #     pass
     
    # cv2.namedWindow("Trackbars")
     
    # cv2.createTrackbar("B", "Trackbars", 0, 255, nothing)
    # cv2.createTrackbar("G", "Trackbars", 0, 255, nothing)
    # cv2.createTrackbar("R", "Trackbars", 0, 255, nothing)

    centroidX = 0
    width = 1
    objects = OrderedDict()

    # Create and start PID controller thread
    horizontalPositionControlThread = Thread(target = horizontalPositionControl_PID)
    horizontalPositionControlThread.start()
    print("horizontal control thread started")

    distanceControlThread = Thread(target = distanceControl_PID)
    distanceControlThread.start()
    print("distance control thread started")


    # # Handle Bluetooth Low Energy device scan
    # scanDelegate = ble.ScanDelegate()
    # deviceScanner = ble.DeviceScanner(scanDelegate)
    # # deviceScanner.startScan(float("inf"))
    # deviceScanner.showAvilableDevices(scanDelegate)
    # # TODO get values from scanner thread

    # wifiScanner = WifiScanner()

    # # deviceScannerThread = Thread(target = wifiScanner.probeRssi)
    # deviceScannerThread = Thread(target = deviceScanner.startScan)
    # deviceScannerThread.start()
    # print("device scanner thread started")

    rawCapture = PiRGBArray(camera, size=(cfg.FRAME_WIDTH, cfg.FRAME_HEIGHT))

    #loop through frames continuously
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        startTime = time.time()
        # print(" my device signal strength in dB: ", scanDelegate.rssi)
        # image = imutils.resize(image, width=min(400, image.shape[1]))

        # print(deviceScannerThread.is_alive())
        # Find object in the image
        
        blurred = cv2.GaussianBlur(image, (3, 3), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV) # convert picture from BGR to HSV color format

        # # Optional trackbars left for determining threshold 'live' if current is not working
        # B = cv2.getTrackbarPos("B", "Trackbars")
        # G = cv2.getTrackbarPos("G", "Trackbars")
        # R = cv2.getTrackbarPos("R", "Trackbars")
        # B = 50
        # G = 10
        # R = 180

        lowerLimit, upperLimit = getColorLimitsFromBGR(50, 10, 180)

        mask = getFilteredColorMask(hsv, lowerLimit, upperLimit)

        contoursSorted = getAreaSortedContours(mask)
        boundingBoxes = getBoundingBoxes(contoursSorted)

        drawBoxes(image, boundingBoxes)

        objects = peopleTracker.update(boundingBoxes)

        if bool(objects):
            maxAreaBboxID = 0
            prevArea = 0
            eraseIDs = []

            drawObjectCoordinates(image, objects)

            # # print("maxAreaBboxID: ", maxAreaBboxID)
            # if maxAreaBboxID in objects :
            #     trackedCentroid = objects[maxAreaBboxID]
            #     centroidX = trackedCentroid[0]
            #     width = movingAverage(trackedCentroid[3], cfg.bBoxWidths, windowSize = 4)
            #     print("width", width)
            biggestBoxID, centroidX = findCenterOfBiggestBox(objects)

            cfg.horizontal_measurement = centroidX#movingAverage(centroidX, 

            _, _, _, width = objects[biggestBoxID]
            cfg.distance_measurement = movingAverage(findCameraDistance(width), cfg.distanceMeasurements, windowSize=2)
            
            # cfg.distance_measurement = findCameraDistance(width)

            print(cfg.distance_measurement)

            cv2.putText(image, "%.2fcm" % (cfg.distance_measurement),
            (image.shape[1] - 200, image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)

            
            cfg.GPG.set_motor_dps(cfg.GPG.MOTOR_LEFT, dps=int(-cfg.distance_correction) - int(cfg.horizontal_correction))
            cfg.GPG.set_motor_dps(cfg.GPG.MOTOR_RIGHT, dps=int(-cfg.distance_correction) + int(cfg.horizontal_correction))

        cv2.imshow("outputImage", image)
        endTime = time.time()
        print("loopTime: ", endTime - startTime)
        # Exit if 'esc' is clicked
        # cleanup hardware
        key = cv2.waitKey(1)
        rawCapture.truncate(0)
        if key == 27:
            cfg.threadStopper.set()
            cfg.GPG.reset_all()
            camera.close()
            cfg.GPG.stop()
            horizontalPositionControlThread.join()
            distanceControlThread.join()
            # deviceScannerThread.join()
            break

cv2.destroyAllWindows()

def main () :
    personFollower()

if __name__== "__main__":
    main()

