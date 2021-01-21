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

from utils import findCameraDistance, horizontalPositionControl_PID, distanceControl_PID, findRssiDistance, rectArea, cameraInit, movingAverage
from centroidTracker import CentroidTracker
from imutils.object_detection import non_max_suppression
from wifiScanner import WifiScanner

def lineFollower() :

    # Initialize PiCamera
    camera = cameraInit()

    #initialize peopleTracker
    peopleTracker = CentroidTracker(maxDisappeared=5)

    centroidX = 0
    width = 1
    objects = OrderedDict()

    # Create and start PID controller thread
    horizontalPositionControlThread = Thread(target = horizontalPositionControl_PID)
    horizontalPositionControlThread.start()
    print("horizontal control thread started")

    rawCapture = PiRGBArray(camera, size=(cfg.FRAME_WIDTH, cfg.FRAME_HEIGHT))

    #loop through frames continuously
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        startTime = time.time()

        blurred = cv2.GaussianBlur(image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV) # convert picture from BGR to HSV color format

        B = 50  #blue 0..255
        G = 10  #green 0..255
        R = 200 #red 0..255

        #Set limits for color filter
        color = np.uint8([[[B, G, R]]])
        hsvColor = cv2.cvtColor(color,cv2.COLOR_BGR2HSV)
        lowerLimit = np.uint8([hsvColor[0][0][0]-10, 150,40])
        upperLimit = np.uint8([hsvColor[0][0][0]+10,255,255])

        # Apply Filters START
        kernel = np.ones((15,15),np.uint8)
        mask = cv2.inRange(hsv, lowerLimit, upperLimit)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        # Apply Filters END

        # Get object contours START
        edged = cv2.Canny(mask, 35, 125)
        contours = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        # Get countours END
        
        contoursSorted = sorted(contours, key=cv2.contourArea, reverse=True)
        if len(contoursSorted) > 1 :
            contoursSorted = contoursSorted[:1]

        rects = []
        if len(contours) != 0 :
            for contour in contoursSorted:
                x,y,w,h = cv2.boundingRect(contour)
                rects.append([x,y,x+w,y+h])
            pass

            rects = np.array(rects)
            pick = non_max_suppression(rects, probs=None, overlapThresh=0.45)

            # draw the final bounding boxes
            for (startX, startY, endX, endY) in pick:
                cv2.rectangle(image, (startX, startY), (endX, endY), (0, 255, 0), 2)
                pass

            objects = peopleTracker.update(pick)

        if bool(objects):
            maxAreaBboxID = 0
            prevArea = 0
            eraseIDs = []
            for (objectID, centroid) in objects.items():
                # draw both the ID of the object and the centroid of the
                # object on the output image
                centerX, centerY, area, width = centroid
                if area > 100 :
                    # print("objectID: %d, area: %d " % (objectID, area))
                    if prevArea < area :
                        maxAreaBboxID = objectID
                    prevArea = area
                    text = "ID {}".format(objectID)
                    cv2.putText(image, text, (centerX - 10, centerY - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.circle(image, (centerX, centerY), 4, (0, 255, 0), -1)
                # else:
                #     eraseIDs.append(objectID.copy())
            # for ID in eraseIDs :
            #     del objects[ID]
            # print("maxAreaBboxID: ", maxAreaBboxID)
            if maxAreaBboxID in objects :
                trackedCentroid = objects[maxAreaBboxID]
                centroidX = trackedCentroid[0]
                # width = movingAverage(trackedCentroid[3], cfg.bBoxWidths, windowSize = 10)

                # startX, startY, endX, endY = pick[0]

                # Find and display distance to the object
                # estimatedDistanceToObj = findCameraDistance(endX - startX)

            ###### HANDLE ROBOT MOVEMENT_ START #####
            # cfg.horizontal_measurement = centroidX # horizontal position measurement
            cfg.horizontal_measurement = movingAverage(centroidX, cfg.horizontalPositions, windowSize=2) # horizontal position 

            cfg.GPG.set_motor_dps(cfg.GPG.MOTOR_LEFT, dps=cfg.MAX_SPEED - int(cfg.horizontal_correction))
            cfg.GPG.set_motor_dps(cfg.GPG.MOTOR_RIGHT, dps=cfg.MAX_SPEED + int(cfg.horizontal_correction))
            # print("distance correction: ", cfg.distance_correction)
            # print("horizontal_correction: ", cfg.horizontal_correction)

            # ###### HANDLE ROBOT MOVEMENT_ END #####
        # else:
        #     cfg.GPG.stop()

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