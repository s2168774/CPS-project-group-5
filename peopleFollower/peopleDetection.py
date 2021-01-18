import cv2
import numpy as np 
import imutils
import time
import easygopigo3
import config as cfg

from threading import Thread
from collections import OrderedDict
from picamera.array import PiRGBArray
from picamera import PiCamera 
from utils import findCameraDistance, positionControl_PID, findCameraDistance, rectArea
from centroidTracker import CentroidTracker
from imutils.object_detection import non_max_suppression

from bleCommunication.bleScanner import DeviceScanner

# Initialize PiCamera
camera = PiCamera()
camera.resolution = (cfg.FRAME_WIDTH, cfg.FRAME_HEIGHT)
camera.framerate = 30
camera.rotation = 180

#initialize peopleTracker
peopleTracker = CentroidTracker()

# def nothing(x):
#     pass
 
# cv2.namedWindow("Trackbars")
 
# cv2.createTrackbar("B", "Trackbars", 0, 255, nothing)
# cv2.createTrackbar("G", "Trackbars", 0, 255, nothing)
# cv2.createTrackbar("R", "Trackbars", 0, 255, nothing)



# Create and start PID controller thread
controlThread = Thread(target = positionControl_PID)
controlThread.start()
print("thread started")

# Handle Bluetooth Low Energy device scan
deviceScanner = DeviceScanner()
# deviceScanner.startScan(float("inf"))
# deviceScanner.showAvilableDevices()
# TODO get values from scanner thread

deviceScannerThread = Thread(target = deviceScanner.startScan, args=(float("inf")))

rawCapture = PiRGBArray(camera, size=(cfg.FRAME_WIDTH, cfg.FRAME_HEIGHT))

#loop through frames continuously
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array

    print(" my device signal strength in dB: ", deviceScanner.rssi)
    # image = imutils.resize(image, width=min(400, image.shape[1]))


    # Find object in the image
    
    blurred = cv2.GaussianBlur(image, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV) # convert picture from BGR to HSV color format

    # Optional trackbars left for determining threshold 'live' if current is not working
    # B = cv2.getTrackbarPos("B", "Trackbars")
    # G = cv2.getTrackbarPos("G", "Trackbars")
    # R = cv2.getTrackbarPos("R", "Trackbars")
    B = 95
    G = 0
    R = 225

    color = np.uint8([[[B, G, R]]])
    hsvColor = cv2.cvtColor(color,cv2.COLOR_BGR2HSV)
    lowerLimit = np.uint8([hsvColor[0][0][0]-10, 150,150])
    upperLimit = np.uint8([hsvColor[0][0][0]+10,255,255])

    # Apply Filters START
    kernel = np.ones((15,15),np.uint8)
    mask = cv2.inRange(hsv, lowerLimit, upperLimit)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    cv2.imshow("mask", mask)
    # Apply Filters END

    # Get object contours START
    edged = cv2.Canny(mask, 35, 125)
    contours = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    # Get countours END
    rects = []
    objects = OrderedDict()
    contoursSorted = sorted(contours, key=cv2.contourArea, reverse=True)
    if len(contoursSorted) > 2 :
        contoursSorted = contoursSorted[:2]

    if len(contours) != 0 :
        i = 0
        for contour in contoursSorted:

            x,y,w,h = cv2.boundingRect(contour)
            print ("pixel width of object: %d" % w )
            i += 1
            # if (w) > 10:
            rects.append([x,y,x+w,y+h])
        # image = cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2)
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
        for (objectID, centroid) in objects.items():
            # draw both the ID of the object and the centroid of the
            # object on the output image
            centerX, centerY, area = centroid

            print("objectID: %d, area: %d " % (objectID, area))
            if prevArea < area :
                maxAreaBboxID = objectID
            prevArea = area
            text = "ID {}".format(objectID)
            cv2.putText(image, text, (centerX - 10, centerY - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.circle(image, (centerX, centerY), 4, (0, 255, 0), -1)

        print("maxAreaBboxID: ", maxAreaBboxID)
        if maxAreaBboxID in objects :
            trackedCentroid = objects[maxAreaBboxID]
            # print(objects[0])
            # print(trackedCentroid)
            centroidX = trackedCentroid[0]

        startX, startY, endX, endY = pick[0]
        # Find and display distance to the object

        estimatedDistanceToObj = findCameraDistance(endX - startX)

        cv2.putText(image, "%.2fcm" % (estimatedDistanceToObj),
        (image.shape[1] - 200, image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)

        ###### HANDLE ROBOT MOVEMENT_ START #####
        cfg.measurement = centroidX
        print(cfg.measurement)

        if estimatedDistanceToObj > 40 :
            cfg.GPG.set_motor_dps(cfg.GPG.MOTOR_LEFT, dps=cfg.MAX_SPEED - cfg.correction)
            cfg.GPG.set_motor_dps(cfg.GPG.MOTOR_RIGHT, dps=cfg.MAX_SPEED + cfg.correction)
        else :
            cfg.GPG.set_motor_dps(cfg.GPG.MOTOR_LEFT, dps=0)
            cfg.GPG.set_motor_dps(cfg.GPG.MOTOR_RIGHT, dps=0)
            # cfg.MAX_SPEED -= 10
        ## get first object found centroid
        
        # followedCentroid = objects[0]
        # followedCentroidX, followedCentroidY = followedCentroid
        # imageMidX = IMAGE_WIDTH / 2

        # # movement rules: stay within STEER_TRESHOLD
        # if (followedCentroidX < (imageMidX + STEER_TRESHOLD)) and (followedCentroidX > (imageMidX - STEER_TRESHOLD)) :
        #     GPG.steer(100, 100)
        # elif followedCentroidX > (imageMidX + STEER_TRESHOLD):
        #     GPG.steer(20, 100)
        # else :
        #     GPG.steer(100, 20)
        # ###### HANDLE ROBOT MOVEMENT_ END #####


    cv2.imshow("outputImage", image)

    # Exit if 'esc' is clicked
    # cleanup hardware
    key = cv2.waitKey(1)
    rawCapture.truncate(0)
    if key == 27:
        controlThread.join()
        deviceScannerThread.join()
        cfg.GPG.reset_all()
        camera.close()
        cfg.GPG.stop()
        break

cv2.destroyAllWindows()
