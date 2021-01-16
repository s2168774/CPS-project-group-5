import cv2
import numpy as np 
import imutils
import time
import easygopigo3
import utils

from picamera.array import PiRGBArray
from picamera import PiCamera 
#from utils import findCameraDistance
from centroidTracker import CentroidTracker
from imutils.object_detection import non_max_suppression

GPG = easygopigo3.EasyGoPiGo3() # Create an instance of the GoPiGo3 class. GPG will be the GoPiGo3 object.
GPG.set_speed(300)

IMAGE_WIDTH = 384
IMAGE_HEIGHT = 288
STEER_TRESHOLD = 20

# Initialize PiCamera
camera = PiCamera()
camera.resolution = (IMAGE_WIDTH, IMAGE_HEIGHT)
camera.framerate = 10
camera.rotation = 180

#initialize peopleTracker
peopleTracker = CentroidTracker()

# Initialize the HOG descriptor/people detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())



rawCapture = PiRGBArray(camera, size=(IMAGE_WIDTH, IMAGE_HEIGHT))

# movement controller parameters
error = 0.0
setpoint = int(IMAGE_WIDTH / 2)

#loop through frames continuously
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    startTime = time.time()

    image = frame.array

    image = imutils.resize(image, width=min(400, image.shape[1]))


    # detect people in the image
    # (rects, weights) = hog.detectMultiScale(image, winStride=(6, 6),
    # padding=(8, 8), scale=1.05)
    (rects, weights) = hog.detectMultiScale(image, winStride=(10, 10),
    padding=(8, 8), scale=1.1)

    rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
    pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)


    # draw the final bounding boxes
    for (startX, startY, endX, endY) in pick:
        cv2.rectangle(image, (startX, startY), (endX, endY), (0, 255, 0), 2)
        pass
        # width = endX - startX
        # height = endY - startY
        # 
    # objects = peopleTracker.update(pick)

    # if bool(objects):
    #     for (objectID, centroid) in objects.items():
    #         # draw both the ID of the object and the centroid of the
    #         # object on the output image
    #         text = "ID {}".format(objectID)
    #         cv2.putText(image, text, (centroid[0] - 10, centroid[1] - 10),
    #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    #         cv2.circle(image, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)

        ###### HANDLE ROBOT MOVEMENT_ START #####

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
    endTime = time.time()

    print("image processing time: ", endTime - startTime)
    # Exit if 'esc' is clicked
    # cleanup hardware
    key = cv2.waitKey(1)
    rawCapture.truncate(0)
    if key == 27:
        GPG.reset_all()
        camera.close()
        break

cv2.destroyAllWindows()

