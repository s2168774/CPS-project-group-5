import cv2
import numpy as np 
import imutils
import time
import easygopigo3
from picamera.array import PiRGBArray
from picamera import PiCamera 

GPG = easygopigo3.EasyGoPiGo3() # Create an instance of the GoPiGo3 class. GPG will be the GoPiGo3 object.
GPG.set_speed(100)

# Initialize PiCamera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
camera.rotation = 180

# Initialize the HOG descriptor/people detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

rawCapture = PiRGBArray(camera, size=(640, 480))

#loop through frames continuously
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array

    image = imutils.resize(image, width=min(400, image.shape[1]))
    orig = image.copy()

    # detect people in the image
    (rects, weights) = hog.detectMultiScale(image, winStride=(4, 4),
    padding=(8, 8), scale=1.05)

    rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
    pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)

    # draw the final bounding boxes
    for (xA, yA, xB, yB) in pick:
    cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)


# show some information on the number of bounding boxes
    filename = imagePath[imagePath.rfind("/") + 1:]
    print("[INFO] {}: {} original boxes, {} after suppression".format(
    filename, len(rects), len(pick)))
    # show the output images
    cv2.imshow("Before NMS", orig)
    cv2.imshow("After NMS", image)

    # Exit if 'esc' is clicked
    # cleanup hardware
    key = cv2.waitKey(1)
    rawCapture.truncate(0)
    if key == 27:
        # GPG.reset_all()
        camera.close()
        break

cv2.destroyAllWindows()

