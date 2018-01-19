# import the necessary packages
from __future__ import print_function
from imutils.object_detection import non_max_suppression
from imutils import paths
import numpy as np
import argparse
import imutils
import cv2
import RPi.GPIO as GPIO

from threading import Thread

from picamera.array import PiRGBArray
from picamera import PiCamera
import time

# initialize the camera and grab a reference to the raw camera capture
resX = 240
resY = 180
camera = PiCamera()
camera.resolution = (resX,resY)
camera.framerate = 10
rawCapture = PiRGBArray(camera, size=(resX, resY))

# initialize textIn and textOut values
textIn = 0
textOut = 0

print(time.strftime("%H_%M_%S"))
#fourcc = cv2.VideoWriter_fourcc(*'XVID')
fourcc = cv2.cv.CV_FOURCC(*'XVID')
out = cv2.VideoWriter(time.strftime("%H_%M_%S")+'.avi',fourcc, 20.0, (resX, resY))

# initialize the HOG descriptor/person detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
detectFlag = 0
detectCounter = [0]
# allow the camera to warmup
time.sleep(0.1)

GPIO.setmode(GPIO.BOARD)

GPIO.setup(16, GPIO.OUT)



# methods for IN and OUT counters
def testIntersectionIn(x, y, z):
    
    if((x >= 50) and  (x <= 130) and (x < z[0]) and (z[0]>0) and (z[0]> 130)):
        print (x,z[0],"IN")
        return True
    return False

def testIntersectionOut(x, y, z):
   
    if((x >= 50) and  (x <= 130) and (x > z[0]) and (z[0]>0) and z[0]<50):
        print (x,z[0],"OUT")
        return True

    return False

previousObj = (0,0)

def classfier(testImage,threadNum,capTime, detectCounter):
    global textIn, textOut, previousObj
    #print(threadNum,capTime)
    (rects, weights) = hog.detectMultiScale(testImage, winStride=(8, 8),
        padding=(16, 16), scale=1.1)

    rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
    pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)

	# draw the final bounding boxes

    cv2.line(image, (50,0), (50,192), (250, 0, 1), 2) #blue line
    cv2.line(image, (130,0), (130,192), (0, 0, 255), 2)#red line
    
    for (xA, yA, xB, yB) in pick:
	
        print("Image detected")
	print ("Previous Coord : ",previousObj)
        detectCounter[0] = 0
        cv2.rectangle(testImage, (xA, yA), (xB, yB), (0, 255, 0), 2)

	rectangleCenterPont = ((xA + xB) /2, (yA + yB) /2 )
	cv2.circle(testImage, rectangleCenterPont, 1, (0,0,255), 5)
	print (rectangleCenterPont)

	if(testIntersectionIn((xA + xB) /2, (yA + yB) /2,previousObj)):
            textIn += 1
            	#print testIntersectionIn((x + x + w) / 2, (y + y + h) / 2)

        if(testIntersectionOut((xA + xB) /2, (yA + yB) /2,previousObj)):
            textOut += 1
     	      	#print textOut
	
	previousObj = rectangleCenterPont
	

    cv2.putText(image, "In: {}".format(str(textIn)), (10, 50),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv2.putText(image, "Out: {}".format(str(textOut)), (10, 70),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    # print(pick,"\n");
    curTime = time.time()
    #print ("Total time from capture", curTime - capTime)
    out.write(testImage)
    cv2.imshow("After NMS", testImage)

# capture frames from the camera
i = 0
frameCount = 0
prevTime = time.time()
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    if (detectCounter[0] < 11):
        GPIO.output(16, GPIO.LOW)
        print ("Waiting ", detectCounter[0])
        detectCounter[0] += 1
    else:
        GPIO.output(16,GPIO.HIGH)
    image = frame.array
    captureTime = time.time()
    # print("FRAME Time", captureTime-prevTime)
    prevTime = captureTime

    # if frameCount == 0:
        # frameCount = 0
    #if i == 0:
    t1 = Thread(target = classfier, args = (image,i,captureTime,detectCounter))
    t1.start()
    threadPick = t1.join()


    # cv2.imshow("Frame", image)
    key = cv2.waitKey(1) & 0xFF

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

     # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        cleanup_stop_thread();
        sys.exit()
  