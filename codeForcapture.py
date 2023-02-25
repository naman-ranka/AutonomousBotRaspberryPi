from picamera import PiCamera
from time import sleep

import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
from picamera.array import PiRGBArray
import time
import RPi.GPIO as GPIO

camera = PiCamera()
camera.resolution = ( 640,480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640,480))

time.sleep(0.1)
i = 1

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array
    a = str(i)
    cv2.imshow('result',image)
    cv2.imwrite("Uturn"+a+".jpg",image)
    key = cv2.waitKey(0)
    rawCapture.truncate(0)
    print(i)
    i = i+1
    
    if i ==60:
        break
    
    if key == ord("n"):
        continue
    




