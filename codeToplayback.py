from picamera import PiCamera
from time import sleep

import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
from picamera.array import PiRGBArray
import time
import RPi.GPIO as GPIO
import csv



file = open('data_file.csv')
csvreader = csv.reader(file)
rows = []
for row in csvreader:
        rows.append(row)
rows
#camera = PiCamera()
# camera.resolution = ( 640,480)
# camera.framerate = 32
# rawCapture = PiRGBArray(camera, size=(640,480))

time.sleep(0.1)
i = 1

for r in rows:
   
    a = str(i)
    img_addr = r[0]
    image = cv2.imread(img_addr)
    print(img_addr)
    steer_angle = (int(r[1]) - 511)
    speed = -(int(r[2])-1023) - 511
    
    cv2.putText(image, "Steering Angle"+str(steer_angle), (200,400), cv2.FONT_HERSHEY_SIMPLEX, 1,(0, 255,255) , 1)
    cv2.putText(image, "Speed"+str(speed), (200,100), cv2.FONT_HERSHEY_SIMPLEX, 1,(0, 255,255) , 1)

    cv2.imshow('result',image)
    
    
    key = cv2.waitKey(100)
    
    
    #print(i)
    i = i+1
    
    
    
    if key == ord("n"):
        continue
    




    