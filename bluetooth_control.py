from picamera import PiCamera
from time import sleep

import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
from picamera.array import PiRGBArray
import time
import RPi.GPIO as GPIO
from smbus import SMBus
import bluetooth
import csv


def ConvertStringToBytes(src):
    converted = []
    for b in src:
        converted.append(ord(b))
    return converted



 
    
   





i2c_cmd = 0x01
f = open('/home/pi/Desktop/finding_lanes/finding_lanes/Data Collection/BetterLap/data_file.csv', 'w')
writer = csv.writer(f)
i = 1
server_socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
port = 1
server_socket.bind(("", port))
server_socket.listen(1)
client_socket, address = server_socket.accept()
print ("Accepted connection from ", address)
addr = 0x8
bus = SMBus(1)
stop_once = True

camera = PiCamera()
camera.resolution = ( 640,480)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(640,480))
red_light = False
"""
camera.brightness = 50
camera.contrast = 30
camera.saturation = 50
#camera.analog_gain = 50
 """   
    
mid_dif = 0
time.sleep(0.1)
x_val = 512
y_val = 512
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    character= client_socket.recv(1024)
    
    mssg = client_socket.recv(1024).decode("utf-8")
    print(mssg)
    try:
        val = mssg.rsplit("Y")
        y = val[1]
        y_val = int(y)
        valx = val[0].rsplit("X")
        x = valx[1]
        #y = '100'
        x_val = int(x)
        ledstate = "X"+x+"Y"+y
        print(ledstate)
        #mid_dif = -x
        
        i2c_val = 128-int(mid_dif)*2
    except:
        """
        val = int(mssg)
        i2c_val = val
        """
    #print(mid_dif)
    #print("mid_sent",character.decode("utf-8"))
    #print ("Received: ",character)
    
    #lane_image = np.copy(image)
 
    bytesToSend = ConvertStringToBytes(ledstate)
    try:
        bus.write_i2c_block_data(addr, i2c_cmd, bytesToSend)
    except:
        print("I2C not established")
        
    
    #int_mid = int(mid_dif)
    #print("EnableL",128-(int_mid)*4)
    #print("EnableR",128+int_mid*4-1)
    """
    try:
        bus.write_byte(addr,i2c_val)
    except:
        print("I2C not established!")
    """ 
    
    start = time.perf_counter()
    image = frame.array
    cv2.namedWindow('result',cv2.WINDOW_NORMAL)
    cv2.moveWindow('result',0,100)
    cv2.resizeWindow('result',640,480)
    cv2.imshow('result',image)
    key = cv2.waitKey(1)
    rawCapture.truncate(0)
    
    
    a = str(i)
    cv2.imwrite("Img"+a+".jpg",image)
    i = i+1
    end = time.perf_counter()
    print("FPS",1/(end-start))
    imgaddr = ("/home/pi/Desktop/finding_lanes/finding_lanes/Data Collection/BetterLap/"+"Img"+a+".jpg")
    
    writer.writerow([imgaddr,x_val,y_val])
    
    if key == ord("q"):
        f.close()
        break

#plt.imshow(combo_image)
#plt.imshow(combo_image)
#plt.show()
#plt.imshow(line_imageR)
#plt.show()



client_socket.close()
server_socket.close()






