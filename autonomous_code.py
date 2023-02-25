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





def canny(image):
    gray = cv2.cvtColor(lane_image, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray,(5,5),0)
    canny = cv2.Canny(blur,80,255)
    return canny


def region_of_interestR(image):
    height = image.shape[0]
    #print(height)
    polygons = np.array([
        [(160,height),(160,125),(320,125),(320,height)]
        ])
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygons, 255)
    masked_image = cv2.bitwise_and(image,mask)
    return masked_image

def region_of_interestL(image):
    height = image.shape[0]
    #print(height)
    polygons = np.array([
        [(0,height),(0,125),(160,125),(160,height)]
        ])
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygons, 255)
    masked_image = cv2.bitwise_and(image,mask)
    return masked_image

def display_lines(image, lines,x_mid):
    line_image = np.zeros_like(image)
    #print(lines)
    if lines is not None:
        for line in lines:
            print(line)
            x1,y1,x2,y2 = line.reshape(4)
            cv2.line(line_image,(x1,y1),(x2,y2),(255,255,255),10)
    cv2.line(line_image,(165,240),(165,200),(255,0,0),3)
    x_mid = int(x_mid)
    cv2.line(line_image,(x_mid,240),(x_mid,200),(0,0,255),3)
    return line_image 


def make_coordinates(image, line_parameters):
    slope ,intercept = line_parameters
    y1 = image.shape[0]
    y2 = int(y1*(3/5))
    if abs(slope) > 0.00001:
        x1 = int((y1 - intercept)/slope)
        x2 = int((y2 - intercept)/slope)
    else:
        x1 = 0
        x2 = 0
    return np.array([x1,y1,x2,y2])


def average_slope_interceptL(image,lines):
    left_fit = []
    
    if lines is not  None:
        for line in lines:
            x1,y1,x2,y2 = line.reshape(4)
            parameters = np.polyfit((x1,x2),(y1,y2),1)
            slope = parameters[0]
            intercept = parameters[1]
            left_fit.append((slope, intercept))
        left_fit_average = np.average(left_fit,axis=0)
        #right_fit_average = np.average(right_fit,axis=0)
        #print(left_fit_average,'L')
        angleL = math.atan(left_fit_average[0])*180/math.pi
        #print(angleL,"left angle")
        #print(right_fit_average,'R')
        left_line = make_coordinates(image,left_fit_average)
        
        xl1,yl1,xl2,yl2 = left_line.reshape(4)
        x_l_avg = (xl1+xl2)/2
        #print(left_line,"leftttttt")
        
        
        #right_line = make_coordinates(image,right_fit_average)
        return left_line,abs(angleL),True,x_l_avg
    
    else:
        return None, 90 ,False,0

def average_slope_interceptR(image,lines):
    right_fit = []
    
    if lines is not  None:
        for line in lines:
            x1,y1,x2,y2 = line.reshape(4)
            parameters = np.polyfit((x1,x2),(y1,y2),1)
            slope = parameters[0]
            intercept = parameters[1]
            right_fit.append((slope, intercept))
            

        right_fit_average = np.average(right_fit,axis=0)
        #right_fit_average = np.average(right_fit,axis=0)
        #print(right_fit_average,'R')
        angleR = math.atan(right_fit_average[0])*180/math.pi
        #print(angleR,"right angle")
        #print(right_fit_average,'R')
        right_line = make_coordinates(image,right_fit_average)
        
        xr1,yr1,xr2,yr2 = right_line.reshape(4)
        x_r_avg = (xr1+xr2)/2
        #print(right_line,"rightttttt")
        #right_line = make_coordinates(image,right_fit_average)
        return right_line,abs(angleR),True,x_r_avg
    
    else:
        return None, 90 ,False,0



    

def stop_detection(img):
    gray_image = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    stops = stop_detect.detectMultiScale(gray_image)
    
    for (x, y, w, h) in stops:
        cv2.rectangle(img, (x, y),(x + w, y + h),(255, 255, 255), 2)
        cv2.putText(img, "Stop Sign", (x,y), cv2.FONT_HERSHEY_TRIPLEX, 1,(255, 255, 255) , 1)
        cv2.putText(img, str(w), (x+w,y+h), cv2.FONT_HERSHEY_SIMPLEX, 1,(0, 255,255) , 2)
        stop_width = w
    #print(len(stops),"lennnnnnnnnnn")
    
    if len(stops) > 0:
        return img,True , stop_width
    return img,False , 0

def uturn_detection(img):
    gray_image = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    uturns = uturn_detect.detectMultiScale(gray_image)
    
    for (x, y, w, h) in uturns:
        cv2.rectangle(img, (x, y),(x + w, y + h),(255, 255, 255), 2)
        cv2.putText(img, "Uturn", (x,y), cv2.FONT_HERSHEY_TRIPLEX, 1,(255, 255, 255) , 1)
        cv2.putText(img, str(w), (x+w,y+h), cv2.FONT_HERSHEY_SIMPLEX, 1,(0, 255,255) , 2)
       
    #print(len(stops),"lennnnnnnnnnn")
    
    if len(uturns) > 0:
        return img,True
    return img,False

def hump_detection(img):
    gray_image = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    humps = hump_detect.detectMultiScale(gray_image)
    
    for (x, y, w, h) in humps:
        cv2.rectangle(img, (x, y),(x + w, y + h),(255, 255, 255), 2)
        cv2.putText(img, "Bump", (x,y), cv2.FONT_HERSHEY_TRIPLEX, 1,(255, 255, 255) , 1)
        cv2.putText(img, str(w), (x+w,y+h), cv2.FONT_HERSHEY_SIMPLEX, 1,(0, 255,255) , 2)
       
    #print(len(stops),"lennnnnnnnnnn")
    
    if len(humps) > 0:
        return img
    return img

def red_light_detection(img):
    gray_image = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    reds = red_detect.detectMultiScale(img)
    
    for (x, y, w, h) in reds:
        cv2.rectangle(img, (x, y),(x + w, y + h),(0, 0, 255), 2)
        cv2.putText(img, "RED", (x+w,y), cv2.FONT_HERSHEY_SIMPLEX, 1,(0, 0,255) , 2)
        cv2.putText(img, str(w), (x+w,y+h), cv2.FONT_HERSHEY_SIMPLEX, 1,(0, 255,255) , 2)
        width = w
    #print(len(reds),"redssss")
    if len(reds)>0:
        return img,True,width
            
    return img,False,0

def green_light_detection(img):
    gray_image = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    greens = green_detect.detectMultiScale(gray_image)
    
    for (x, y, w, h) in greens:
        cv2.rectangle(img, (x, y),(x + w, y + h),(0, 255, 0), 2)
        cv2.putText(img, "GREEN", (x+w,y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0, 255,0) , 1)

        
    
    #print(len(greens),"greennnns")
    if len(greens)>0:
        return img,True
    
    return img,False



def object_detection(img):
    gray_image = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    objects = obj_detect.detectMultiScale(gray_image)
    
    for (x, y, w, h) in objects:
        cv2.rectangle(img, (x, y),(x + w, y + h),(0, 255, 0), 2)
        cv2.putText(img, "VEHICLE", (x+w,y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0, 255,0) , 1)
        cv2.putText(img, str(w), (x+w,y+h), cv2.FONT_HERSHEY_SIMPLEX, 1,(0, 255,255) , 2)
        width = w
    
    #print(len(greens),"greennnns")
    if len(objects)>0:
        return img,True,width
    
    return img,False,0




stop_detect = cv2.CascadeClassifier("final_stop_cascade.xml")
red_detect = cv2.CascadeClassifier("red_cascade.xml")
green_detect = cv2.CascadeClassifier("green_cascade.xml")
obj_detect = cv2.CascadeClassifier("obj_cascade.xml")

uturn_detect = cv2.CascadeClassifier("uturn_cascade.xml")
hump_detect = cv2.CascadeClassifier("humps2_cascade.xml")
uturn_detected = False


addr = 0x8
bus = SMBus(1)
stop_once = True
camera = PiCamera()
camera.resolution = ( 320,240)
camera.framerate = 20
rawCapture = PiRGBArray(camera, size=(320,240))
red_light = False
"""
camera.brightness = 50
camera.contrast = 30
camera.saturation = 50
#camera.analog_gain = 50
 """   
    

time.sleep(0.1)
start = time.perf_counter()
counter = 1
obj_check = False
obj_check_once = True

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    start = time.perf_counter()
    image = frame.array
    #lane_image = np.copy(image)
    
    lane_image,stopp,stop_width= stop_detection(image)
    
    canny1 = canny(lane_image)
    cropped_imageL = region_of_interestL(canny1)
    cropped_imageR = region_of_interestR(canny1)
    


    linesL = cv2.HoughLinesP(cropped_imageL,1,np.pi/180,30,np.array([]),minLineLength = 20,maxLineGap = 5)
    linesR = cv2.HoughLinesP(cropped_imageR,1,np.pi/180,30,np.array([]),minLineLength = 20,maxLineGap = 5)

    betterlinesL,leftangle,flag1,xl = average_slope_interceptL(image,linesL)
    betterlinesR,rightangle,flag2,xr = average_slope_interceptR(image,linesR)
    
    x_mid = (xl+xr)/2
    mid_dif = (165-x_mid)
    print(mid_dif,"midddd")
    
    i2c_val = 128-int(mid_dif)*2
    int_mid = int(mid_dif)
    print("EnableL",128-(int_mid)*4)
    print("EnableR",128+int_mid*4-1)
    
    
    #print(leftangle,'LA')
    #print(rightangle,'RA')
    
    if not red_light:
        try:
            bus.write_byte(addr,i2c_val)
        except:
            print("I2C not established")
        
    if stopp and stop_width <50 and stop_width>40:
        print('Stoppppppppppppppppp')
        try:
            bus.write_byte(addr,255)
        except:
            print("I2C not established")
        #stop_once = False
    
    #print(stop_once)  
    #line_imageL = display_lines(lane_image, betterlinesL)
    #line_imageR = display_lines(lane_image, betterlinesR)
            
    if flag1 and flag2:
        betterlines = np.array([betterlinesL,betterlinesR])
        line_image = display_lines(lane_image, betterlines,x_mid)
        
    else:
        line_image = np.zeros_like(image)
        try:
            bus.write_byte(addr,0)
        except:
            print("I2C not established")
        if uturn_detected:
            try:
                bus.write_byte(addr,254)
            except:
                print("I2C not established")
            uturn_detected = False
            
            
        
    
    image,red_flag,width = red_light_detection(lane_image)
    if red_flag and width>45:
        red_light = True
        try:
            bus.write_byte(addr,255)
        except:
            print("I2C not established")
        print("red_light")
    
    image,green_flag = green_light_detection(image)
    
    image,obj_flag,obj_w = object_detection(image)
    
    if obj_w>70 and obj_check_once:
        obj_check = True
        obj_check_once = False
        obj_time1 = time.perf_counter() 
        print("timeeeee",obj_time1)
        
        try:
            bus.write_byte(addr,251)
        except:
            print("I2C not established")
        
    if obj_check:
        #y = '100'
        obj_time2 = time.perf_counter() - obj_time1
        #send Right
        
    if obj_check and obj_time2 > 6:
         #mssg = 'L'
         obj_check = False
         obj_check_once= True
         try:
            bus.write_byte(addr,252)
         except:
            print("I2C not established")
    
    
    if red_light and green_flag:
        red_light = False
        try:
            bus.write_byte(addr,128)
        except:
            print("I2C not established")
            print("green_light")
        
        """
        if green_flag:
            red_light = False
            try:
                bus.write_byte(addr,128)
            except:
                print("I2C not established")
            print("green_light")
         """   
            
          
          
    image,uflag = uturn_detection(image)
    #image = hump_detection(image) 
    if uflag:
        uturn_detected = True
    
    
        
    combo_image = cv2.addWeighted(line_image,1,image,0.8,1)
    
    """
    #image = cv2.imread('image18.jpg')
    #print(canny)
    """
    
    cv2.namedWindow('result',cv2.WINDOW_NORMAL)
    cv2.moveWindow('result',0,100)
    cv2.resizeWindow('result',640,480)
    cv2.imshow('result',combo_image)
    key = cv2.waitKey(1)
    rawCapture.truncate(0)
    
    end = time.perf_counter()
    print("FPS",1/(end-start))

    if key == ord("q"):
            break

#plt.imshow(combo_image)
#plt.imshow(combo_image)
#plt.show()
#plt.imshow(line_imageR)
#plt.show()







