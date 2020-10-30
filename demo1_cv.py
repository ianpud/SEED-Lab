#Demo1
#Open CV
#Zoe Logan and Lydia Jameson - Group 8
#Performs a variety of tasks starting at taking a picture, cropping it, converting to grayscale, masking out a specific color,
#detecting aruco markers, finding the color values of a pixel, and determining the distance and angle of an aruco marker from the camera within all of these images

#Set up all the LCD Display stuff
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import time

lcd_columns = 16
lcd_rows = 2

i2c = busio.I2C(board.SCL, board.SDA)
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

#Import the other stuff we need
from smbus2 import SMBus

#Set our bus and address
bus = SMBus(1)
#arduino_address = 0x04

import numpy as np
import cv2 as cv
from picamera import PiCamera
from time import sleep
import math

camera = PiCamera()
cam = np.array([[ 534, 0, 341],
 [0, 534, 233],[0, 0, 1]])
dist = np.array([-2.94138985e-01, 1.22538100e-01, 1.28531080e-03, -2.57140954e-04, 1.35279717e-02])
focal = cam[0,0]


#get gains of camera to behave better in lighting
cam_gain = camera.awb_gains
camera.awb_mode = 'off' #turn off auto white balanec
camera.awb_gains = cam_gain #set to manual data given by gains earlier

camera.iso = 640 #set to take pictures in poorer light

def angle_finder():
    i = 0 #to number pics taken
    pic_not_found = 1
    while(pic_not_found == 1):#go until a marker is found
        
        camera.capture("demo1-%d.jpg" % i)
        img = cv.imread("demo1-%d.jpg" % i)
        cv.imshow("img", i)
        gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)#convert to grayscale
        cv.imshow('gray img', gray_img)
        cv.waitKey(10)
        cv.destroyAllWindows()
            
        #print("here")
        dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_1000)
        parameters = cv.aruco.DetectorParameters_create()
        #detect markers
        corners,ids,rejectedImgPoints = cv.aruco.detectMarkers(gray_img, dictionary, parameters = parameters)
        #print(ids)
        if(ids != None):
            print(ids)
            h, w = gray_img.shape #determine size of image to find quadrants
            mid_h = h/2
            mid_w = w/2 #image midpoints

            #find center of aruco marker
            for corner in corners:
                center_x = (corner[0][0][0] + corner[0][1][0] + corner[0][2][0] + corner[0][3][0]) / 4
                center_y = (corner[0][0][1] + corner[0][1][1] + corner[0][2][1] + corner[0][3][1]) / 4

            angle = math.atan((mid_w - center_x)/focal) #find differencce between center and marker center, and use focal to find angle
            #width - center marker so that marker to the left of marker is positive
            angleDegree = (angle * 180) / math.pi#convert to degrees
            print("Angle: %s" % angleDegree)
            
            pic_not_found = 0 #marker is now found so exit loop
            
        else:
            print("No markers found")
            pic_not_found = 1 #still no marker so keep taking pics

                    
        
        i = i + 1
        
    return(angleDegree)

#Initialize everything for the LCD
lcd.color = [100, 0, 0]

#Loop for testing. Press ^C to exit
while True:
    
    #angle_finder takes pictures until it gras a measurement
    angle = angle_finder()

    #Print to the LCD -- convert to radians
    lcd.message = "Aruco detected!" + "\nAngle: " + angle
    
    #keep the message on the LCD for three seconds
    time.sleep(3)

