#Demo2
#Open CV and LCD communication
#Zoe Logan & Lydia Jameson - Group 8
#initializes the lcd and picamera, and sets up a loop so that the camera will continue to take pictures until an aruco marker is found,
#and determine the angle and distance from the robot to the aruco marker, and send that information to the arduino and LCD 

#Set up all the LCD Display stuff
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import time
import os

lcd_columns = 16
lcd_rows = 2

##i2c = busio.I2C(board.SCL, board.SDA)
##lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
##
###Import the other stuff we need
##from smbus2 import SMBus
##
###Set our bus and address
##bus = SMBus(1)
##arduino_address = 0x04

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

camera.iso = 800 #set to take pictures in poorer light

###Functions to communicate bytes
##def read_byte():
##    try:
##        byte = bus.read_byte(arduino_address)
##        return byte
##    except:
##        print("Reading Error")
##        return None
##
##def write_byte(value):
##    try:
##        bus.write_byte(arduino_address, value)
##    except:
##        print("Writing Error")
##    return -1

def take_picture():
    pic_not_found = 1
    while(pic_not_found == 1):#go until a marker is found
        camera.capture("demo2.jpg")
        img = cv.imread("demo2.jpg")
        #cv.imshow("img", i)
        gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)#convert to grayscale
        cv.imshow('gray img', gray_img)
        cv.waitKey(15)
        cv.destroyAllWindows()

        dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_1000)
        parameters = cv.aruco.DetectorParameters_create()
        #detect markers
        corners,ids,rejectedImgPoints = cv.aruco.detectMarkers(gray_img, dictionary, parameters = parameters)
        
        if(ids != None):
            rvecs, tvecs, _objPoints = cv.aruco.estimatePoseSingleMarkers(corners, 96, cam, dist)#find the pose of the aruco markers to use tvecs for distance and angle
            read_distance = distance_finder(corners)
            read_angle = angle_finder(gray_img, corners)
            pic_not_found = 0 #marker is now found so exit loop
            
        else:
            print("No markers found")

    os.system('rm demo2.jpg')
    return read_distance, read_angle

        
def distance_finder(corners):#determines the distance from the aruco marker to the camera
    #constants to define the relationship between distance and width
    CONST_WIDTH = 241.25#pixels
    CONST_DIST = 18#inches
    #find average width of marker via the corners
    width_1 = abs(corners[0][0][0][1]- corners[0][0][3][1])#top left to bottom left
    print("width 1:", width_1)
    width_2 = abs(corners[0][0][0][0]- corners[0][0][1][0])#top left to top right
    print("width 2:", width_2)
    width_3 = abs(corners[0][0][3][0]- corners[0][0][2][0])#bottom left to bottom right
    print("width 3:", width_3)
    width_4 = abs(corners[0][0][2][1]- corners[0][0][1][1])#bottom right to top right
    print("width 4:", width_4)
    avg_width = (width_1 + width_2 + width_3 + width_4)/4 #find avg width of all sides
    print(avg_width)
    

    #use dist/width ratio to find new distance
    distance = CONST_DIST*(CONST_WIDTH/avg_width) #in inches
    distance = round(distance, 2)
    
    return distance

def angle_finder(gray_img, corners):#determines the angle from the aruco marker to the camera
    
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
    angleDegree = angleDegree/2 #the number is always twice what it should be so divide by 2
    #print("Angle: %s" % angleDegree)
        
    return angleDegree

#Initialize everything for the LCD
#lcd.color = [100, 0, 0]



distance, angle = take_picture()

print("distance: ", distance)
    #print("angle: ", angle)

    #Print to the LCD -- convert to radians
    #lcd.message = "Aruco detected!" + "\nAngle: " + str(angle)
    #lcd.message = "sent: " + str(cam_quadrant) + "\nreceived: " + '45'
    
    #time.sleep(3)

