#Demo2
#Open CV and I2C communication
#Zoe Logan and Lydia Jameson - Group 8
#Performs a variety of tasks starting at taking a picture, cropping it, converting to grayscale, masking out a specific color,
#detecting aruco markers, finding the color values of a pixel, and determining the distance and angle of an aruco marker from the camera within all of these images
#Sends instructions to the Pi based on these images

#Set up all the LCD Display stuff
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import time
import os

lcd_columns = 16
lcd_rows = 2

i2c = busio.I2C(board.SCL, board.SDA)
#lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

#Import the other stuff we need
from smbus2 import SMBus

#Set our bus and address
bus = SMBus(1)
arduino_address = 0x04

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

#Functions to communicate bytes
def read_byte():
    try:
        byte = bus.read_byte(arduino_address)
        return byte
    except:
        print("Reading Error")
        return None

def write_byte(value):
    try:
        bus.write_byte(arduino_address, value)
        print("Wrote: ", value)
    except:
        print("Writing Error")
    return -1

#Finds an aruco and sends the stop sweep signal to the Pi
def find_aruco():
    while(True):#go until a marker is found
        camera.capture("demo2.jpg")
        img = cv.imread("demo2.jpg")
        
        gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)#convert to grayscale

        dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_1000)
        parameters = cv.aruco.DetectorParameters_create()
        #detect markers
        corners,ids,rejectedImgPoints = cv.aruco.detectMarkers(gray_img, dictionary, parameters = parameters)
        
        if(ids != None):
            #Send stop sweep signal
            write_byte(251)
            break #marker is now found so exit loop

#Takea  fresh picture and calculate the distance and angle from it
def take_picture():
    pic_not_found = 1
    temp = 0
    while(pic_not_found == 1):#go until a marker is found
        camera.capture("demo2.jpg")
        img = cv.imread("demo2.jpg")
        #cv.imshow("img", i)
        gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)#convert to grayscale
        #cv.imshow('gray img', gray_img)
        cv.waitKey(3)
        #cv.destroyAllWindows()

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
            temp += 1

        if(temp == 3):
            os.system('rm demo2.jpg')
            return None, None

    os.system('rm demo2.jpg')
    return read_distance, read_angle

#Fin the distance an aruco is away        
def distance_finder(corners):#determines the distance from the aruco marker to the camera
    #constants to define the relationship between distance and width
    print(corners)
    CONST_WIDTH = 241.25#pixels
    CONST_DIST = 18#inches
    #find average width of marker via the corners
    width_1 = abs(corners[0][0][0][0]- corners[0][0][3][0])#top left to bottom left
    print("width 1:", width_1)
    width_2 = abs(corners[0][0][0][1]- corners[0][0][1][1])#top left to top right
    print("width 2:", width_2)
    width_3 = abs(corners[0][0][3][1]- corners[0][0][2][1])#bottom left to bottom right
    print("width 3:", width_3)
    width_4 = abs(corners[0][0][2][0]- corners[0][0][1][0])#bottom right to top right
    print("width 4:", width_4)
    avg_width = (width_1 + width_2 + width_3 + width_4)/4 #find avg width of all sides
    print(avg_width)
    

    #use dist/width ratio to find new distance
    distance = CONST_DIST*(CONST_WIDTH/avg_width) #in inches
    distance = round(distance, 0)
    print("Distance:", distance)
    
    return distance

#Find the angle to an Aruco
def angle_finder(gray_img, corners):
    
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
    print("Angle: %s" % angleDegree)
    angleDegree = round(angleDegree, 0)
    return angleDegree

#Initialize the things necessary for main
ans = 0
while ans != 'q':
    print("Press '1' to run the move to the beacon test, '2' to run the circle test, 'q' to quit: ")
    ans = input()

    #Move to test
    if(ans == '1'):
        #Sweep until an Aruco is found
        distance = None
        while(distance == None):
            write_byte(250)
        
            find_aruco()
            distance, angle = take_picture()

        #Convert the distance and angle to be sent
        distance = int(distance)
        angle = int(angle)

        #Send the distance
        write_byte(255)
        time.sleep(0.1)
        write_byte(distance)
        time.sleep(0.1)

        #send the angle
        write_byte(254)
        time.sleep(0.1)
        write_byte(angle)
        time.sleep(0.1)

        #Send the "new command" signal
        write_byte(252)
        time.sleep(0.1)

    #Circle Test
    elif(ans == '2'):
        #Sweep until an Aruco is found
        distance = None
        while(distance == None):
            write_byte(250)
        
            find_aruco()
            distance, angle = take_picture()

        #Convert the distance and angle to be sent
        distance = int(distance)
        angle = int(angle)

        #Send the distance
        write_byte(255)
        time.sleep(0.1)
        write_byte(distance - 4)
        time.sleep(0.1)

        #Send the angle
        write_byte(254)
        time.sleep(0.1)
        write_byte(angle)
        time.sleep(0.1)

        #Send the circle command
        write_byte(253)
        time.sleep(0.1)

        #Send the "new command" signal
        write_byte(252)
        time.sleep(0.1)

    #A test function to just run the circle
    elif(ans == '3'):
        write_byte(253)
        time.sleep(0.1)

        write_byte(252)
        time.sleep(0.1)

    #Quit statement
    elif(ans == 'q'):
        break

    else:
        print("Enter a valid choice")
