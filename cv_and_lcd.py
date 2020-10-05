#lcd code + 4.2



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
arduino_address = 0x04


#import necessary libraries for capturing image
import numpy as np
import cv2 as cv
from picamera import PiCamera
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
    except:
        print("Writing Error")
    return -1


def camera_quadrant_finder():
    i = 0 #used for however many times necessary to take images
    quadrant = 0 #will equal 0,1,2,3
    radian = 0 #equal the value to print to the lcd

    while(i < 1): #set i< for desired number of images
        camera.capture("%d.jpg" % i)
        img = cv.imread("%d.jpg" % i)
        cv.imshow("img", i)
        gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)#convert to grayscale
        cv.imshow('gray img', gray_img)
        cv.waitKey(0)
        cv.destroyAllWindows()
    
        #print("here")
        dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
        parameters = cv.aruco.DetectorParameters_create()
        #detect markers
        corners,ids,rejectedImgPoints = cv.aruco.detectMarkers(gray_img, dictionary, parameters = parameters)

        if(ids != None):
            #print(ids)
            h, w = gray_img.shape #determine size of image to find quadrants
            mid_h = h/2
            mid_w = w/2 #image midpoints
            #print("mid height:", mid_h)
            #print("mid width:", mid_w)

            #find center of aruco marker
            for corner in corners:
                center_x = (corner[0][0][0] + corner[0][1][0] + corner[0][2][0] + corner[0][3][0]) / 4
                center_y = (corner[0][0][1] + corner[0][1][1] + corner[0][2][1] + corner[0][3][1]) / 4
            print("center x:", center_x)
            print("center y:", center_y)
            #4 quadrants: starting at 0 in top right and moving CW
            #0: y > mid_h and x > mid_w (3pi/2)(top right)
            #1: < mid_h and > mid_w (0) (bottom right)
            #2: < mid_h and < mid_w (pi/2) (bottom left)
            #3: > mid_h and < mid_w (pi) (top left)

            #check to see which quadrant marker falls into and print/return
            if(center_y < mid_h) and (center_x > mid_w):#the height <,>, signs are flipped due to camera flippingimg
                quadrant = 0
                radian = '3pi/2 - 2pi'
                print(quadrant, " : ", radian)
                print("top right")
            if(center_y > mid_h) and (center_x > mid_w):
                quadrant = 1
                radian = '0 - pi/2'
                print("bottom right")
                print(quadrant, " : ", radian)
            if(center_y > mid_h) and (center_x < mid_w):
                quadrant = 2
                radian = 'pi/2 - pi'
                print("bottom left")
                print(quadrant, " : ", radian)
            if(center_y < mid_h) and (center_x < mid_w):
                quadrant = 3
                radian = 'pi - 3pi/2'
                print("top left")
                print(quadrant, " : ", radian) 
        else:
            print("No markers found")

        i = i + 1

    return(quadrant)#return the quadrant value to be returned to the LCD/motor

#Initialize everything
lcd.color = [100, 0, 0]
ans2 = 0

#Make sure the correct arduino file is loaded
print("Load I2C_Test.ino on the arduino.")

#Loop until the user is done testing
while ans2 != 'q':
    ans2 = input("Enter 1-9 or 'q' to quit: ")
    if ans2 == 'q':
        break
    else:
        #Send and receive the message
        #take picture and save quadrant
        cam_quadrant = camera_quadrant_finder()
        write_byte(int(cam_quadrant))
        print("RPi sent:", int(cam_quadrant))
        time.sleep(1)
                
        number = read_byte()
        print("Arduino sent:", number)

        #Print to the LCD
        lcd.message = "sent: " + str(cam_quadrant) + "\nreceived: " + str(number)
        #lcd.message = "sent: " + str(cam_quadrant) + "\nreceived: " + '45'


