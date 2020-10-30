# SEED-Lab
Demo 1

Demo1 pertained to three large tasks for the arduino/pi robot, including:
-detect and recognize an aruco marker/beacon, and report the angle in degrees between the camera axis and the beacon
-move forward in a straight line and stop after a specified distance in feet (between 1 and 10)
-rotate the robot by a specified angle in degrees, and move forward 1 foot

The Demo1 code is comprised of the two following files:

demo1_cv.py

demo1_dist_angle.ino


demo1_cv.py goes onto the Pi, and contains the code for the first task (detecting the aruco marker and reporting the angle back), as well as the system integration code that allows the pi and arduino to communicate and display the angle reported to the LCD screen. 

demo1_dist_angle.ino is loaded onto the arduino, and enables the motors on the robot to send the robot to either the desired distance between 1 and 10 feet, or turn the robot the desired angle. It can be loaded onto the arduino from the Pi as well, so that no computer is needed to access it. 
