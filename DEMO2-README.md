DEMO 2 - README.txt

Demo2 is the next step for the robot to get to the final demonstration. The two tasks to tackle for demo2 include:

-the robot must be able to move within 1 foot of the beacon and stop before hitting it. 
-the robot must move to within 1 foot of the beacon, and then completes a move around the beacon moving no more than 2 feet away from the beacon at any time, and stop within 3 inches of where the robot arrived at the beacon

The demo2 code is comprised of the following files: 
demo2_cv_lcd.py
**

demo2_cv_lcd.py consists of all of the Raspberry pi code, and is responsible for the computer vision, system integration, and controlling the Arduino.
It first prompts for what test to be completed, and then runs said test. 

For both tests, the robot will first take images until it finds an Aruco marker, (via a sweep run through the Arduino), and then sends the angle and distance of the marker to the Arduino via the system integration read and write byte functions. 




