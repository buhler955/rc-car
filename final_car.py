#==================================================================================
# Program: Self-Driving RC Car Project
# Programmer: Austin Buhler
# Class: CIRC 224
# Date: December 5, 2019
#----------------------------------------------------------------------------------
# Sources
#----------------------------------------------------------------------------------
# OpenCV, NumPy, PiCamera, Time, and PiGPIO Libraries used
#==================================================================================

import cv2
import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray
from time import sleep
import pigpio

#-------------------------------------------------------------------------------

#PiCamera Setup

#-------------------------------------------------------------------------------
WIDTH = 320
HEIGHT = 240
cam = PiCamera()
cam.resolution = (WIDTH, HEIGHT)
cam.framerate = 30
raw_capture = PiRGBArray(cam, size=(WIDTH, HEIGHT))
#-------------------------------------------------------------------------------

#Set up esc and steering servo constants and variables

#-------------------------------------------------------------------------------
ESC = 18 #GPIO for ESC (Motor)
pi = pigpio.pi();
pi.set_servo_pulsewidth(ESC, 0)
STOP = 1500 #Pulsewidth for stop - in microseconds
FORWARD = 1400 #Pulsewidth for moving forward - in microseconds
speed = 1400 #Current Speed pulsewidth- in microseconds

STEER = 13 #GPIO for steering servo
MID = 1400 #Pulsewidth for steering straight - in microseconds
LEFT = 1050 #Pulsewidth for full left - in microseconds
RIGHT = 1750 #Pulsewidth for full right - in microseconds
direction = MID #Current Direction - in microseconds
#-------------------------------------------------------------------------------

#Function Definitions

#-------------------------------------------------------------------------------
def get_canny(img):
	low_thresh = 60
	high_thresh = 180
	
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #convert to grayscale
	blur = cv2.GaussianBlur(gray, (5, 5), 0) #blur pixels slightly
	canny = cv2.Canny(blur, low_thresh, high_thresh) #convert to binary image
	return canny
#-------------------------------------------------------------------------------
def get_area(img):
	rect = np.array([[(0, 240), (0, 100), (320, 100), (320, 240)]])
	mask = np.zeros_like(img)
	cv2.fillPoly(mask, rect, 255) #Put white rectangle over region of interest on image
	masked_img = cv2.bitwise_and(img, mask) #Logic AND, only shows white pixels in reg of interest
	return masked_img
#-------------------------------------------------------------------------------
def average_lines(image, lines):
    global direction
	left = [] #Lines on left of screen
	right = [] #Lines of right of screen
	lanes = []
	if lines != None:
		for line in lines:
			x1, y1, x2, y2 = line.reshape(4) #Get the 2 points of each line
			if x1 == x2: #code will crash
				continue
			equation = np.polyfit((x1, x2), (y1, y2), 1) #Gives slope and y intercept
			slope = equation[0]
			intercept = equation[1]
			if x1 < WIDTH*5/8 and x2 < WIDTH*5/8: #Left 5/8 of screen 
				left.append((slope, intercept))
			elif x1 > WIDTH*3/8 and x2 > WIDTH*3/8: #Right 5/8 of screen 
				right.append((slope, intercept))
				
		left_average = np.average(left, axis=0) #Find the average slope and intercept for each line
		right_average = np.average(right, axis=0)
		
		try:
			if len(left) > 0:
				left_line = get_line(image, left_average) #Makes left line out of the average slope and intercept
				lanes.append(left_line)
			if len(right) > 0:
				right_line = get_line(image, right_average) #Makes right line out of the average slope and intercept
				lanes.append(right_line)
		except Exception: #Unable to make line
			direction = MID	
	return np.array(lanes)
#-------------------------------------------------------------------------------
def get_line(image, line_specs):
	slope, intercept = line_specs
	slope = round(slope, 8) #Round to 8 decimal places
	intercept = round(intercept, 8)
	y1 = image.shape[0] #line starts at bottom of image
	y2 = int(y1*(2/5)) # goes up to 2/5 of way DOWN image
	x1 = int((y1-intercept)/slope) #From y=mx+b (Slope intercept form)
	x2 = int((y2-intercept)/slope) 
	return np.array([x1, y1, x2, y2])
#-------------------------------------------------------------------------------
def find_dir(lines):
	global direction
	global speed
	gain1 = 480 #Gain for 1 line
	gain2a = 6 #Gain for 2 lines (close to center)
	gain2b = 12 #Gain for 2 lines (far from center)
	gain2c = 30 #Gain for 2 lines (farthest from center)
	
	if len(lines) == 2: #If there are 2 lines found
		left = lines[0][2] #left x2 point
		right = lines[1][2] #right x2 point
		dist_from_center = ((left + right) / 2) - (WIDTH / 2) #Calculate cars distance from the center of the lines
		if abs(dist_from_center) < 25: #If car is close to center, lower gain
			direction = MID + (dist_from_center * gain2a)
			speed = FORWARD
		elif abs(dist_from_center) < 35: #If car is close to center, lower gain
			direction = MID + (dist_from_center * gain2b)
			speed = FORWARD
		else: #If car is farther from center, higher gain to get back to center faster
			direction = MID + (dist_from_center * gain2c) 
			speed = FORWARD + 5
	elif len(lines) == 1: #If only 1 line is found
		x1, y1, x2, y2 = lines[0]
		slope = (y2 - y1) / (x2 - x1)
		direction = MID - (slope * gain1) #Find direction based on the slope of the line
		speed = FORWARD + 5
	else: #No lines found
		direction = MID
		speed = STOP #Don't move
			
	if direction < LEFT: #If the calculations put the steering out of range, make it the max for that direction
		direction = LEFT
	elif direction > RIGHT:
		direction = RIGHT
#-------------------------------------------------------------------------------
def show_lines(line_img, lines):
	if lines != None:
		for x1, y1, x2, y2 in lines:
			cv2.line(line_img, (x1, y1), (x2, y2), (0, 255, 0), 5) #Draw green lines over the current frame
	return line_img
#-------------------------------------------------------------------------------
def drive():
	pi.set_servo_pulsewidth(STEER, direction) #Set steering direction
	pi.set_servo_pulsewidth(ESC, speed) #Set speed

#-------------------------------------------------------------------------------
	
#Loop for processing each frame (Main)

#-------------------------------------------------------------------------------
for frame in cam.capture_continuous(raw_capture, format='bgr', use_video_port=True):
	img = frame.array #Current frame
		
	canny = get_canny(img) #Black and white image
	area = get_area(canny) #Trim the frame for faster processing
	lines = cv2.HoughLinesP(area, 1, np.pi/180, 50, np.array([]), 50, 10) #find all lines in image
	avg_lines = average_lines(img, lines) #Average the lines found into 1 or 2 lane lines
	find_dir(avg_lines) #Find direction based on the average lines
	line_img = show_lines(img, avg_lines) #Draw out the 2 lines
	drive()	#Set speed and steering
	
	cv2.imshow("lines", line_img) #Preview window
	key = cv2.waitKey(1) & 0xFF #Key input
	raw_capture.truncate(0) #Reset frame
	
	if key == ord("p"):
		pi.set_servo_pulsewidth(STEER, MID) #Reset straight
		pi.set_servo_pulsewidth(ESC, STOP) #Neutral
		break #End the loop, ending the program

#-------------------------------------------------------------------------------

cv2.destroyAllWindows() #Close the windows
