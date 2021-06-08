# Semi-Autonomous Car Project
# Overview
The Semi-Autonomous Car was a computer interfacing project from my third semester in Computer Engineering Technology at Sask Polytech. A picture of the project is in the 'Pictures' folder.

This project uses a Vaterra Kemora RC Car with a Raspberry Pi and PiCamera mounted to it. The PiCamera is used to view the "road" the car is on. A Python program uses OpenCV to detect the lines on the road. Calculations are performed based on the position and slope of the lines. Finally, the PWM output of the GPIO pins are changed to control the steering and speed of the car accordingly. The road used for the demo was an oval-shaped track of white tape on black posterboard.

# Software:
Once the Python program starts, base values are set on the GPIO pins for the speed and steering. Then a loop begins that gets a frame from the camera stream, manipulates the frame, finds lines in the frame, and modifies the steering and direction based on the position and slope of the lines. This loop continues until the program is stopped.
