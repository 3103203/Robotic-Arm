import time
from adafruit_servokit import ServoKit

# Set the PWM frequency of the PCA9685 to 50Hz
kit = ServoKit(channels=16)
kit.frequency = 50
angle[5]={90,90,90,90,90}
state=0

# Define the minimum and maximum pulse lengths for the servos
SERVO_MIN = 150  # Min pulse length out of 4096
SERVO_MAX = 600  # Max pulse length out of 4096

# Function to set servo angle
def set_servo_angle(servo, angle):
    pulse_width = int(SERVO_MIN + (angle / 180) * (SERVO_MAX - SERVO_MIN))
    kit.servo[servo].angle = angle
    kit.servo[servo].set_pulse_width_range(SERVO_MIN, SERVO_MAX)

try:
    switch (state):
	case 0:
	  if(angle[0]>45):
		angle[0]--
		set_servo_angle(1,angle[0])
		time.sleep(0.05)
	  else:
		state=1	
	case 1:
	  if(angle[1]>15):
		angle[1]--
		set_servo_angle(2,angle[1])
		time.sleep(0.05)
	  else:
		state=2
	case 2:
	  if(angle[2]>15):
		angle[2]--
		set_servo_angle(3,angle[2])
		time.sleep(0.05)
	  else:
		state=3
	case 3:
	  if(angle[3]>5):
		angle[3]--
		set_servo_angle(4,angle[3])
		time.sleep(0.05)
	  else:
		state=4		
	case 4:
	  if(angle[3]<80):
		angle[3]++
		set_servo_angle(1,angle[0])
		time.sleep(0.05)
	  else:
		state=5
	case 5:
	   if(angle[0]<90):
		angle[0]++
		set_servo_angle(1,angle[0])
		time.sleep(0.05)
	  else:
		state=6	
	case 6:
	  if(angle[1]<90):
		angle[1]++
		set_servo_angle(2,angle[1])
		time.sleep(0.05)
	  else:
		state=7
	case 7:
	  if(angle[2]<90):
		angle[2]++
		set_servo_angle(3,angle[2])
		time.sleep(0.05)
	  else:
		state=8
	
	case 8:
	  if(angle[4]>40):
		angle[4]--
		set_servo_angle(0,angle[4])
		time.sleep(0.05)
	  else:
		state=9
	case 9:	
	   if(angle[0]>45):
		angle[0]--
		set_servo_angle(1,angle[0])
		time.sleep(0.05)
	  else:
		state=10	
	case 10:
	   if(angle[1]>15):
		angle[1]--
		set_servo_angle(2,angle[1])
		time.sleep(0.05)
	  else:
		state=11
	case 11:
	  if(angle[2]>15):
		angle[2]--
		set_servo_angle(3,angle[2])
		time.sleep(0.05)
	  else:
		state=12
	case 12:
	  if(angle[3]>5):
		angle[3]--
		set_servo_angle(4,angle[3])
		time.sleep(0.05)
	  else:
		state=13		
	case 13:
	  if(angle[3]<80):
		angle[3]++
		set_servo_angle(1,angle[0])
		time.sleep(0.05)
	  else:
		state=14
	case 14:
	   if(angle[0]<90):
		angle[0]++
		set_servo_angle(1,angle[0])
		time.sleep(0.05)
	  else:
		state=14	
	case 15:
	  if(angle[1]<90):
		angle[1]++
		set_servo_angle(2,angle[1])
		time.sleep(0.05)
	  else:
		state=16
	case 16:
	  if(angle[2]<90):
		angle[2]++
		set_servo_angle(3,angle[2])
		time.sleep(0.05)
	  else:
		state=17
	case 17:
	  if(angle[4]>90):
		angle[4]--
		set_servo_angle(0,angle[4])
		time.sleep(0.05)
	  else:
		state=0
			

except KeyboardInterrupt:
    pass


import cv2
import numpy as np

# Initialize the webcam
cap = cv2.VideoCapture(1)

while True:
    # Read the frame from the webcam
    ret, frame = cap.read()

    # Convert the frame from BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the lower and upper bounds for the red color
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])

    # Define the lower and upper bounds for the green color
    lower_green = np.array([50, 100, 100])
    upper_green = np.array([70, 255, 255])

    # Threshold the HSV image to get only red and green colors
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    # Find contours in the masks
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw bounding boxes and label the objects as "red" or "green"
    for contour in contours_red:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.putText(frame, "red", (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    for contour in contours_green:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, "green", (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display the frame with bounding boxes and labels
    cv2.imshow('Object Detection', frame)

    # Exit on 'q' press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam
cap.release()

# Close all OpenCV windows
cv2.destroyAllWindows()