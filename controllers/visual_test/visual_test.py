import numpy as np
import cv2
from controller import Robot,Motor
import matplotlib.pyplot as plt
import sys
import torch 

# Load YOLOv5 model
model = torch.hub.load('ultralytics/yolov5', 'yolov5x', pretrained=True)

# Initialize Values
MAX_SPEED=6.28  # Max Rotational Velocity of Motors
# Initialize Webots robot
robot = Robot()

# Initialize the Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

def motor_control(left_speed,right_speed):
    leftMotor.setVelocity(left_speed)
    rightMotor.setVelocity(right_speed)

# Get the camera device
camera = robot.getDevice("camera")
camera.enable(10)  # Enable the camera with a refresh rate of 10 ms

# Placeholder functions for turning the robot right and left
def turn_right():
    # Implement code to turn the robot right in Webots
    left_speed = 0.36*MAX_SPEED
    right_speed = 0.24*MAX_SPEED
    motor_control(left_speed,right_speed)

def turn_left():
    # Implement code to turn the robot left in Webots
    left_speed = 0.24*MAX_SPEED
    right_speed =0.36*MAX_SPEED
    motor_control(left_speed,right_speed)

def go_straight():
    left_speed = 0.45*MAX_SPEED
    right_speed = 0.45*MAX_SPEED
    motor_control(left_speed,right_speed)
 
 
 
writer = cv2.VideoWriter("output.avi",cv2.VideoWriter_fourcc(*"MJPG"), 30,(1280,720))
# Function to detect human using YOLOv5
def detect_human(image):
    # Perform inference
    results = model(image)
   
    cood=results.xyxy[0].cpu().numpy()
    if len(cood)==0:
        return False, None
    cood=cood[0]
    x=cood[0]
    y=cood[1]
    w=cood[2]
    h=cood[3]
  
    #Rectanglle around the detected 
    cv2.rectangle(image, (int(x), int(y)), (int(w), int(h)), (255, 0, 0), 2)
    cv2.imshow("result",image)
    
    writer.write(image)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        writer.release()
        return

    if len(cood) > 0:

        # Get center position of the first detected human
        x_center = int((w + x) / 2) # Mid-Point Formula
        y_center = int((h + y) / 2)
        return True, (x_center, y_center),image  # Return center position of the detected human

    else:
        return False, None,image  # No human detected


# Main loop
while robot.step(10) != -1:  # Run the loop every 10 ms
    image=camera.getImage()
    # Convert image to a NumPy array
    image = np.frombuffer(image, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)  # Convert to BGR format for OpenCV
    
    try:
        # Process the frame to detect human
        human_detected, human_position,result = detect_human(image)
        
        
        if not human_detected:
            motor_control(0,0)
        else:
            dir=(image.shape[1]/2)-human_position[0] #To calculate the direction in whicch robot should turn
                
            if dir<45 and dir>60:
                go_straight()
            if dir<=45:#dir_thres:
                turn_right()
            elif dir>=60:#dir_thres:
                turn_left()
            else:
                pass
    except:
        pass
    