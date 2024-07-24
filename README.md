It is the Project Work for the Course AI5153 - Mobile Robotics at IIT Hyderabad

This project introduces a novel human-following robot designed to autonomously track and follow human
The  robot integrates sensors including vision based cameras
The human following robot holds significant promise for applications in service robotics , surveillance and assistance for individuals with  mobility challenges paving the way for safer and more intuitive human-robot interactions in the future 

In recent years, the field of robotics has witnessed rapid advancements, leading to the development of robots capable of performing complex tasks and interacting with humans in various environments
One such area of research that has gained considerable attention is the design and implementations of human - following robot
The robot needs a mechanism which can make decision for it to take actions accordingly so that the task can be performed  correctly 
The ability of a robot to autonomously follow a human has significant implications for enhancing human - robot collaboration and interaction  


# Methods and materials
- The robot consist of a camera which can detect and identify humans by using human detection algorithm
- Once the human is detect a boundary around the human will be created by which all the parameters required for robot  motion is acquired
- A threshold will be set for the boundary size of the human within the camera frame . Based on the boundary size around the human ,the robot longitudinal  motion will be determined
  - The robot moves forward if the size of the boundary is less than the threshold value.
  - And the robot moves backward if the size of the boundary is greater than the threshold value
- Meanwhile the lateral motion or the yaw moment of the robot is determined by the offset between the centroid of the boundary created and the frame center. If the offset is towards right the controller minimize the error which is  offset and turns the robot towards right.

# Simulation

The Simulation in Webots utilizes the following pre defined objects:

- **TurtleBot3**: TurtleBot3 Burger is a small , affordable , programmable, ROS-based mobile robot for use  education , research and product prototyping. Raspberry camera was integrated with this robot as a sensor.
* **E-puck**: E-puck is a small differential wheeled robot it was originally designed for micro engineering education. It was mounted by a human pedestrian by which the whole human model moved within  the simulation.
* **Raspberry camera**: Used as Camera.
* **Skin**: It was used as a dummy Human.
## Hardware
* Processor: Intel Core i7 12700H
* GPU: Nvidia RTX 4060 Mobile GPU
## Software
- Webots R2022b
  - e-Puck Robot
  - Skin human model
  - Turtle Bot3 Burger
  - JetBotRaspberry Pi Camera
- Python 3.10
  - Numpy
  - OpenCV
  - YoloV5x
  - Scipy
  - PyTorch

# Result
The result of the simulation can be seen in the "Results.avi" file.
