import sys
import numpy as np
sys.path.append("../../controllers")
from mybot import MyBot

######################
# Define Human robot #
######################
humanbot = MyBot(robot="e-puck", reset=False)
humanbot.initializeSensors(gps=True, imu=True, lidar=True)
humanbot.initializeDevices(display=True)
humanbot.log.setLevel(30)

######################
# Path of the Human #
######################
waypoints = [[0.5,0],[0.5,-0.5],[-0.2,-0.5],[-0.2,0.5]]
humanbot.setPath(waypoints)

######################################
# The main human robot control loop #
######################################
while humanbot.step(humanbot.timestep) != -1:
    # Follow the path
    _, controls = humanbot.forwardKinematics_0()
    if(humanbot.followPath(show=True)):
        humanbot.pauseSimulation()
        break