import sys
import numpy as np
sys.path.append("../../controllers")
from mybot import MyBot
from points import get_points

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
waypoints = ([[1,-0.1],[1.8,-0.4],[2.5,-0.7],[2,-3],[3,-4],[3.5,-5],[3.5,-6]])
wayppoints=get_points(waypoints)
humanbot.setPath(waypoints,show=True)

######################################
# The main human robot control loop #
######################################
while humanbot.step(humanbot.timestep) != -1:
    # Follow the path
    _, controls = humanbot.forwardKinematics_0()
    if(humanbot.followPath()):
        humanbot.pauseSimulation()
        break