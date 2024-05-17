from controller import Supervisor, Display, Keyboard
import numpy as np
import sys
import cv2
import logging
import math
import matplotlib.pyplot as plt

from spatialmath import base

logging.basicConfig(level=logging.INFO, format="%(levelname)s :: %(message)s")

class Machine:
    def __init__(self) -> None:
        self.__s = {"IDLE": self.__idle}
        self.curr_state = "IDLE"

    def __idle(self):
        print(sys._getframe().f_code.co_name)

    def __call__(self, states):
        self.__s = states
        self.curr_state = list(self.__s.keys())[0]

    @property
    def state(self):
        return self.curr_state

    @state.setter
    def state(self, state):
        if state not in self.__s:
            raise Exception("Value not in STATES")
        print(f"Entering Machine State: {state}")
        self.curr_state = state

    def run(self):
        self.__s[self.curr_state]()


characteristics = {
    "e-puck": {
        "Wheel Radius": 20.0,  # mm
        "Axle Length": 57.0,  # mm
        "Max Rotation": 6.28,
        "Max Velocity": 32,
        "Precision": 0.001,
        "Controller": "UNCYCLE",
    },
    "pioneer": {
        "Wheel Radius": 97.5,  # mm
        "Axle Length": 340.0,  # mm
        "Max Rotation": 12.3,
        "Max Velocity": 512,
        "Precision": 0.01,
        "Controller": "UNCYCLE",
    },
}


class MyBot(Supervisor, Machine):
    def __init__(self, robot, pose=[0, 0, 0], reset=True):
        super().__init__()
        self.log = logging.getLogger()
        self.timestep = int(self.getBasicTimeStep())
        self.counter = 0
        # self.timestep = 16
        self.super = self.getSelf()
        self.charecteristics = characteristics[robot]
        self.rotation = self.charecteristics["Max Rotation"]
        self.velocity = self.charecteristics["Max Velocity"]
        self.precision = self.charecteristics["Precision"]
        # reset physics
        if reset:
            self.reset()
        # variabes
        self.pose = pose
        self.goal = [0, 0]
        self.controls = [0, 0]
        self.dest = None
        self.path = None
        self.objects = {}
        self.halt = False
        # private
        self.__teleop_motion = [0, 0]
        self.__prev_enc = [0.000001, 0.000001]

        self.machine = Machine()

    # call funciton (very pythony)
    def __call__(self, camera=True, lidar=True):
        print("MYBOT")

    def reset(self):
        self.simulationResetPhysics()
        self.simulationReset()
        self.step(self.timestep)
        self.enableMotors()
        self.enableEncoders()
        self.step(self.timestep)

    def pause(self):
        self.simulationSetMode(self.SIMULATION_MODE_PAUSE)

    def play(self):
        self.simulationSetMode(self.SIMULATION_MODE_REAL_TIME)

    # initialize robot devices
    def initializeDevices(
        self,
        motors=True,
        keyboard=False,
        display=False,
    ):
        self.step(self.timestep)
        if motors:
            self.enableMotors()
        if keyboard:
            self.enableKeyboard()
        if display:
            self.enableDisplay()

    # initialize robot sensors
    def initializeSensors(
        self,
        position=True,
        encoders=True,
        camera=False,
        lidar=False,
        range=False,
        gps=False,
        imu=False,
        gyro=False,
        accel=False,
        compass=False,
        proxi=False,
        light=False,
        ground=False,
    ):
        # internal start
        self.step(self.timestep)
        # initialize precise location
        if position:
            self.pose = [
                self.super.getPosition()[0],
                self.super.getPosition()[1],
                np.arctan2(
                    self.super.getOrientation()[3], self.super.getOrientation()[0]
                ),
            ]
            self.hist = np.array([[self.pose[0], self.pose[1]]])
        if encoders:
            self.enableEncoders()
        if camera:
            self.enableCamera()
        if lidar:
            self.enableLidar()
        if range:
            self.enableRange()
        if gps:
            self.enableGPS()
        if imu:
            self.enableIMU()
        if gyro:
            self.enableGyro()
        if accel:
            self.enableAccel()
        if compass:
            self.enableCompass()
        if proxi:
            self.enableProximitySensor()
        if light:
            self.enableLightSensor()
        if ground:
            self.enableGroundSensor()

    # initialize motors
    def enableMotors(self):
        self.step(self.timestep)
        self.leftMotor = self.getDevice("left wheel")
        self.rightMotor = self.getDevice("right wheel")
        self.leftMotor.setPosition(float("+inf"))
        self.rightMotor.setPosition(float("+inf"))
        self.leftMotor.setVelocity(0.0)
        self.rightMotor.setVelocity(0.0)

    # enable encoders
    # offset issue -> https://stackoverflow.com/questions/61150174/reset-webots-position-sensor
    def enableEncoders(self, names=["left wheel sensor"]):
        self.step(self.timestep)
        self.leftEncoder = self.getDevice("left wheel sensor")
        self.leftEncoder.enable(self.timestep)
        self.leftEncoder_offset = self.leftEncoder.getValue()
        self.rightEncoder = self.getDevice("right wheel sensor")
        self.rightEncoder.enable(self.timestep)
        self.rightEncoder_offset = self.rightEncoder.getValue()

    # enable lidar
    def enableLidar(self, name="lidar"):
        self.lidar = self.getDevice(name)
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()

    # enable GPS
    def enableGPS(self, name="gps"):
        self.gps = self.getDevice(name)
        self.gps.enable(self.timestep)

    # enable compass
    def enableCompass(self, name="compass"):
        self.compass = self.getDevice(name)
        self.compass.enable(self.timestep)

    # enable IMU
    def enableIMU(self, name="inertial unit"):
        self.imu = self.getDevice(name)
        self.imu.enable(self.timestep)

    # enable accelerometer
    def enableAccel(self, name="accelerometer"):
        self.accel = self.getDevice(name)
        self.accel.enable(self.timestep)

    # enable gyro
    def enableGyro(self, name="gyro"):
        self.gyro = self.getDevice(name)
        self.gyro.enable(self.timestep)

    # enable camera
    def enableCamera(self, name="camera1"):
        self.camera = self.getDevice(name)
        self.camera.enable(self.timestep)
        self.camera.recognitionEnable(self.timestep)

    # enable range
    def enableRange(self, name="range-finder"):
        self.range = self.getDevice(name)
        self.range.enable(self.timestep)

    # enable ground sensor
    def enableGroundSensor(self, name="gs"):
        self.ground = []
        for i in range(3):
            self.ground.append(self.getDevice(name + str(i)))
            self.ground[i].enable(self.timestep)

    # enable proximity sensors
    def enableProximitySensor(self, name="ps"):
        self.ps = []
        for i in range(8):
            self.ps.append(self.getDevice(name + str(i)))
            self.ps[i].enable(self.timestep)

    # enable light sensors
    def enableLightSensor(self, name="ls"):
        self.ls = []
        for i in range(8):
            self.ls.append(self.getDevice(name + str(i)))
            self.ls[i].enable(self.timestep)

    # enable display
    def enableDisplay(self, name="map"):
        self.display = self.getDevice(name)

    # shome image to display
    def showDisplay(self, image_path):
        maper = cv2.rotate(
            cv2.flip(cv2.imread(image_path), 1), cv2.ROTATE_90_COUNTERCLOCKWISE
        )
        imageRef = self.display.imageNew(
            width=self.display.getWidth(),
            height=self.display.getHeight(),
            data=maper.tolist(),
            format=Display.RGB,
        )
        self.display.imagePaste(imageRef, 0, 0)

    # enable keyboard
    def enableKeyboard(self):
        self.keyboard = Keyboard()
        self.keyboard.enable(self.timestep)

    # teleop
    def teleop(
        self, update_kinematics=False, reset_rotation=True, reset_velocity=False
    ):
        key = self.keyboard.getKey()
        if key != -1:
            if key == 32:
                self.__teleop_motion = [0, 0]
            elif key == Keyboard.RIGHT:
                self.__teleop_motion[1] = self.__teleop_motion[1] - 1
            elif key == Keyboard.LEFT:
                self.__teleop_motion[1] = self.__teleop_motion[1] + 1
            elif key == Keyboard.DOWN:
                self.__teleop_motion[0] = self.__teleop_motion[0] - 5
            elif key == Keyboard.UP:
                self.__teleop_motion[0] = self.__teleop_motion[0] + 5
        vr, vl = self.inverseKinematics(
            self.__teleop_motion[0], self.__teleop_motion[1]
        )
        self.setMotorSpeed(vr, vl)
        self.__teleop_motion[0] = 0 if reset_velocity else self.__teleop_motion[0]
        self.__teleop_motion[1] = 0 if reset_rotation else self.__teleop_motion[1]
        if update_kinematics:
            self.forwardKinematics_0()

    # wait for keypress
    def wait_keyboard(self):
        while self.keyboard.getKey() != ord("Y"):
            super().step(self.__timestep)

    # set motor speed
    def setMotorSpeed(self, right_speed, left_speed):
        self.leftMotor.setVelocity(left_speed)
        self.rightMotor.setVelocity(right_speed)

    # spawn an object/box and add to object list
    #def spawnObject(
    #    self, pose=[1, 1], color=[1, 0, 0], name="ball", size=0.06, recognition=True
    #):
    #    root_node = self.getRoot()
    #    rec_col = (
    #        f"recognitionColors [ {color[0]} {color[1]} {color[2]} ]"
    #        if recognition
    #        else ""
    #    )
    #    children_field = root_node.getField("children")
    #    def_name = f"DEF {name} Solid {{ \
    #                    translation {pose[0]} {pose[1]} {size/2} \
    #                    children [ \
    #                        Shape {{ \
    #                        appearance Appearance {{ \
    #                            material Material {{ \
    #                            diffuseColor {color[0]} {color[1]} {color[2]} \
    #                            }} \
    #                        }} \
    #                        geometry Box {{ \
    #                            size {size} {size} {size} \
    #                        }} \
    #                        }} \
    #                    ] \
    #                    {rec_col} \
    #                    }}"
    #    children_field.importMFNodeFromString(-1, def_name)
    #    obj_node = self.getFromDef(name)
    #    self.objects[name] = obj_node
    #    return obj_node

    # spawn an object/box and add to object list
    def spawnObject(self, pose=[1, 1], color=[1, 0, 0], name='ball', size=[0.05, 0.05, 0.05], recognition=True):
        root_node = self.getRoot()
        rec_col = f"recognitionColors [ {color[0]} {color[1]} {color[2]} ]" if recognition else ""
        children_field = root_node.getField('children')
        def_name = f"DEF {name} Solid {{ \
                        translation {pose[0]} {pose[1]} {size[2]/2} \
                        children [ \
                            Shape {{ \
                                appearance Appearance {{ \
                                    material Material {{ \
                                        diffuseColor {color[0]} {color[1]} {color[2]} \
                                    }} \
                                }} \
                                geometry Box {{ \
                                    size {size[0]} {size[1]} {size[2]} \
                                }} \
                            }} \
                        ] \
                        {rec_col} \
                        }}"
        children_field.importMFNodeFromString(-1, def_name)
        obj_node = self.getFromDef(name)
        self.objects[name] = obj_node
        return obj_node

    def spawnFloorplan(self, floorplan, size_arena):
        # Get the shape of the floorplan
        rows, cols = floorplan.shape
        # Calculate the size of cells in the floorplan in world coordinates
        cell_h = size_arena[1] / rows
        cell_w = size_arena[0] / cols
        self.cell_size = np.array([cell_w, cell_h])
        for i in range(rows):
            for j in range(cols):
                if floorplan[i,j]>0:
                    self.spawnObject(name="obj", pose=((j+.5)*cell_w,(i+.5)*cell_h),size=[cell_w,cell_h,0.1], color=[0,0,1])

    def spawnLandmarks(self, landmarks):
        # Spawn landmarks in Webots
        for i in range(landmarks.shape[1]):
            self.spawnObject(
                pose=landmarks[:, i],
                color=[0, 1, 1],
                name=f"lm_{i}",
                size = [0.05, 0.05, 0.2],
                recognition=False,
            )
    
    # add an existing object to objects list
    def addObject(self, name):
        obj_node = self.getFromDef(name)
        self.objects[name] = obj_node
        return obj_node

    # set goal
    # set goal
    def setGoal(self, goto, show=False):
        self.goal = goto
        self.log.info(f"goal  -->\tx: {goto[0]}, \ty: {goto[1]}")
        if(show):
            self.spawnObject(name="wp", pose=goto, size=[0.02,0.02,0.02], color=[1,0,0])

    # go to goal
    def gotoGoal(self, precision=0.05, stop=False):
        finished = False
        # calculate heading to goal
        preheading = math.atan2((self.goal[1]-self.pose[1]), (self.goal[0]-self.pose[0]))
        heading = preheading - self.pose[2]
        heading_corrected = round((math.atan2(math.sin(heading), math.cos(heading))), 4)
        #calculate distance to goal 
        distance = ((self.pose[0] - self.goal[0])**2 + (self.pose[1] - self.goal[1])**2)**0.5
        vel = self.inverseKinematics(64, 2*heading_corrected)
        self.setMotorSpeed(vel[0], vel[1])
        if distance < precision:
            self.log.info(f"GOAL AT CORDINATES {self.goal} REACHED")
            if stop: self.setMotorSpeed(0, 0)
            finished = True
        return distance, heading_corrected, finished

    def stopRobot(self, stop_loop=True):
        self.setMotorSpeed(0, 0)
        if stop_loop:
            self.halt = True

    def inverseKinematics(self, vel, rot):
        r = self.charecteristics["Wheel Radius"]
        l = self.charecteristics["Axle Length"]
        # rot = np.clip(rot, -(np.pi), np.pi)
        vr = (vel + (l / 2) * rot) / r
        vl = (vel - (l / 2) * rot) / r
        return vr, vl

    def forwardKinematics_0(self, update=True):
        self.log.info(f"---- {self.counter} ----")
        x = self.super.getPosition()[0]
        y = self.super.getPosition()[1]
        o = self.super.getOrientation()
        o = np.arctan2(o[3], o[0])
        self.log.info(
            f"pose  -->  \tx: {round(x, 3)}, \ty: {round(y,3)}, \tθ: {round(np.degrees(o), 0)}"
        )
        state = [x, y, o]
        if update:
            self.controls = [
                np.sqrt(
                    (state[0] - self.pose[0]) ** 2 + (state[1] - self.pose[1]) ** 2
                ),
                state[2] - self.pose[2],
            ]
            self.hist = np.append(
                self.hist, np.array([[self.pose[0], self.pose[1]]]), axis=0
            )
            self.pose = state
        return state, self.controls

    def forwardKinematics_1(self, update=True, imu=True):
        self.log.info(f"---- {self.counter} ----")
        x = self.gps.getValues()[0]
        y = self.gps.getValues()[1]
        if imu:
            o = self.imu.getRollPitchYaw()[2]
        else:
            x = self.compass.getValues()[0]
            y = self.compass.getValues()[1]
            radians = np.arctan2(y, x)
            degrees = 360 - ((radians * 180) / np.pi - 90) % 360
            o = np.radians(degrees)
        self.counter += 1
        self.log.info(
            f"pose  -->  \tx: {round(x, 3)}, \ty: {round(y,3)}, \tθ: {round(np.degrees(o), 0)}"
        )
        state = [x, y, o]
        if update:
            self.controls = [
                np.sqrt(
                    (state[0] - self.pose[0]) ** 2 + (state[1] - self.pose[1]) ** 2
                ),
                state[2] - self.pose[2],
            ]
            self.hist = np.append(
                self.hist, np.array([[self.pose[0], self.pose[1]]]), axis=1
            )
            self.pose = state
        return state, self.controls

    def forwardKinematics_2(self, update=True, delta=1):
        self.counter += 1
        self.log.info(f"---- {self.counter} ----")
        length = self.charecteristics["Axle Length"] / 1000
        radius = self.charecteristics["Wheel Radius"] / 1000
        right_encoder = self.rightEncoder.getValue() - self.rightEncoder_offset
        left_encoder = self.leftEncoder.getValue() - self.leftEncoder_offset
        __curr_enc = np.asarray([right_encoder, left_encoder]) * radius
        difference = __curr_enc - self.__prev_enc
        vel = (difference[0] + difference[1]) / 2
        rot = (difference[0] - difference[1]) / length
        self.controls = [vel, rot]
        o = self.pose[2] + (rot * delta)
        x = self.pose[0] + (vel * np.cos(o) * delta)
        y = self.pose[1] + (vel * np.sin(o) * delta)
        self.log.info(
            f"pose  -->  \tx: {round(x, 3)}, \ty: {round(y,3)}, \tθ: {round(np.degrees(o), 0)}"
        )
        state = [x, y, o]
        if update:
            self.controls = [
                np.sqrt(
                    (state[0] - self.pose[0]) ** 2 + (state[1] - self.pose[1]) ** 2
                ),
                state[2] - self.pose[2],
            ]
            self.hist = np.append(
                self.hist, np.array([[self.pose[0], self.pose[1]]]), axis=1
            )
            self.pose = state
        self.__prev_enc = __curr_enc
        return state, self.controls

    def spin(self, direction: str, angle=90, halt=False, delta=5, update_state=False):
        if direction == "left":
            self.setMotorSpeed(self.rotation / delta, -(self.rotation / delta))
        else:
            self.setMotorSpeed(-(self.rotation / delta), self.rotation / delta)
        state = self.pose
        while self.step(self.timestep) != -1:
            if update_state:
                self.forwardKinematics_0()
            i = np.subtract(self.forwardKinematics_0(), state)
            if abs(i[2]) > np.radians(angle):
                if halt:
                    self.halt = True
                self.setMotorSpeed(0, 0)
                break

    def move(self, direction: str, dist=0.5, halt=False, delta=2, update_state=False):
        if direction == "forward":
            self.setMotorSpeed(self.rotation / delta, self.rotation / delta)
        else:
            self.setMotorSpeed(-(self.rotation / delta), -(self.rotation / delta))
        state = self.pose
        while self.step(self.timestep) != -1:
            if update_state:
                self.forwardKinematics_0()
            i = np.subtract(self.forwardKinematics_0(), state)
            if np.hypot(i[0], i[1]) > dist:
                if halt:
                    self.halt = True
                self.setMotorSpeed(0, 0)
                break

    def getGlobalPoint(self, degrees, distance):
        # local cordinates
        lo = np.radians(-degrees - 180)
        lx = distance * np.cos(lo)
        ly = distance * np.sin(lo)
        # global cordinates
        x, y, o = self.pose
        local_cordinates = np.array([lx, ly])
        rotation_matrix = np.array([[np.cos(o), -np.sin(o)], [np.sin(o), np.cos(o)]])
        robot_cordinates = np.array([x, y])
        global_cordinates = (
            np.matmul(rotation_matrix, local_cordinates) + robot_cordinates
        )
        return global_cordinates

    def f(self, x, odo, v=None):
        r"""
        State transition function

        :param x: vehicle state :math:`(x, y, \theta)`
        :type x: array_like(3), ndarray(n,3)
        :param odo: vehicle odometry :math:`(\delta_d, \delta_\theta)`
        :type odo: array_like(2)
        :param v: additive odometry noise, defaults to (0,0)
        :type v: array_like(2), optional
        :return: predicted vehicle state
        :rtype: ndarray(3)

        Predict the next state based on current state and odometry
        value.  ``v`` is a random variable that represents additive
        odometry noise for simulation purposes.

        .. math::

            f: \vec{x}_k, \delta_d, \delta_\theta \mapsto \vec{x}_{k+1}

        For particle filters it is useful to apply the odometry to the
        states of all particles and this can be achieved if ``x`` is a 2D
        array with one state per row.  ``v`` is ignored in this case.

        .. note:: This is the state update equation used for EKF localization.

        :seealso: :meth:`deriv` :meth:`Fx` :meth:`Fv`
        """
        odo = base.getvector(odo, 2)

        if isinstance(x, np.ndarray) and x.ndim == 2:
            # x is Nx3 set of vehicle states, do vectorized form
            # used by particle filter
            dd, dth = odo
            theta = x[:, 2]
            return (
                np.array(x)
                + np.c_[
                    dd * np.cos(theta), dd * np.sin(theta), np.full(theta.shape, dth)
                ]
            )
        else:
            # x is a vector
            x = base.getvector(x, 3)
            dd, dth = odo
            theta = x[2]

            if v is not None:
                v = base.getvector(v, 2)
                dd += v[0]
                dth += v[1]

            return x + np.r_[dd * np.cos(theta), dd * np.sin(theta), dth]

    def Fx(self, x, odo):
        r"""
        Jacobian of state transition function df/dx

        :param x: vehicle state :math:`(x, y, \theta)`
        :type x: array_like(3)
        :param odo: vehicle odometry :math:`(\delta_d, \delta_\theta)`
        :type odo: array_like(2)
        :return: Jacobian matrix
        :rtype: ndarray(3,3)

        Returns the Jacobian matrix :math:`\frac{\partial \vec{f}}{\partial \vec{x}}` for
        the given state and odometry.

        :seealso: :meth:`f` :meth:`deriv` :meth:`Fv`
        """
        dd, dth = odo
        theta = x[2]

        # fmt: off
        J = np.array([
                [1,   0,  -dd * np.sin(theta)],
                [0,   1,   dd * np.cos(theta)],
                [0,   0,   1],
            ])
        # fmt: on
        return J

    def Fv(self, x, odo):
        r"""
        Jacobian of state transition function df/dv

        :param x: vehicle state :math:`(x, y, \theta)`
        :type x: array_like(3)
        :param odo: vehicle odometry :math:`(\delta_d, \delta_\theta)`
        :type odo: array_like(2)
        :return: Jacobian matrix
        :rtype: ndarray(3,2)

        Returns the Jacobian matrix :math:`\frac{\partial \vec{f}}{\partial \vec{v}}` for
        the given state and odometry.

        :seealso:  :meth:`f` :meth:`deriv` :meth:`Fx`
        """
        dd, dth = odo
        theta = x[2]

        # fmt: off
        J = np.array([
                [np.cos(theta),    0],
                [np.sin(theta),    0],
                [0,           1],
            ])
        # fmt: on
        return J

    def plot_xy(self, *args, block=False, **kwargs):
        """
        Plot xy-path from history

        :param block: block until plot dismissed, defaults to False
        :type block: bool, optional
        :param args: positional arguments passed to :meth:`~matplotlib.axes.Axes.plot`
        :param kwargs: keyword arguments passed to :meth:`~matplotlib.axes.Axes.plot`


        The :math:`(x,y)` trajectory from the simulation history is plotted as
        :math:`x` vs :math:`y.

        :seealso: :meth:`run` :meth:`plot_xyt`
        """
        if args is None and "color" not in kwargs:
            kwargs["color"] = "b"
        xyt = self.hist
        plt.plot(xyt[:, 0], xyt[:, 1], *args, **kwargs)
        plt.show(block=block)

 
    def setPath(self, waypoints, show=False):
        self.waypoints = waypoints.copy()
        self.nxt_wp, self.waypoints = self.waypoints[0], self.waypoints[1:]
        self.setGoal(self.nxt_wp, show=show)

    def followPath(self, d_th=0.05, show=False):
        distance,_,_ = self.gotoGoal(d_th)
        if distance < d_th:
            if len(self.waypoints) < 1:
                self.log.info(f"FINAL DESTINATION REACHED")
                return(True)
            
            self.log.info(f"HEADING FOR THE NEXT WAYPOINT")
            self.nxt_wp, self.waypoints = self.waypoints[0], self.waypoints[1:]
            self.setGoal(self.nxt_wp, show=show)
            

        return(False)

    def pauseSimulation(self):
        self.simulationSetMode(0)
        self.log.info("Simulation paused")

    def startSimulation(self):
        self.simulationSetMode(1)
        self.log.info("Simulation started")

