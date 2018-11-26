import pybullet as p
import pybullet_data
import os
import numpy as np
from math import pi
from time import sleep


class BipedRobot(object):
    JOINT_ID = [9, 12, 14, 3, 6, 8, 1]  # revolute joint id

    # [right_hip, right_knee, right_ankle, left_hip, left_knee, left_ankle, upper]

    def __init__(self, step=0.01, is_gui=True):
        """
        init the pybullet simulation environment
        :param step:  time step for pybullet simulation
        :param is_gui:  flag, true, connect the gui mode, false, direct mode
        """
        # connect the client
        if is_gui:
            self.physicsClient = p.connect(p.GUI, options="--opengl3")  # connect the gui client
        else:
            self.physicsClient = p.connect(p.DIRECT)  # connect the direct mode

        # add the ground into the simulation environment
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF to load the plane
        self.planeId = p.loadURDF("plane.urdf")
        p.setAdditionalSearchPath(os.getcwd())
        self.biped_robot = p.loadURDF("Real_robot.urdf", [0, 0, 1], p.getQuaternionFromEuler([0, 0, 0]))
        # self.biped_robot = p.loadURDF("biped_robot_mirror.urdf", init_pos, p.getQuaternionFromEuler([0, 0, 0]))
        p.setGravity(0, 0, -10)  # set the gravity of the simulation environment
        # p.setGravity(0, 0, 0)  # set the gravity of the simulation environment

        self.step = step  # set the time step, the default value is 0.01s
        p.setTimeStep(self.step)
        self.index = 0  # index increase every time simulation, and if index == frequency, clear to zero

    def initialize(self, init_pos, init_angle):
        """
        init the robot in the simulation environment
        :param init_pos: [x,y,z], initial base origin center
        :param init_angle: init motor angle of the robot joint
        :return: return nothing
        """
        self.index = 0
        init_angle = init_angle / 180 * pi
        p.resetBasePositionAndOrientation(self.biped_robot, init_pos, p.getQuaternionFromEuler([0, 0, 0]))
        for i in range(init_angle.size):
            # init_angle.type should be narray
            p.resetJointState(self.biped_robot, self.JOINT_ID[i], init_angle[i])
        sleep(0.1)

    def run(self, angle_control):
        """
        control the joint angle one time
        :param angle_control: angle array, [right_hip, right_knee, right_ankle, left_hip, left_knee, left_ankle, {upper}]
        :return: nothing
        """
        angle_control = angle_control / 180 * pi
        p.setJointMotorControlArray(self.biped_robot, self.JOINT_ID[0:angle_control.size],
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=angle_control)
        p.stepSimulation()
        sleep(self.step)

    def stop(self):
        pass
