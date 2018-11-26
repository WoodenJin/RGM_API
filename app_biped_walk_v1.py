"""
this script is used to control the biped robot with two leg and one torso (pendulum)
Author: Wooden Jin
e-mail: yongbinjin@zju.edu.cn
address: Zhejiang University
"""

from time import sleep, time
import tinyik as ik
import numpy as np
from math import *
from scipy import interpolate
from RGMs_Controller.RGM.rgm.RGM_network import RgmNetwork
import pybullet as p
import pybullet_data
import os
import random


class Robot_Time(object):

    def __init__(self, period=1.0, sample_num=30, flag_real_time=True):
        """
        init the Robot_TIme system
        :param period:  period for single motion cycle
        :param sample_num:  sample_num for single motion cycle
        :param flag_real_time: flag, true, using real time mode. false, step time mode
        :output nothing
        """
        self.period = period
        self.sample_num = sample_num
        self.flag = flag_real_time

        self._current_time = 0
        self._current_index = 0
        self._start_stamp = 0

    def start(self):
        """
        this function can start or restart recording time
        :return: nothing
        """
        self._start_stamp = time()
        self._current_time = 0
        self._current_index = 0

    def current(self):
        """
        this time return current time in one period
        :return: current_time
        """
        current_time = 0
        if self.flag:
            current_time = (time() - self._start_stamp) % self.period
        else:
            self._current_index += 1
            self._current_index = self._current_index % self.sample_num
            current_time = self._current_index / self.sample_num * self.period
        return current_time


class Trajectory(object):
    # static parameter
    thigh_length = 0.353  # length of thigh
    shank_length = 0.350  # length of shank

    def __init__(self, h0, a, b, period, amp=0, phase=0, flag_up=False):
        """
        produce tarjectory for robot
        :param h0: the height of hip
        :param a: lift foot height
        :param b: move foot length
        :param period: period for single
        """
        self.height = h0
        self.lift = a
        self.move = b
        self.period = period
        self.amp = amp
        self.phase = phase
        self.flag = flag_up

        # ================================
        # start calculate the trajectory
        # suppose leg1 is the supporting leg
        # suppose leg2 is the swing leg
        # define the leg configuration
        leg1 = ik.Actuator(['y', [0, 0, -self.thigh_length], 'y', [0, 0, -self.shank_length]])
        leg1.angles = [-0.01, 0.01]  # init the configuration of the leg1

        leg2 = ik.Actuator(['y', [0, 0, -self.thigh_length], 'y', [0, 0, -self.shank_length]])
        leg2.angles = [-0.01, 0.01]  # init the configuration of the leg2

        sample_num = 10  # number of the sampling points in half cycle
        # the first half cycle
        leg1_aim_x = np.linspace(0, -self.move, sample_num)
        leg1_aim_y = np.zeros(sample_num)
        leg1_aim_z = np.ones(sample_num) * -self.height
        leg1_aim = np.stack((leg1_aim_x, leg1_aim_y, leg1_aim_z), axis=-1)
        leg1_angle = np.zeros((sample_num, 2))

        theta_temp = np.linspace(0, pi, sample_num)
        curve_z = self.lift * np.sin(theta_temp)
        curve_x = -self.move * np.cos(theta_temp)
        leg2_aim_x = leg1_aim_x + curve_x
        leg2_aim_y = leg1_aim_y
        leg2_aim_z = leg1_aim_z + curve_z
        leg2_aim = np.stack((leg2_aim_x, leg2_aim_y, leg2_aim_z), axis=-1)
        leg2_angle = np.zeros((sample_num, 2))

        for i in range(sample_num):
            leg1.ee = leg1_aim[i, :]
            leg1_angle[i, :] = leg1.angles
            leg2.ee = leg2_aim[i, :]
            leg2_angle[i, :] = leg2.angles

        leg1_angle = np.stack((leg1_angle[:, 0], leg1_angle[:, 1]), axis=-1)
        leg2_angle = np.stack((leg2_angle[:, 0], leg2_angle[:, 1]), axis=-1)
        leg1_hip = leg1_angle[:, 0]
        leg1_knee = leg1_angle[:, 1]
        leg1_ankle = -(leg1_angle[:, 0] + leg1_angle[:, 1])
        leg2_hip = leg2_angle[:, 0]
        leg2_knee = leg2_angle[:, 1]
        leg2_ankle = -(leg2_angle[:, 0] + leg2_angle[:, 1])
        angle_control = np.stack((leg1_hip, leg1_knee, leg1_ankle, leg2_hip, leg2_knee, leg2_ankle), axis=-1)

        # the second half cycle
        angle_control_2 = np.hstack((angle_control[:, 3:6], angle_control[:, 0:3]))
        # total period
        angle_control = np.vstack((angle_control, angle_control_2))

        # mapping to the real robot configuration
        angle_control[:, 0] = -angle_control[:, 0]
        angle_control[:, 4] = -angle_control[:, 4]
        angle_control[:, 5] = -angle_control[:, 5]
        angle_control = angle_control / pi * 180

        # interpolation
        time_array = np.linspace(0, period, sample_num * 2)
        self.tck = []
        # tck = [right_hip, right_knee, right_ankle, left_hip, left_knee, left_ankle, upper_body]
        # self.tck_left_hip = interpolate.splrep(time_array, angle_control[:, 0], s=0)
        for i in range(6):
            self.tck.append(interpolate.splrep(time_array, angle_control[:, i], s=0))
        self.tck.append(
            interpolate.splrep(time_array,
                               self.amp * np.sin(2 * pi * time_array / self.period + self.phase),
                               s=0))

    def get_control_angle(self, current_time):
        """
        input current_time, return the control angle
        :param current_time: current_time in one period
        :return: control_angle
        """

        if self.flag:
            # control the upper body
            control_angle = np.zeros(7)
        else:
            # do not control the upper body
            control_angle = np.zeros(6)
        for i in range(control_angle.size):
            # get the current control angle
            control_angle[i] = interpolate.splev(current_time, self.tck[i], der=0)
        return control_angle


class BipedRobot(object):
    JOINT_ID = [1, 3, 6, 8, 9, 12, 14]

    def __init__(self, init_pos, step=0.01, is_gui=True):
        """
        init the BipedRobot simulation object
        :param IsGUI: bool, True, open the graphical interface, False, do not open the graphical interface
        """
        # connect the client
        if is_gui:
            self.physicsClient = p.connect(p.GUI, options="--opengl3")
        else:
            self.physicsClient = p.connect(p.DIRECT)

        # add the ground into the simulation environment
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF to load the plane
        self.planeId = p.loadURDF("plane.urdf")
        p.setAdditionalSearchPath(os.getcwd())
        self.biped_robot = p.loadURDF("Real_robot.urdf", init_pos, p.getQuaternionFromEuler([0, 0, 0]))
        # self.biped_robot = p.loadURDF("biped_robot_mirror.urdf", init_pos, p.getQuaternionFromEuler([0, 0, 0]))
        p.setGravity(0, 0, -10)  # set the gravity of the simulation environment
        self.step = step  # set the time step, the default value is 0.01s
        p.setTimeStep(self.step)
        self.index = 0  # index increase every time simulation, and if index == frequency, clear to zero
        self.init_pos = init_pos

    def take_action(self, angle_control):
        """
        this method is used to drive the biped robot, control mode is position control
        :param angle_control: a angle list, [left_hip, left_knee, left_ankle, right_hip, right_knee, right_ankle]
        :return: none
        """
        self.index += 1
        p.setJointMotorControlArray(self.biped_robot, self.JOINT_ID,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=angle_control)
        p.stepSimulation()
        sleep(self.step)

    def reset_joint_state(self, init_angle):
        """
        this is used to reset the biped robot joint
        :param init_angle: initial joint angle
        :return: none
        """
        self.index = 0
        for i in range(len(self.JOINT_ID)):
            p.resetJointState(self.biped_robot, self.JOINT_ID[i], init_angle[i])

    def attack_define(self, frequency, number, strength):
        self.attack_frequency = frequency
        self.attack_number = number
        self.attack_strength = strength

    def single_attack(self):
        V = 3
        robot_base_position = p.getBasePositionAndOrientation(self.biped_robot)
        robot_base_position = robot_base_position[0]
        robot_base_position = [robot_base_position[0] + self.init_pos[0],
                               robot_base_position[1] + self.init_pos[1],
                               robot_base_position[2] + self.init_pos[2] - 0.3]
        Velocity = [random.random() * 5 - 2.5, random.random() * 5 - 2.5, random.random() * 5 - 2.5]
        mass = self.attack_strength / sqrt(Velocity[0] ** 2 + Velocity[1] ** 2 + Velocity[2] ** 2)
        mass = mass / V
        sphereRadius = 0.02 * random.randint(2, 4)
        colSphereId = p.createCollisionShape(p.GEOM_SPHERE, radius=sphereRadius)
        visualShapeId = -1
        # basePosition = robot_base_position + Velocity
        basePosition = [robot_base_position[0] + Velocity[0],
                        robot_base_position[1] + Velocity[1],
                        robot_base_position[2] + Velocity[2]]
        baseOrientation = [0, 0, 0, 1]
        sphereUid = p.createMultiBody(mass, colSphereId, visualShapeId, basePosition, baseOrientation)
        p.changeDynamics(sphereUid, -1, spinningFriction=0.001, rollingFriction=0.001, linearDamping=0.0,
                         contactStiffness=10000, contactDamping=0)
        p.resetBaseVelocity(sphereUid,
                            [-V * Velocity[0] * random.uniform(0.8, 1.2),
                             -V * Velocity[1] * random.uniform(0.8, 1.2),
                             -V * Velocity[2] * random.uniform(0.8, 1.2)])

    def take_action_with_sphere_attack(self, angle_control):
        self.index += 1
        p.setJointMotorControl2(self.biped_robot, 0, controlMode=p.POSITION_CONTROL,
                                targetPosition=angle_control[0])  # left hip joint
        p.setJointMotorControl2(self.biped_robot, 2, controlMode=p.POSITION_CONTROL,
                                targetPosition=angle_control[1])  # left knee joint
        p.setJointMotorControl2(self.biped_robot, 4, controlMode=p.POSITION_CONTROL,
                                targetPosition=angle_control[2])  # left ankle joint
        p.setJointMotorControl2(self.biped_robot, 6, controlMode=p.POSITION_CONTROL,
                                targetPosition=angle_control[3])  # right hip joint
        p.setJointMotorControl2(self.biped_robot, 8, controlMode=p.POSITION_CONTROL,
                                targetPosition=angle_control[4])  # right knee joint
        p.setJointMotorControl2(self.biped_robot, 10, controlMode=p.POSITION_CONTROL,
                                targetPosition=angle_control[5])  # right ankle joint

        if not (self.index % self.attack_frequency):
            self.index = 0  # clear index to zero
            for i in range(self.attack_number):
                self.single_attack()

        p.stepSimulation()
        sleep(self.step)


class BipedRobotControl(object):
    """
    control the robot in simulation or real world
    """
    angle_init = [153.33, 214.19, 149.56, 192.24, 146.61, 220.25, 180]  # the default motor position

    def __init__(self, Nodeid, h0, a, b, period, amp=0, phase=0, flag_up=False, flag_real_time=False, flag_real=False):
        """
        init the biped robot control system
        :param Nodeid is the motor in the can network
        :param h0: h0 is the height of hip
        :param a: a is the lift height of foot
        :param b: b is the move forward length
        :param period: the period time for one cycle
        :param amp: swing amp
        :param phase: phase difference between the leg and torso
        :param flag_up: whether control the upper body, true, control up, false, do not control
        :param flag_real_time: step simulation or real time simulation, true, real time. false, step simulation
        :param flag_real: true, real robot control, false, control robot in simulation
        """
        self.Time = Robot_Time(period=period, flag_real_time=flag_real_time)
        self.trajectory = Trajectory(h0, a, b, period, amp=amp, phase=phase, flag_up=flag_up)
        self.Nodeid = Nodeid
        self.flag_real = flag_real

        if flag_real:
            # init the real robot control
            self.RgmNet = RgmNetwork()  # create an can net for motor control
            self.RgmNet.add_node(self.Nodeid, 'config.ini')  # add the motor id in the can network
            # change the can network state
            self.RgmNet.control_state(self.Nodeid, 'RESET FAULT')  # reset fault
            self.RgmNet.control_state(self.Nodeid, 'SHUT DOWN')  # close every thing
            self.RgmNet.control_state(self.Nodeid, 'SWITCH 0N')  # add the power but not enable
            angle_path_init = np.array(self.angle_init) + self.trajectory.get_control_angle(0)

            self.motion_flag = False  # a flag to note whether motion is allowed
            self.init_flag = True  # init flag

        else:
            # init the simulation environment
            init_pos = [0, 0, h0 + 0.033]
            self.robot = BipedRobot(init_pos, step=period / self.Time.sample_num, is_gui=True)
            self.robot.reset_joint_state(self.trajectory.get_control_angle(0))

    def start(self):
        """
        start robot control
        :return: nothing
        """
        if self.flag_real:
            angle = self.get_angle(self.RgmNet, self.Nodeid)

    def get_angle(RgmNet, id):
        """
        this parameter is used to get the angle of the motor
        :param RgmNet: the can network object
        :param id: motor's id
        :return:
        """
        # angle = []
        angle = {}
        para = RgmNet.get_motion_para(id)
        for i in range(len(id)):
            temp_para = para[str(id[i])]
            angle.update({str(id[i]): temp_para[0]})
            # angle.append(temp_para[0])
            print("the angle of id %d is %f" % (id[i], temp_para[0]))
        return angle


def main():
    RT = Robot_Time(flag_real_time=True)
    RT.start()
    while True:
        print(RT.current())
        sleep(0.2)


if __name__ == "__main__":
    main()
