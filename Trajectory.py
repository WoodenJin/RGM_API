import tinyik as ik
import numpy as np
from math import *
from scipy import interpolate
import matplotlib.pyplot as plt
from cycler import cycler


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
        # np.savetxt('test2.txt', angle_control)
        # mapping to the real robot configuration
        angle_control[:, 0] = -angle_control[:, 0]
        angle_control[:, 4] = -angle_control[:, 4]
        angle_control[:, 5] = -angle_control[:, 5]
        # angle_control[:, 3] = -angle_control[:, 3]
        angle_control = angle_control / pi * 180
        self.angle_control = angle_control  # store in the self class

        # interpolation
        time_array = np.linspace(0, period, sample_num * 2)
        self.tck = []
        # tck = [right_hip, right_knee, right_ankle, left_hip, left_knee, left_ankle, upper_body]
        # self.tck_left_hip = interpolate.splrep(time_array, angle_control[:, 0], s=0)
        for i in range(6):
            self.tck.append(interpolate.splrep(time_array, angle_control[:, i], s=0))
        self.tck.append(
            interpolate.splrep(time_array,
                               self.amp / pi * 180 * np.sin(2 * pi * time_array / self.period + self.phase),
                               s=0))

    def get_control_angle(self, current_time):
        """
        input current_time, return the control angle
        :param current_time: current_time in one period
        :return: control_angle unit is degree
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
        # print(control_angle)
        return control_angle

    def display(self):
        custom_cycler = (cycler(color=['r', 'r', 'r', 'g', 'g', 'g']) +
                         cycler(lw=[1, 2, 3, 1, 2, 3]) +
                         cycler(linestyle=['-', '--', ':', '-', '--', ':']))
        plt.rc('axes', prop_cycle=custom_cycler)
        fig, (ax) = plt.subplots(nrows=1)
        # for i in range(6):
        #     ax.plot(np.transpose(self.angle_control[:, i]))
        ax.plot(self.angle_control)
        # ax.set_prop_cycle(custom_cycler)
        plt.show()
