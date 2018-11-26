"""
Author: Wooden Jin
E-mail: yongbinjin@zju.edu.cn
this script is used to control the biped robot with two leg (six DOF)
real time control
"""
import tinyik as ik
import numpy as np
from scipy import interpolate
import msvcrt
from RGM.rgm.RGM_network import RgmNetwork
from time import sleep, time
import logging
from math import pi


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


def trajectory_produce(h0, a, b, period):
    """
    This is used to produce a trajectory function for leg robot
    :param h0:
    :param a:
    :param b:
    :param period:
    :return:
    """

    # ============================================
    # define the robot kinematic model
    thigh_length = 0.353  # length of the thigh, the static parameter
    shank_length = 0.350  # length of the shank, the static parameter

    # suppose leg1 is the right leg, supporting leg
    # suppose leg2 is the left leg, swinging leg
    leg1 = ik.Actuator(['y', [0, 0, -thigh_length], 'y', [0, 0, -shank_length]])
    leg1.angles = [-0.01, 0.01]  # init the configuration of the leg1

    leg2 = ik.Actuator(['y', [0, 0, -thigh_length], 'y', [0, 0, -shank_length]])
    leg2.angles = [-0.01, 0.01]  # init the configuration of the leg2

    sample_num = 10  # number of the sampling points in half cycle
    # the first half cycle
    leg1_aim_x = np.linspace(0, -b, sample_num)
    leg1_aim_y = np.zeros(sample_num)
    leg1_aim_z = np.ones(sample_num) * -h0
    leg1_aim = np.stack((leg1_aim_x, leg1_aim_y, leg1_aim_z), axis=-1)
    leg1_angle = np.zeros((sample_num, 2))
    theta_temp = np.linspace(0, pi, sample_num)
    curve_x = a * np.sin(theta_temp)
    curve_y = -b * np.cos(theta_temp)

    leg2_aim_x = leg1_aim_x + curve_y
    leg2_aim_y = leg1_aim_y
    leg2_aim_z = leg1_aim_z + curve_x
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
    angle_control[:, 0:3] = -angle_control[:, 0:3]
    angle_control[:, 0] = -angle_control[:, 0]
    angle_control[:, 3] = -angle_control[:, 3]
    temp = np.copy(angle_control[:, 0:3])
    angle_control[:, 0:3] = np.copy(angle_control[:, 3:6])
    angle_control[:, 3:6] = np.copy(temp)
    angle_control = angle_control / pi * 180

    global tck_leg1_hip, tck_leg1_knee, tck_leg1_ankle, tck_leg2_hip, tck_leg2_knee, tck_leg2_ankle

    # interpolation
    time_array = np.linspace(0, period, sample_num * 2)
    tck_leg1_hip = interpolate.splrep(time_array, angle_control[:, 0], s=0)
    tck_leg1_knee = interpolate.splrep(time_array, angle_control[:, 1], s=0)
    tck_leg1_ankle = interpolate.splrep(time_array, angle_control[:, 2], s=0)
    tck_leg2_hip = interpolate.splrep(time_array, angle_control[:, 3], s=0)
    tck_leg2_knee = interpolate.splrep(time_array, angle_control[:, 4], s=0)
    tck_leg2_ankle = interpolate.splrep(time_array, angle_control[:, 5], s=0)

    return tck_leg1_hip, tck_leg1_knee, tck_leg1_ankle, tck_leg2_hip, tck_leg2_knee, tck_leg2_ankle


def get_current_path(t):
    """
    return current time angle trajectory point
    :param t:
    :return:
    """
    global tck_leg1_hip, tck_leg1_knee, tck_leg1_ankle, tck_leg2_hip, tck_leg2_knee, tck_leg2_ankle
    left_hip = interpolate.splev(t, tck_leg1_hip, der=0)
    left_knee = interpolate.splev(t, tck_leg1_knee, der=0)
    left_ankle = interpolate.splev(t, tck_leg1_ankle, der=0)
    right_hip = interpolate.splev(t, tck_leg2_hip, der=0)
    right_knee = interpolate.splev(t, tck_leg2_knee, der=0)
    right_ankle = interpolate.splev(t, tck_leg2_ankle, der=0)
    return np.array([left_hip, left_knee, left_ankle, right_hip, right_knee, right_ankle])


global tck_leg1_hip, tck_leg1_knee, tck_leg1_ankle, tck_leg2_hip, tck_leg2_knee, tck_leg2_ankle


def main():
    # prepare the parameter needed for trajectory_produce()
    h0 = 0.65  # init robot height
    period = 1.5  # total motion period
    step = 0.02  # time step
    a = 0.02  # short axis of the ellipse trajectory
    b = 0.015  # long axis of the ellipse trajectory
    global tck_leg1_hip, tck_leg1_knee, tck_leg1_ankle, tck_leg2_hip, tck_leg2_knee, tck_leg2_ankle
    tck_leg1_hip, tck_leg1_knee, tck_leg1_ankle, tck_leg2_hip, tck_leg2_knee, tck_leg2_ankle \
        = trajectory_produce(h0, a, b, period)

    # =======================================================
    RgmNet = RgmNetwork()  # create an can net for motor control
    id = [1, 2, 3, 4, 5, 6]  # the motor's id in the can network
    RgmNet.add_node(id, 'config.ini')  # add the id in the network

    # change the CAN network state
    # initialize the control word and control state
    RgmNet.control_state(id, 'RESET FAULT')  # reset
    RgmNet.control_state(id, 'SHUT DOWN')  # close every thing
    RgmNet.control_state(id, 'SWITCH 0N')  # add the power but not enable

    # init the motor joint
    angle_init = [153.33, 214.19, 149.56, 192.24, 146.61, 220.25]  # get current angle
    angle_path_init = np.array(angle_init) + get_current_path(0)

    motion_flag = False  # a flag to note whether motion is allowed
    init_flag = True  # init flag

    while True:
        # using the keyboard to control the motion state
        """ [enter] means start to move, [space] means quick stop, [esc] means quit the code"""
        if msvcrt.kbhit():
            temp = ord(msvcrt.getch())

            if temp == 27:
                # [esc] = 27
                print("quit program [real_time_biped_walk.py]")
                RgmNet.control_state(id, 'QUICK STOP')
                RgmNet.control_state(id, "DISABLE VOLTAGE")
                break

            elif temp == 32:
                print("stop the interaction, press [ENTER] to continue")
                RgmNet.control_state(id, 'QUICK STOP')
                motion_flag = False
                """stop the operation"""
                pass

            elif temp == 13:
                print("Continue the interaction, press [SPACE] to stop")
                RgmNet.control_state(id, 'ENABLE OPERATION')
                RgmNet.operation_mode('CYCLIC SYNCHRONOUS POSITION')
                RgmNet.syn_pos_init(id)
                motion_flag = True
                start = time()
                sleep(0.1)
                """continue"""
                pass

            elif temp == 0:
                pass

            else:
                logging.warning("illegal keyboard pressed, please check!")

        if motion_flag:

            if init_flag:
                angle = get_angle(RgmNet, id)
                angle_current = np.array(
                    [angle[str(id[0])], angle[str(id[1])], angle[str(id[2])],
                     angle[str(id[3])], angle[str(id[4])], angle[str(id[5])]])
                angle_step = (angle_current - angle_path_init) / 10

                for i in range(10):
                    angle_control = angle_current - i * angle_step
                    RgmNet.syn_pos_ctrl(id, angle_control)
                    sleep(0.05)
                init_flag = False
                start = time()

            now = time() - start
            angle_control = get_current_path(now % period)
            RgmNet.syn_pos_ctrl(id, list(angle_control+angle_init))


if __name__ == '__main__':
    main()
