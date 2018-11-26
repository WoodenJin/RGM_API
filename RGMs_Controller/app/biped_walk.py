"""
this script is used to control the biped robot with two leg (six degree)
test right leg's motion property
"""
import tinyik as ik
import numpy as np
from scipy import interpolate
import msvcrt
from RGM.rgm.RGM_network import RgmNetwork
from time import sleep
import logging


PI = 3.1415926


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


def find_index(id, RgmNet, angle_control):
    angle = get_angle(RgmNet, id)
    angle_current = np.array(
        [angle[str(id[0])], angle[str(id[1])], angle[str(id[2])],
         angle[str(id[3])], angle[str(id[4])], angle[str(id[5])]])
    temp_min = 100000000
    index_min = 0
    N = len(angle_control)
    print("N is ", N)
    for i in range(int(N / 2)):
        temp = np.linalg.norm(angle_current - angle_control[i])
        if temp < temp_min:
            index_min = i
            temp_min = temp
    print("closest point in trajectory is :")
    print(angle_control[index_min])

    return index_min


def Interpolation(x0, y0, period, step):
    """
    this function is used to interpolate the control angle
    :param x0: sample point, x coordinate
    :param y0: sample point, y coordinate
    :param period:  total time of the action
    :param step:  time step
    :return:  return the detailed control angle
    """
    t = interpolate.splrep(x0, y0)
    N = int(period / 2 / step)
    x = np.linspace(0, period / 2, N)
    y = interpolate.splev(x, t)
    return y


def trajectory_produce(h0, a, b, period, step):
    """
    This is used to produce a cycle trajectory for leg robot
    :param h0: h0 is the hip height of the leg robot
    :param r0: r0 is the cycle radius of foot
    :param time0: time0 is the time period
    :return: return a trajectory of the leg

    """
    thigh_length = 0.353  # length of the thigh, the static parameter
    shank_length = 0.350  # length of the shank, the static parameter

    # check whether the parameter is reasonable
    # r_limit = sqrt(h0 ** 2 + (thigh_length + shank_length) ** 2)
    # if r0 > r_limit:
    #     r0 = r_limit
    # else:
    #     pass

    # suppose leg1 is right leg, supporting leg
    # suppose leg2 is left leg, swinging leg
    leg1 = ik.Actuator(['y', [0, 0, -thigh_length], 'y', [0, 0, -shank_length]])
    leg1.angles = [-0.01, 0.01]  # init the configuration of the leg1

    leg2 = ik.Actuator(['y', [0, 0, -thigh_length], 'y', [0, 0, -shank_length]])
    leg2.angles = [-0.01, 0.01]  # init the configuration of the leg2

    sample_num = 10  # number of the sampling points in half cycle
    # first half cycle
    leg1_aim_x = np.linspace(0, -b, sample_num)
    # leg1_aim_y = np.linspace(0, r0, sample_num)
    leg1_aim_y = np.zeros(sample_num)
    leg1_aim_z = np.ones(sample_num) * -h0
    leg1_aim = np.stack((leg1_aim_x, leg1_aim_y, leg1_aim_z), axis=-1)
    leg1_angle = np.zeros((sample_num, 2))

    theta_temp = np.linspace(0, PI, sample_num)
    # curve_x = r0 * np.sin(theta_temp)
    # curve_y = -r0 * np.cos(theta_temp)
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

    # print(leg1_angle)
    # print(leg2_angle)
    leg1_angle1 = Interpolation(np.linspace(0, period / 2, sample_num), leg1_angle[:, 0], period, step)
    leg1_angle2 = Interpolation(np.linspace(0, period / 2, sample_num), leg1_angle[:, 1], period, step)
    leg2_angle1 = Interpolation(np.linspace(0, period / 2, sample_num), leg2_angle[:, 0], period, step)
    leg2_angle2 = Interpolation(np.linspace(0, period / 2, sample_num), leg2_angle[:, 1], period, step)
    leg1_angle = np.stack((leg1_angle1, leg1_angle2), axis=-1)
    leg2_angle = np.stack((leg2_angle1, leg2_angle2), axis=-1)

    leg1_hip = leg1_angle[:, 0]
    leg1_knee = leg1_angle[:, 1]
    leg1_ankle = -(leg1_angle[:, 0] + leg1_angle[:, 1])
    leg2_hip = leg2_angle[:, 0]
    leg2_knee = leg2_angle[:, 1]
    leg2_ankle = -(leg2_angle[:, 0] + leg2_angle[:, 1])
    angle_control = np.stack((leg1_hip, leg1_knee, leg1_ankle, leg2_hip, leg2_knee, leg2_ankle), axis=-1)
    # angle_control = np.stack((-leg1_hip, -leg1_knee, leg1_ankle, -leg2_hip, leg2_knee, leg2_ankle), axis=-1)
    # print(angle_control.shape)
    # angle_control_2 = np.stack((angle_control[:, 3:6], angle_control[:, 0:3]), axis=2)
    angle_control_2 = np.hstack((angle_control[:, 3:6], angle_control[:, 0:3]))
    # print(angle_control_2.shape)
    # angle_control = np.stack((angle_control, angle_control_2), axis=0)
    angle_control = np.vstack((angle_control, angle_control_2))
    # angle_control[:, 3:6] = -angle_control[:, 3:6]
    angle_control[:, 0:3] = -angle_control[:, 0:3]
    angle_control[:, 0] = -angle_control[:, 0]
    angle_control[:, 3] = -angle_control[:, 3]
    temp = np.copy(angle_control[:, 0:3])
    angle_control[:, 0:3] = np.copy(angle_control[:, 3:6])
    angle_control[:, 3:6] = np.copy(temp)
    print(angle_control)
    return angle_control


def main():
    # prepare the parameter needed for trajectory_produce()
    h0 = 0.65  # init robot height
    period = 0.8  # total motion period
    step = 0.02  # time step
    a = 0.015  # short axis of the ellipse trajectory
    b = 0.025  # long axis of the ellipse trajectory
    res_time = 0.5  # delay time, between the leg change
    angle_path = trajectory_produce(h0, a, b, period, step)  # get the motion trajectory
    angle_path = np.rad2deg(angle_path)
    # print(angle_path)

    # =====================================================
    # init the robot
    RgmNet = RgmNetwork()  # create an can net for motor control
    # id = [4, 5, 6, 1, 2, 3]  # the motor's id in the CAN network
    id = [1, 2, 3, 4, 5, 6]  # the motor's id in the CAN network
    RgmNet.add_node(id, 'config.ini')  # add the id into the network

    # change the CAN network state
    # initialize the control word and control state
    RgmNet.control_state(id, 'RESET FAULT')  # reset
    RgmNet.control_state(id, 'SHUT DOWN')  # close every thing
    RgmNet.control_state(id, 'SWITCH 0N')  # add the power but not enable

    # according to the hardware to change this value
    # angle_init = [157.31, 190.36, 178.94, 188.99, 169.39, 201.85]  # get current angle
    angle_init = [153.33, 214.19, 149.56, 193.95, 146.61, 222.56]  # get current angle

    N = angle_path.shape[0]
    angle_control = []
    for i in range(N):
        angle_control.append(
            [angle_init[0] + angle_path[i][0],
             angle_init[1] + angle_path[i][1],
             angle_init[2] + angle_path[i][2],
             angle_init[3] + angle_path[i][3],
             angle_init[4] + angle_path[i][4],
             angle_init[5] + angle_path[i][5]])
        print(angle_control[i])
    # print(angle_control)

    print("press ENTER to start")
    motion_flag = 0  # a flag to note whether motion is allowed
    direction_flag = 1  # determine the motion direction/ up or down
    index = 0  # note current point index
    index = find_index(id, RgmNet, angle_control)

    while True:
        # using the keyboard to control the motion state
        """ [enter] means start to move, [space] means quick stop, [esc] means quit the code"""
        if msvcrt.kbhit():
            temp = ord(msvcrt.getch())

            if temp == 27:
                # [esc] = 27
                print("quit program [app_multi_interaction.py]")
                RgmNet.control_state(id, 'QUICK STOP')
                RgmNet.control_state(id, 'DISABLE VOLTAGE')
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
                sleep(0.1)
                """continue"""
                pass

            elif temp == 0:
                pass

            else:
                logging.warning("illegal keyboard pressed, please chack!")

        if motion_flag:
            if index == N:
                # motion_flag = False
                # RgmNet.control_state(id, 'QUICK STOP')
                # RgmNet.control_state(id, 'DISABLE VOLTAGE')
                # break
                # direction_flag = 0
                index = 0
                sleep(res_time)
            elif index == int(N / 2):
                sleep(res_time)

            elif index == 0:
                direction_flag = 1
            else:
                pass

            # angle1 = angle_init[0] - angle_path[index][1]
            # angle2 = angle_init[1] - angle_path[index][0]
            # angle1 = angle_control[index][0]
            # angle2 = angle_control[index][1]
            # RgmNet.syn_pos_ctrl(id, [angle1, angle2])
            RgmNet.syn_pos_ctrl(id, angle_control[index])
            if direction_flag:
                index += 1
            else:
                index -= 1
            # sleep(step-0.003)
            # sleep(0.5)


if __name__ == '__main__':
    main()
