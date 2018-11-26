"""
this script is used to control the right leg
test right leg's motion property
"""
import tinyik as ik
import numpy as np
from scipy import interpolate
import msvcrt
from RGM.rgm.RGM_network import RgmNetwork
from time import sleep
import logging


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
    N = int(period / step)
    x = np.linspace(0, period, N)
    y = interpolate.splev(x, t)
    return y


def trajectory_produce(h0, period, step):
    """
    this function is used to produce the right leg's motion trajectory
    :param h0: the initial height
    :param period: the time period for realize this trajectory
    :param step: time step
    :return: trajectory angle and joint velocity to realize this trajectory
    """
    thigh_length = 0.353  # length of the thigh, the static parameter
    shank_length = 0.350  # length of the shank, the static parameter
    leg = ik.Actuator(['y', [0, 0, -thigh_length], 'y', [0, 0, -shank_length]])  # def the leg's configuration
    leg.angles = [-0.01, 0.01]  # init the configuration of the leg
    sample_num = 10  # number of the sampling points in half cycle

    leg_aim_x = np.zeros(sample_num)  # leg end-point's trajectory_x, keep static
    leg_aim_y = np.zeros(sample_num)  # leg end-point's trajectory_y, keep static
    leg_aim_z = np.linspace(-h0, -(thigh_length + shank_length), sample_num)  # leg end-point's trajectory_x
    leg_aim = np.stack((leg_aim_x, leg_aim_y, leg_aim_z), axis=-1)  # combine the x,y,z component
    leg_angle = np.zeros((sample_num, 2))  # angle buffer
    for i in range(sample_num):
        leg.ee = leg_aim[i, :]
        leg_angle[i, :] = leg.angles
    leg_angle = np.vstack((leg_angle[::-1], leg_angle))
    leg_angle1 = Interpolation(np.linspace(0, period, 2 * sample_num), leg_angle[:, 0], period, step)
    leg_angle2 = Interpolation(np.linspace(0, period, 2 * sample_num), leg_angle[:, 1], period, step)
    leg_angle = np.stack((leg_angle1, leg_angle2), axis=-1)
    leg_right_hip = leg_angle[:, 0]
    leg_right_knee = -leg_angle[:, 1]
    leg_right_ankle = (leg_angle[:, 0] + leg_angle[:, 1])
    angle_control = np.stack((leg_right_hip, leg_right_knee, leg_right_ankle), axis=-1)
    speed_control = []
    for i in range(angle_control.shape[0] - 1):
        speed_control.append((angle_control[i + 1, :] - angle_control[i, :]) / step)
    # print(leg_angle)
    return angle_control, speed_control


def main():
    # prepare the parameter needed for trajectory_produce()
    h0 = 0.6  # init robot height
    period = 3.0  # total motion period
    step = 0.005  # time step
    angle_path, speed_control = trajectory_produce(h0, period, step)  # get the motion trajectory
    angle_path = np.rad2deg(angle_path)
    # print(angle_path)

    # =====================================================
    # init the robot
    RgmNet = RgmNetwork()  # create an can net for motor control
    id = [4, 5, 6]  # the motor's id in the CAN network
    RgmNet.add_node(id, 'config.ini')  # add the id into the network

    # change the CAN network state
    # initialize the control word and control state
    RgmNet.control_state(id, 'RESET FAULT')  # reset
    RgmNet.control_state(id, 'SHUT DOWN')  # close every thing
    RgmNet.control_state(id, 'SWITCH 0N')  # add the power but not enable

    # according to the hardware to change this value
    angle_init = [188.99, 169.39, 196.41]  # get current angle
    N = angle_path.shape[0]
    angle_control = []
    for i in range(N):
        angle_control.append(
            [angle_init[0] + angle_path[i][0],
             angle_init[1] + angle_path[i][1],
             angle_init[2] + angle_path[i][2]])
        print(angle_control[i])
    # print(angle_control)

    print("press ENTER to start")
    motion_flag = 0  # a flag to note whether motion is allowed
    direction_flag = 1  # determine the motion direction/ up or down
    index = 0  # note current point index

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
            if index == N - 1:
                # motion_flag = False
                # RgmNet.control_state(id, 'QUICK STOP')
                # RgmNet.control_state(id, 'DISABLE VOLTAGE')
                # break
                direction_flag = 0

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
            sleep(0.015)
            # sleep(1)


if __name__ == '__main__':
    main()
