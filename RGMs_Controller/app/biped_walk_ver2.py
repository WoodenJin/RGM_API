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
# import matplotlib
# matplotlib.use("Qt4Agg")
import matplotlib.pyplot as plt

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


# def Interpolation(x0, y0, period, step):
#     """
#     this function is used to interpolate the control angle
#     :param x0: sample point, x coordinate
#     :param y0: sample point, y coordinate
#     :param period:  total time of the action
#     :param step:  time step
#     :return:  return the detailed control angle
#     """
#     t = interpolate.splrep(x0, y0)
#     N = int(period / step)
#     x = np.linspace(0, period, N)
#     y = interpolate.splev(x, t)
#     return y

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


def trajectory_produce(h0, a, b, period, step, t, h):
    # define the mathematical model
    # -----------------------------------------------------------------------------------
    thigh_length = 0.353  # length of the thigh, the static parameter
    shank_length = 0.350  # length of the shank, the static parameter

    leg1 = ik.Actuator(['y', [0, 0, -thigh_length], 'y', [0, 0, -shank_length]])
    leg1.angles = [-0.01, 0.01]  # init the configuration of the leg1

    leg2 = ik.Actuator(['y', [0, 0, -thigh_length], 'y', [0, 0, -shank_length]])
    leg2.angles = [-0.01, 0.01]  # init the configuration of the leg2
    # -----------------------------------------------------------------------------------

    # -----------------------------------------------------------------------------------
    # define the trajectory of the end-effector point
    dt = 0.05  # this parameter is used to control the timestep during calculate the inverse kinematics
    num_lame = int(t[0] * period * 0.5 / dt)  # the number of point during lame the leg
    num_lift = int((t[1] * period * 0.5 - t[0] * period * 0.5) / dt)  # the number of the point during left the leg
    num_land = int(1 * period * 0.5 / dt) - num_lame - num_lift  # the number of the point during land the leg
    sample_num = num_land + num_lift + num_lame  # total sample number

    # first half cycle
    leg1_aim_x = np.hstack((
        np.linspace(0, 0, num_lame),
        np.linspace(0, 0, num_lift),
        np.linspace(0, 0, num_land)
    ))  # since now, the left leg doesn't move forward at all

    leg1_aim_y = leg1_aim_x.copy()

    leg1_aim_z = np.hstack((
        np.linspace(-h0, -h0 - h[0], num_lame),
        np.linspace(-h0 - h[0], -h0 + h[1], num_lift),
        np.linspace(-h0 + h[1], -h0, num_land)
    ))  # foot trajectory in z direction

    leg1_aim = np.stack((leg1_aim_x, leg1_aim_y, leg1_aim_z), axis=-1)
    leg1_angle = np.zeros((sample_num, 2))

    leg2_aim_x = leg1_aim_x.copy()
    leg2_aim_y = leg1_aim_y.copy()
    leg2_aim_z = np.ones(sample_num) * -h0
    leg2_aim = np.stack((leg2_aim_x, leg2_aim_y, leg2_aim_z), axis=-1)
    leg2_angle = np.zeros((sample_num, 2))

    for i in range(sample_num):
        leg1.ee = leg1_aim[i, :]
        leg1_angle[i, :] = leg1.angles
        leg2.ee = leg2_aim[i, :]
        leg2_angle[i, :] = leg2.angles

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
    angle_control_2 = np.hstack((angle_control[:, 3:6], angle_control[:, 0:3]))
    angle_control = np.vstack((angle_control, angle_control_2))
    angle_control[:, 1:4] = -angle_control[:, 1:4]
    temp = np.copy(angle_control[:, 0:3])
    angle_control[:, 0:3] = np.copy(angle_control[:, 3:6])
    angle_control[:, 3:6] = np.copy(temp)

    # ---------------------------------------------------------------
    # visualize the data curve
    N = angle_control.shape[0]
    # fig, ax = plt.subplots(figsize=(3, 3))
    # ax.plot(np.linspace(0, N, N), angle_control[:, 0])
    # ax.plot(np.linspace(0, N, N), angle_control[:, 1])
    # ax.plot(np.linspace(0, N, N), angle_control[:, 2])
    # ax.plot(np.linspace(0, N, N), angle_control[:, 3])
    # ax.plot(np.linspace(0, N, N), angle_control[:, 4])
    # ax.plot(np.linspace(0, N, N), angle_control[:, 5])
    # plt.show()

    print(angle_control)
    return angle_control


def main():
    # prepare the parameter needed for trajectory_produce()
    h0 = 0.68  # init robot height
    period = 2  # total motion period
    step = 0.02  # time step
    a = 0.02  # short axis of the ellipse trajectory
    b = 0.00  # long axis of the ellipse trajectory
    res_time = 0.5
    # angle_path = trajectory_produce(h0, a, b, period, step)  # get the motion trajectory
    angle_path = trajectory_produce(h0, a, b, period, step, [0.55, 0.75], [0.007, 0.005])
    angle_path = np.rad2deg(angle_path)
    # print(angle_path)
    print("the shape of angle_path is : ", angle_path.shape)

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
    angle_init = [153.33, 214.19, 182.26, 193.95, 146.61, 197.57] # get current angle
    # angle_init = [188.99, 169.39, 196.41, 157.31, 190.36, 176.41]  # get current angle
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
    # index = 0

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
            print(index)
            if direction_flag:
                index += 1
            else:
                index -= 1
            # sleep(step - 0.003)
            # sleep(0.5)


if __name__ == '__main__':
    main()
