import tinyik as IK
import numpy as np
from numpy import deg2rad as dr
from numpy import rad2deg as rd
import msvcrt
from RGM.rgm.RGM_network import RgmNetwork
from time import sleep
import logging
from scipy import interpolate

"""
Here is an example of tinyik

arm = IK.Actuator(['z', [2., 0., 0.], 'z', [1., 0., 0.]])

arm.ee = [-1, 2, 0]
angle = np.round(np.rad2deg(arm.angles))
angle = np.array([angle[0] % 360, angle[1] % 360])
print(angle)
"""


def get_angle(RgmNet, id):
    """
    this function is used to get the angle of the motor
    :param RgmNet: the can network object
    :param id: id list, return the corresponding motor's angle
    :return: a dictionary, {'motor's id': motor's angle}
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


# def the leg structure
# Length = [432, 405.44]
Length = [350, 353]
arm = IK.Actuator(['z', [Length[0], 0, 0], 'z', [Length[1], 0, 0]])

# init the initial pos of leg
arm.angles = [dr(-1), dr(1)]
# print(arm.ee)

# def an path of the leg
angle_path = []  # this is a list which is used to record the angle path
# from [830,0,0] to [600,0,0]
n = 20
N = 100  # sample point
StartPoint = [700, 0, 0]
EndPoint = [600, 0, 0]
Path_x = np.linspace(StartPoint[0], EndPoint[0], n)
Path_y = np.linspace(StartPoint[1], EndPoint[1], n)
Path_z = np.linspace(StartPoint[2], EndPoint[2], n)
for i in range(int(n / 2)):
    arm.ee = [Path_x[i], Path_y[i], Path_z[i]]  # specific the end effector position
    angle_path.append(rd(arm.angles))  # record the angle needed to be control
    # print(angle_path[i])
angle_path = np.asarray(angle_path)
angle_path_temp = angle_path[::-1]
angle_path = np.vstack((angle_path, angle_path_temp))
# print(angle_path)
t0 = np.linspace(1, n, n)
t = interpolate.splrep(t0, angle_path[:, 0])
angle_path_interpolate_1 = interpolate.splev(np.linspace(1, n, N), t)
t = interpolate.splrep(t0, angle_path[:, 1])
angle_path_interpolate_2 = interpolate.splev(np.linspace(1, n, N), t)
angle_path = np.stack((angle_path_interpolate_1, angle_path_interpolate_2), axis=-1)
print(angle_path)

# Start to control the motor to move
# keyboard [enter] [space] [esc] means start to control, stop, quit

RgmNet = RgmNetwork()  # create an can net for motor control
id = [2, 3]  # the motor's id in the CAN network
RgmNet.add_node(id, 'config.ini')  # add the id into the network

# change the CAN network state
# initialize the control word and control state
RgmNet.control_state(id, 'RESET FAULT')  # reset
RgmNet.control_state(id, 'SHUT DOWN')  # close every thing
RgmNet.control_state(id, 'SWITCH 0N')  # add the power but not enable

angle_init = [190, 177]  # get current angle

motion_flag = 0  # a flag to note whether motion is allowed
direction_flag = 1  # determine the motion direction/ up or down
index = 0  # note current point index

# transform the angle_path into the real model
angle_control = []
for i in range(N):
    angle_control.append([angle_init[0] + angle_path[i][1], angle_init[1] + angle_path[i][0]])
    print(angle_control[i][0], angle_control[i][1])

print("press ENTER to start")

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
        angle1 = angle_control[index][0]
        angle2 = angle_control[index][1]
        RgmNet.syn_pos_ctrl(id, [angle1, angle2])
        if direction_flag:
            index += 1
        else:
            index -= 1
        sleep(0.03)
