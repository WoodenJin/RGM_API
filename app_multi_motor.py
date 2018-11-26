"""
Nov 8, works
this script using keyboard to control the motor
"""
from RGMs_Controller.RGM.rgm.RGM_network import RgmNetwork
from time import sleep
import logging
import keyboard as key

"""
keyboard manual
|================================================|
|    Keyboard 1 2 3 4 5 6 7 8 9 0                |
|    code 2 3 4 5 6 7 8 9 10 11                  |
|    note the id number of the motor             |
|------------------------------------------------|
|Keyboard   |  code  |    # note                 |
|------------------------------------------------|
|[Space]    |  57    |    # stop the program     |
|[Enter]    |  28    |    # start the program    |
|[Esc]      |  1     |    # exit the program     |
|[a]        |  30    |    # decrease angle       |
|[d]        |  32    |    # increase angle       |
|------------------------------------------------|
"""


def get_angle(RgmNet, Nodeid):
    """
    this parameter is used to get the angle of the motor
    :param RgmNet: the can network object
    :param Nodeid: motor's id
    :return:
    """
    # angle = []
    angle = {}
    para = RgmNet.get_motion_para(Nodeid)
    for i in range(len(Nodeid)):
        temp_para = para[str(Nodeid[i])]
        angle.update({str(Nodeid[i]): temp_para[0]})
        # angle.append(temp_para[0])
        print("the angle of id %d is %f" % (Nodeid[i], temp_para[0]))
    return angle


def enter_pressed():
    global RgmNet
    global Nodeid
    global motion_flag
    print("Continue the interaction, press [SPACE] to stop")
    RgmNet.control_state(Nodeid, 'ENABLE OPERATION')
    RgmNet.operation_mode('CYCLIC SYNCHRONOUS POSITION')
    RgmNet.syn_pos_init(Nodeid)
    motion_flag = True
    sleep(0.1)
    """continue"""


def space_pressed():
    global RgmNet
    global Nodeid
    global motion_flag
    RgmNet.control_state(Nodeid, 'QUICK STOP')
    motion_flag = False
    """stop the operation"""


def a_pressed():
    global RgmNet
    global Nodeid
    global control_angle
    global control_id
    global motion_flag
    global id_flag
    global angle_memory
    control_angle += 0.2  # add control angle
    angle_memory.update({str(control_id): control_angle})
    RgmNet.syn_pos_ctrl(control_id, control_angle)
    for i in range(len(Nodeid)):
        print("id is ", Nodeid[i], " angle is ", angle_memory[str(Nodeid[i])])


def d_pressed():
    global RgmNet
    global Nodeid
    global control_angle
    global control_id
    global motion_flag
    global id_flag
    global angle_memory
    control_angle -= 0.2  # add control angle
    angle_memory.update({str(control_id): control_angle})
    RgmNet.syn_pos_ctrl(control_id, control_angle)
    for i in range(len(Nodeid)):
        print("id is ", Nodeid[i], " angle is ", angle_memory[str(Nodeid[i])])


def number_pressed(index):
    global RgmNet
    global control_id
    global id_flag
    global Nodeid
    global control_angle
    global angle_memory
    try:
        Nodeid.index(index)
        id_flag = True
        control_id = index
        control_angle = angle_memory[str(control_id)]
        print("motor %d is selected" % control_id)
    except:
        logging.warning("the input id number is l=illegal")
        id_flag = False
    finally:
        pass


# -------------------------------------------
# create can network as a global value
RgmNet = RgmNetwork()  # create an can network

# init the can net, add the motor-id in the network
# Nodeid = [7]
Nodeid = [1, 2, 3, 4, 5, 6, 7]  # motor id for legged robot

RgmNet.add_node(Nodeid, 'config.ini')

# init the state of the motor
RgmNet.control_state(Nodeid, 'RESET FAULT')
RgmNet.control_state(Nodeid, 'SHUT DOWN')
RgmNet.control_state(Nodeid, 'SWITCH 0N')

# get the initial motor angle
angle_init = get_angle(RgmNet, Nodeid)
angle_memory = angle_init

# init the control parameter & flag
motion_flag = False
id_flag = False
control_id = 0  # default motor id, 0 is a dummy id, need to be change
control_angle = 180  # default required motor angle, need to be change


def main():
    global RgmNet
    global Nodeid
    # -------------------------
    # add hotkey according to the manual
    key.add_hotkey('enter', enter_pressed, args=[])
    key.add_hotkey('space', space_pressed, args=[])
    key.add_hotkey('a', a_pressed, args=[])
    key.add_hotkey('d', d_pressed, args=[])

    for i in range(10):
        key.add_hotkey(str(i), number_pressed, args=[i])

    # -------------------------------
    key.wait('esc')
    # stop the can network communication
    RgmNet.control_state(Nodeid, 'QUICK STOP')
    RgmNet.control_state(Nodeid, 'DISABLE VOLTAGE')
    # stop the program
    pass


main()
