from RGMs_Controller.RGM.rgm.RGM_network import RgmNetwork
from time import sleep
import logging
import keyboard as key
import threading

"""
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
    :param id: motor's id
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


def robot_control(Nodeid):
    """
    this script is used to control the robot
    :param id: the motor id in the can network
    :return: return nothing, control the robot
    """

    global pressed_key_flag

    RgmNet = RgmNetwork()  # create an can network
    RgmNet.add_node(Nodeid, 'config.ini')
    RgmNet.control_state(Nodeid, 'RESET FAULT')
    RgmNet.control_state(Nodeid, 'SHUT DOWN')
    RgmNet.control_state(Nodeid, 'SWITCH 0N')

    angle_init = get_angle(RgmNet, Nodeid)
    angle_memory = angle_init
    motion_flag = False
    id_flag = False
    control_id = 0  # this need to be change, according to the keyboard input
    control_angle = 180  # this need to be change, according to the angle memory

    # motion_flag = False
    # id_flag = False

    while True:

        # print(pressed_key_flag)
        # sleep(0.1)

        if pressed_key_flag == 1:
            print("quit program [app_multi_interaction.py]")
            RgmNet.control_state(Nodeid, 'QUICK STOP')
            RgmNet.control_state(Nodeid, 'DISABLE VOLTAGE')
            break

        elif pressed_key_flag == 57:
            print("stop the interaction, press [ENTER] to continue")
            RgmNet.control_state(Nodeid, 'QUICK STOP')
            motion_flag = False
            """stop the operation"""
            pass

        elif pressed_key_flag == 28:
            print("Continue the interaction, press [SPACE] to stop")
            RgmNet.control_state(Nodeid, 'ENABLE OPERATION')
            print("1")
            RgmNet.operation_mode('CYCLIC SYNCHRONOUS POSITION')
            print("2")
            RgmNet.syn_pos_init(Nodeid)
            motion_flag = True
            sleep(0.1)
            """continue"""
            pass

        elif 1 < pressed_key_flag < 11:
            control_id = pressed_key_flag - 1
            try:
                Nodeid.index(control_id)
                control_angle = angle_memory[str(control_id)]
                id_flag = True
                print("motor %d is selected" % control_id)
                pass
            except:
                logging.warning("the input id number is illegal")
                id_flag = False
                pass
            finally:
                pass
            pass

        elif pressed_key_flag == 30 or pressed_key_flag == 32 and motion_flag and id_flag:
            if pressed_key_flag == 30:
                control_angle += 0.2
                angle_memory.update({str(control_id): control_angle})
                RgmNet.syn_pos_ctrl(control_id, control_angle)
                pass
            elif pressed_key_flag == 32:
                control_angle -= 0.2
                angle_memory.update({str(control_id): control_angle})
                RgmNet.syn_pos_ctrl(control_id, control_angle)
                pass
            else:
                pass
            for i in range(len(Nodeid)):
                print("id is ", Nodeid[i], " angle is ", angle_memory[str(Nodeid[i])])
                pass

        else:
            # logging.warning("illegal keyboard pressed, please check!")
            # print(pressed_key_flag)
            pass

        pressed_key_flag = 0  # clear the buffer

    # key.unhook_all_hotkeys()

    return None


def pressed_keys(e):
    global pressed_key_flag
    # line = ''.join(str(code) for code in key._pressed_events)
    for code in key._pressed_events:
        pressed_key_flag = code
    pass


# global pressed_key_flag  # global


def main():
    """
    main function
    :return:
    """
    global pressed_key_flag
    pressed_key_flag = 0

    Nodeid = [2]
    control_thread = threading.Thread(target=robot_control, args=(Nodeid,))
    control_thread.setDaemon(True)
    control_thread.start()
    sleep(1)

    key.hook(pressed_keys)
    key.wait()


if __name__ == "__main__":
    main()
