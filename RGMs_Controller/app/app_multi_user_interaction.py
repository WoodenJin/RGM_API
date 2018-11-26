# import msvcrt
from RGMs_Controller.RGM.rgm.RGM_network import RgmNetwork
from time import sleep
import logging

"""
keyboard detect
[esc] quit  --27
[Space] means stop --32
[Enter] means continue --13
[a] anticlockwise rotation --97
[d] clockwise rotation --100
[1] control id change to 1 --49
[2] control id change to 2 --50
[*] control id change to * --48+*  (* <= 9)
"""


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
    # init the CAN net
    RgmNet = RgmNetwork()
    # id = [1, 2, 3, 4, 6]
    id = [1, 2, 3, 4, 5, 6]
    # id = [2]
    RgmNet.add_node(id, 'config.ini')
    RgmNet.control_state(id, 'RESET FAULT')
    RgmNet.control_state(id, 'SHUT DOWN')
    RgmNet.control_state(id, 'SWITCH 0N')
    # RgmNet.control_state(id, 'ENABLE OPERATION')
    # RgmNet.operation_mode('CYCLIC SYNCHRONOUS POSITION')
    angle_init = get_angle(RgmNet, id)
    angle_memory = angle_init
    motion_flag = False
    id_flag = False
    control_id = 0  # this need to be change
    control_angle = 180  # this need to be change
    while True:

        if msvcrt.kbhit():
            temp = ord(msvcrt.getch())
            # print(temp)
            if temp == 27:
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

            elif 48 < temp < 58:
                control_id = temp - 48
                try:
                    id.index(control_id)
                    control_angle = angle_memory[str(control_id)]
                    id_flag = True
                    print("motor %d is selected" % control_id)
                except:
                    logging.warning("the input id number is illegal")
                    id_flag = False
                finally:
                    pass
                pass

            elif temp == 97 or temp == 100 and motion_flag and id_flag:
                # print(control_id)
                if temp == 97:
                    control_angle += 0.2
                    # control_angle += 10
                    angle_memory.update({str(control_id): control_angle})
                    RgmNet.syn_pos_ctrl(control_id, control_angle)
                elif temp == 100:
                    control_angle -= 0.2
                    # control_angle -= 10
                    angle_memory.update({str(control_id): control_angle})
                    RgmNet.syn_pos_ctrl(control_id, control_angle)
                else:
                    pass
                # angle_memory.update({str(control_id): control_angle})
                # print(angle_memory[str(1)], angle_memory[str(2)], angle_memory[str(3)],
                #       angle_memory[str(4)], angle_memory[str(6)])
                # print(angle_memory[str(1)], angle_memory[str(2)],
                #       angle_memory[str(3)], angle_memory[str(4)],
                #       angle_memory[str(5)], angle_memory[str(6)])
                # print(angle_memory[str(6)])
                # print(angle_memory[str(4)], angle_memory[str(5)],angle_memory[str(6)])
                for i in range(len(id)):
                    print("id is ", id[i], " angle is ", angle_memory[str(id[i])])

            elif temp == 0:
                pass
            else:
                logging.warning("illegal keyboard pressed, please chack!")
        pass


if __name__ == '__main__':
    main()
