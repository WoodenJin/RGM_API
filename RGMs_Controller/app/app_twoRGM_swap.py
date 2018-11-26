import msvcrt
from RGM.rgm.RGM_network import RgmNetwork
from time import sleep
import logging

"""
keyboard detect
[esc] quit  --27
[Space] means stop --32
[Enter] means continue --13
[a] anticlockwise rotation --97
[d] clockwise rotation --100
"""

# init the CAN network
RgmNet = RgmNetwork()
id = [2, 3]
RgmNet.add_node(id, 'config.ini')
RgmNet.control_state(id, 'SHUT DOWN')
RgmNet.control_state(id, 'SWITCH 0N')
# RgmNet.control_state(id, 'ENABLE OPERATION')
# RgmNet.operation_mode('CYCLIC SYNCHRONOUS POSITION')
para = RgmNet.get_motion_para(id)
para1 = para[str(id[0])]
position, velocity, current = para1[0], para1[1], para1[2]
angle1 = position
print(angle1)
para2 = para[str(id[1])]
position, velocity, current = para2[0], para2[1], para2[2]
angle2 = position
print(angle2)
# sleep(3)
print("Start to move")
motion_flag = 0
direction_flag = 1
while True:

    if msvcrt.kbhit():
        temp = ord(msvcrt.getch())
        # print(temp)
        if temp == 27:
            print("quit program [app_user_interaction.py]")
            # RgmNet.control_state(id, 'SWITCH 0N')
            # RgmNet.control_state(id, 'SHUT DOWN')
            RgmNet.control_state(id, 'QUICK STOP')
            RgmNet.control_state(id, 'DISABLE VOLTAGE')
            break
        elif temp == 32:
            print("stop the interaction, press [ENTER] to continue")
            RgmNet.control_state(id, 'QUICK STOP')
            motion_flag = 0
            """stop the operation"""
            pass
        elif temp == 13:
            print("Continue the interaction, press [SPACE] to stop")
            RgmNet.control_state(id, 'ENABLE OPERATION')
            RgmNet.operation_mode('CYCLIC SYNCHRONOUS POSITION')
            RgmNet.syn_pos_init(id)
            motion_flag = 1
            sleep(0.1)
            """continue"""
            pass
        else:
            logging.warning("Illegal keyboard pressed, please check!")

    if motion_flag:

        if direction_flag:
            angle1 += 0.2
            angle2 += 0.2
        else:
            angle1 -= 0.2
            angle2 -= 0.2

        RgmNet.syn_pos_ctrl(id, [angle1, angle2])
        print("angle1: %f, angle2: %f" % (angle1, angle2))
        sleep(0.025)

        if angle1 > 90 or angle2 > 90:
            direction_flag = 0
        elif angle1 < 5 or angle2 < 5:
            direction_flag = 1
        else:
            pass
