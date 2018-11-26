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

# angle = 5

# init the CAN network
RgmNet = RgmNetwork()
id = 2
RgmNet.add_node(id, 'config.ini')
RgmNet.control_state(id, 'RESET FAULT')
RgmNet.control_state(id, 'SHUT DOWN')
RgmNet.control_state(id, 'SWITCH 0N')
RgmNet.control_state(id, 'ENABLE OPERATION')
RgmNet.operation_mode('CYCLIC SYNCHRONOUS POSITION')
para = RgmNet.get_motion_para(id)
para = para[str(id)]
position, velocity, current = para[0], para[1], para[2]
angle = position
print(angle)

sleep(2)
#
RgmNet.syn_pos_init(id)
RgmNet.syn_pos_ctrl(id, angle)
sleep(1)
# RgmNet.control_state(id, 'SWITCH 0N')
# RgmNet.control_state(id, 'SHUT DOWN')
while True:
    # print(angle)
    # para = RgmNet.get_motion_para(id)
    # para = para[str(id)]
    # position, velocity, current = para[0], para[1], para[2]
    # tenp_angle = position
    # print(tenp_angle)
    if msvcrt.kbhit():
        temp = ord(msvcrt.getch())
        # print(temp)
        if temp == 27:
            logging.warning("quit program [app_user_interaction.py]")
            RgmNet.control_state(id, 'QUICK STOP')
            RgmNet.control_state(id, 'DISABLE VOLTAGE')
            break
        elif temp == 32:
            logging.warning("stop the interaction, press [ENTER] to continue")
            RgmNet.control_state(id, 'QUICK STOP')
            angle_temp = angle
            """stop the operation"""
            pass
        elif temp == 13:
            logging.warning("Continue the interaction, press [SPACE] to stop")
            RgmNet.control_state(id, 'ENABLE OPERATION')
            angle = angle_temp
            sleep(0.1)
            """continue"""
            pass
        elif temp == 97:
            """anticlockwise rotation"""
            angle += 0.2
            RgmNet.syn_pos_ctrl(id, angle)
            pass
        elif temp == 100:
            """clockwise rotation"""
            angle -= 0.2
            RgmNet.syn_pos_ctrl(id, angle)
            pass
        else:
            logging.warning("Illegal keyboard pressed, please check!")
