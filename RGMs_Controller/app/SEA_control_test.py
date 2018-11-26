from RGM.rgm.RGM_network import RgmNetwork
from time import sleep
import msvcrt
import logging

stiffness = 1 / 90 * 4

RgmNet = RgmNetwork()
id = 1
RgmNet.add_node(id, 'config.ini')
sleep(0.1)
RgmNet.control_state(id, 'RESET FAULT')
RgmNet.control_state(id, 'SHUT DOWN')
RgmNet.control_state(id, 'SWITCH 0N')
sleep(0.1)
RgmNet.control_state(id, 'ENABLE OPERATION')
RgmNet.operation_mode('CYCLIC SYNCHRONOUS TORQUE')
sleep(0.1)
RgmNet.syn_tor_init(id)

while True:

    if msvcrt.kbhit():
        temp = ord(msvcrt.getch())
        if temp == 27:
            logging.warning("quit program [app_user_interaction.py]")
            RgmNet.syn_tor_ctrl(id, 0)
            RgmNet.control_state(id, 'SWITCH 0N')
            RgmNet.control_state(id, 'SHUT DOWN')
            break

    # get current motion state
    para = RgmNet.get_motion_para(id)
    para = para[str(id)]
    position, velocity, current = para[0], para[1], para[2]
    print('position: %f, velocity: %f, current: %f' % (position, velocity, current))
    current_target = position * -1 * stiffness
    # if current_target < 0:
    #     current_target -= 0.6
    # elif current_target > 0:
    #     current_target += 0.6
    # else:
    #     pass
    print('target current: %f' % current_target)
    RgmNet.syn_tor_ctrl(id, current_target)
