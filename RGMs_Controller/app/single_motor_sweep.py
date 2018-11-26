import msvcrt
from RGM.rgm.RGM_network import RgmNetwork
from time import sleep, time
import logging
from math import pi, sin

RgmNet = RgmNetwork()

id = [2]
RgmNet.add_node(id, 'config.ini')
RgmNet.control_state(id, 'RESET FAULT')
RgmNet.control_state(id, 'SHUT DOWN')
RgmNet.control_state(id, 'SWITCH 0N')

para = RgmNet.get_motion_para(id)
para1 = para[str(id[0])]
position, velocity, current = para1[0], para1[1], para1[2]
angle1 = position

Amplitude = 5
Frequency = 3
Period = 1 / Frequency
time_step = 0.05

motion_flag = 0
direction_flag = 1
index = 0

while True:

    if msvcrt.kbhit():
        keyboard_flag = ord(msvcrt.getch())

        if keyboard_flag == 27:
            print("quit program [app_user_interaction.py]")
            RgmNet.control_state(id, 'QUICK STOP')
            RgmNet.control_state(id, 'DISABLE VOLTAGE')
            break
        elif keyboard_flag == 32:
            print("stop the interaction, press [ENTER] to continue")
            RgmNet.control_state(id, 'QUICK STOP')
            motion_flag = 0
            pass
        elif keyboard_flag == 13:
            print("Continue the interaction, press [SPACE] to stop")
            RgmNet.control_state(id, 'ENABLE OPERATION')
            RgmNet.operation_mode('CYCLIC SYNCHRONOUS POSITION')
            RgmNet.syn_pos_init(id)
            motion_flag = 1
            sleep(0.1)
            """continue"""
            start = time()
            pass
        elif keyboard_flag == 0:
            pass
        else:
            logging.warning("Illegal keyboard pressed, please check!")

    if motion_flag:
        # index += 1
        now = time()
        t = now - start
        angle = angle1 + Amplitude * sin(2 * pi * Frequency * t)
        RgmNet.syn_pos_ctrl(id, [angle])
        sleep(time_step)
