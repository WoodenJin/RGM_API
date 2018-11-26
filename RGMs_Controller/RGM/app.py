from RGM.rgm.RGM_network import RgmNetwork
from time import sleep

RgmNet = RgmNetwork()

# RgmNet.operation_mode('PROFILE POSITION')

"""
# test for change the control word

RgmNet.add_node(1, 'config.ini')

RgmNet.control_state(1, 'SHUT DOWN')
RgmNet.get_state(1)
# RgmNet.get_state(2)
sleep(1)

RgmNet.control_state(1, 'SWITCH 0N')
RgmNet.get_state(1)
sleep(1)

RgmNet.control_state(1, 'ENABLE OPERATION')
RgmNet.get_state(1)
RgmNet.operation_mode('PROFILE POSITION')
sleep(3)


RgmNet.control_state(1, 'QUICK STOP')
RgmNet.get_state(1)
sleep(1)

RgmNet.control_state(1, 'ENABLE OPERATION')
RgmNet.get_state(1)
sleep(1)

RgmNet.control_state(1, 'DISABLE VOLTAGE')
RgmNet.get_state(1)
"""

"""syn position control"""
# RgmNet.add_node(1, 'config.ini')
# sleep(0.1)
# RgmNet.control_state(1, 'RESET FAULT')
#
# RgmNet.control_state(1, 'SHUT DOWN')
#
# RgmNet.control_state(1, 'SWITCH 0N')
# sleep(0.1)
# RgmNet.control_state(1, 'ENABLE OPERATION')
# # RgmNet.operation_mode('ENABLE OPERATION')
# # RgmNet.operation_mode('PROFILE POSITION')
# RgmNet.operation_mode('CYCLIC SYNCHRONOUS POSITION')
#
# sleep(0.1)
# # RgmNet.pro_pos_init(1)
# RgmNet.syn_pos_init(1)
# sleep(0.1)
# # RgmNet.pro_pos_ctrl(1, 60)
# for i in range(180):
#     RgmNet.syn_pos_ctrl(1, i)
#     sleep(0.013)
# for i in range(180):
#     RgmNet.syn_pos_ctrl(1, (179 - i))
#     sleep(0.013)
# for i in range(180):
#     RgmNet.syn_pos_ctrl(1, -1*i)
#     sleep(0.013)
# for i in range(180):
#     RgmNet.syn_pos_ctrl(1, -1*(179 - i))
#     sleep(0.013)
# RgmNet.control_state(1, 'QUICK STOP')
# RgmNet.control_state(1, 'ENABLE OPERATION')
# sleep(0.1)
# RgmNet.control_state(1, 'SHUT DOWN')

""""syn velocity control"""
# RgmNet.add_node(1, 'config.ini')
# sleep(0.1)
# RgmNet.control_state(1, 'RESET FAULT')
# RgmNet.control_state(1, 'SHUT DOWN')
# RgmNet.control_state(1, 'SWITCH 0N')
# sleep(0.1)
# RgmNet.control_state(1, 'ENABLE OPERATION')
# RgmNet.operation_mode('CYCLIC SYNCHRONOUS VELOCITY')
# sleep(0.1)
# RgmNet.syn_vel_init(1)
# RgmNet.syn_vel_ctrl(1, -20)
# para = RgmNet.get_motion_para(1)
# temp = para[str(1)]
# print('position: ', temp[0])
# print('velocity: ', temp[1])
# print('current: ', temp[2])
# sleep(1)
# para = RgmNet.get_motion_para(1)
# temp = para[str(1)]
# print('position: ', temp[0])
# print('velocity: ', temp[1])
# print('current: ', temp[2])
# sleep(1)
# para = RgmNet.get_motion_para(1)
# temp = para[str(1)]
# print('position: ', temp[0])
# print('velocity: ', temp[1])
# print('current: ', temp[2])
# sleep(1)
# RgmNet.syn_vel_ctrl(1, 0)
# para = RgmNet.get_motion_para(1)
# temp = para[str(1)]
# print('position: ', temp[0])
# print('velocity: ', temp[1])
# print('current: ', temp[2])
# sleep(1)
# RgmNet.syn_vel_ctrl(1, 20)
# sleep(1)
# para = RgmNet.get_motion_para(1)
# temp = para[str(1)]
# print('position: ', temp[0])
# print('velocity: ', temp[1])
# print('current: ', temp[2])
# sleep(3)
# RgmNet.syn_vel_ctrl(1, 0)
# para = RgmNet.get_motion_para(1)
# temp = para[str(1)]
# print('position: ', temp[0])
# print('velocity: ', temp[1])
# print('current: ', temp[2])
# RgmNet.control_state(1, 'SWITCH 0N')
# RgmNet.control_state(1, 'SHUT DOWN')

""" syn torque control"""
# id = 2
# RgmNet.add_node(id, 'config.ini')
# sleep(0.1)
# RgmNet.control_state(id, 'RESET FAULT')
# RgmNet.control_state(id, 'SHUT DOWN')
# RgmNet.control_state(id, 'SWITCH 0N')
# sleep(0.1)
# RgmNet.control_state(id, 'ENABLE OPERATION')
# RgmNet.operation_mode('CYCLIC SYNCHRONOUS TORQUE')
# sleep(0.1)
# RgmNet.syn_tor_init(id)
# RgmNet.syn_tor_ctrl(id, 1)
# sleep(8)
# RgmNet.syn_tor_ctrl(id, 0)
# sleep(2)
# RgmNet.control_state(id, 'QUICK STOP')
# RgmNet.control_state(id, 'ENABLE OPERATION')
# RgmNet.control_state(id, 'SHUT DOWN')

"""profile position control"""
id = 2
RgmNet.add_node(id, 'config.ini')
sleep(0.1)
RgmNet.control_state(id, 'RESET FAULT')
RgmNet.control_state(id, 'SHUT DOWN')
RgmNet.control_state(id, 'SWITCH 0N')

RgmNet.control_state(id, 'ENABLE OPERATION')
RgmNet.operation_mode('PROFILE POSITION')
sleep(1)
RgmNet.pro_pos_init(id)
RgmNet.pro_pos_ctrl(id, 180)
sleep(10)
RgmNet.control_state(id, 'QUICK STOP')
RgmNet.control_state(id, 'ENABLE OPERATION')
RgmNet.control_state(id, 'SHUT DOWN')

"""state parameter(position, velocity, current) get from motor test"""
# RgmNet = RgmNetwork()
# RgmNet.add_node(1, 'config.ini')
# para = RgmNet.get_motion_para(1)
# temp = para[str(1)]
# print('position: ', temp[0])
# print('velocity: ', temp[1])
# print('current: ', temp[2])
