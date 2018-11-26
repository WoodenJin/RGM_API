"""
this script is used for
"""
from RGMs_Controller.RGM.rgm.RGM_network import RgmNetwork
from time import sleep

Nodeid = [2]  # a list used to save the motor id

RgmNet = RgmNetwork()  # create an can network
RgmNet.add_node(Nodeid, 'config.ini')
RgmNet.control_state(Nodeid, 'SWITCH 0N')
RgmNet.control_state(Nodeid, 'ENABLE OPERATION')

sleep(2)
RgmNet.control_state(Nodeid, 'SHUT DOWN')

