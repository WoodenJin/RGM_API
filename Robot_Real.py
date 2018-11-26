"""
this script is used to control the real biped_robot
Author: Wooden Jin
e-mail: yongbinjin@zju.edu.cn
address: Zhejiang University
"""

from RGMs_Controller.RGM.rgm.RGM_network import RgmNetwork
import numpy as np
from time import sleep


class BipedRobotreal(object):
    JOINT_ID = [1, 2, 3, 4, 5, 6, 7]  # rgm motor's id
    JOINT_ANGLE_OFFSET = [156.306, 215.496, 153.713,
                          188.339, 142.858, 218.563,
                          182.651]  # motor's angle offset, unit is degree

    # [right_hip, right_knee, right_ankle, left_hip, left_knee, left_ankle, upper]

    def __init__(self, flag_up=True):
        self.RgmNet = RgmNetwork()  # create an can network

        # init the can net, add the motor-id in the network
        if flag_up:
            self.Nodeid = self.JOINT_ID
        else:
            self.Nodeid = self.JOINT_ID[0:6]

        self.RgmNet.add_node(self.Nodeid, 'config.ini')

        # init the state of the motor
        self.RgmNet.control_state(self.Nodeid, 'RESET FAULT')
        self.RgmNet.control_state(self.Nodeid, 'SHUT DOWN')
        self.RgmNet.control_state(self.Nodeid, 'SWITCH 0N')

    def initialize(self, init_pos, init_angle):
        """
        init the robot in the real world, init the control mode
        :param init_pos: this parameter is ignored in the method, just to keep the same style with robot_simulation.py
        :param init_angle: initial robot state, unit is degree
        :return: return nothing, robot response
        """
        # init the control mode
        self.RgmNet.control_state(self.Nodeid, 'ENABLE OPERATION')  # enable the motor
        self.RgmNet.operation_mode('CYCLIC SYNCHRONOUS POSITION')  # choose the position control mode
        self.RgmNet.syn_pos_init(self.Nodeid)  # init the synchronous position parameter

        angle_path_init = self.JOINT_ANGLE_OFFSET + init_angle
        angle = self.get_real_angle()
        angle_current = np.zeros(len(self.Nodeid))
        for i in range(angle_current.size):
            angle_current[i] = angle[str(self.Nodeid[i])]
        angle_step = (angle_current - angle_path_init) / 10

        for i in range(10):
            angle_control = angle_current - i * angle_step
            self.RgmNet.syn_pos_ctrl(self.Nodeid, angle_control)
            sleep(0.05)

    def run(self, angle_control):
        """
        robot will do one step
        :param angle_control: aim angle, unit is degree
        :return: nothing, robot will response
        """
        angle_control = angle_control + self.JOINT_ANGLE_OFFSET  # mapping the aim angle to the real robot configuration
        self.RgmNet.syn_pos_ctrl(self.Nodeid, angle_control)  # do one step robot control
        pass

    def stop(self):
        """
        stop the real robot control
        :return: nothing
        """
        print("quit program [real_time_biped_walk.py]")
        self.RgmNet.control_state(self.Nodeid, 'QUICK STOP')
        self.RgmNet.control_state(self.Nodeid, "DISABLE VOLTAGE")

    def get_real_angle(self):
        """
        this function is used to get the real robot's joint angle
        :return: angle list
        """
        angle = {}  # create an black angle dictionary
        para = self.RgmNet.get_motion_para(self.Nodeid)
        for i in range(len(self.Nodeid)):
            temp_para = para[str(self.Nodeid[i])]
            angle.update({str(self.Nodeid[i]): temp_para[0]})
            print("the angle of id %d is %f" % (self.Nodeid[i], temp_para[0]))
        return angle
