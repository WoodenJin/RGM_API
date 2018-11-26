"""
robot control class
"""
import logging
from sys import exit
from Robot_Simulation import BipedRobot
from Robot_Real import BipedRobotreal
from Robot_Time import Robot_Time
from Trajectory import Trajectory


class RobotControl(object):
    def __init__(self, env, h0, a, b, period=1, amp=0, phase=0, flag_up=False, sample_num=200, flag_real_time=False):
        # init the env
        if env == 'simulation':
            # ENV = 1
            self.robot = BipedRobot(step=period / sample_num)
        elif env == 'real':
            # ENV = 2
            self.robot = BipedRobotreal(flag_up=flag_up)
        else:
            logging.warning("illegal env name, please check, only simulation or real is allowed")
            exit()  # if the environment name is wrong, quit the program and show an logging warning
        # init the trajectory
        self.trajectory = Trajectory(h0, a, b, period, amp=amp, phase=phase, flag_up=flag_up)
        self.h0 = h0
        # init the robot timer
        self.timer = Robot_Time(period=period, sample_num=sample_num, flag_real_time=flag_real_time)
        self.current_time = 0

    def initialize(self):
        # init the robot geometry
        init_pos = [0, 0, self.h0 + 0.033]
        init_angle = self.trajectory.get_control_angle(0)
        self.robot.initialize(init_pos, init_angle)
        # robot in the real world and in the simulation have different during initialize

        # init the robot timer
        self.timer.start()
        print("initialize success")
        pass

    def run(self):
        current_time = self.timer.current()
        self.current_time = current_time
        angle_control = self.trajectory.get_control_angle(current_time)
        self.robot.run(angle_control)

        # print("running")
        pass

    def stop(self):
        self.robot.stop()
        self.timer.stop()
        print("stoped")
        pass

    def close(self):
        print("closed")
        exit()
        pass
