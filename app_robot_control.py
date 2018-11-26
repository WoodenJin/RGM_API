import threading
import keyboard as key
from RobotControl import RobotControl
from time import sleep
from math import pi


def flag_change(user_encoder):
    """
    callback function, according to the keyboard input to change the control flag
    :param user_encoder:
    :return:
    """
    global CONTROL_FLAG
    if user_encoder == 1:
        CONTROL_FLAG = True
        print(CONTROL_FLAG)
    elif user_encoder == 2:
        CONTROL_FLAG = False
        print(CONTROL_FLAG)
    else:
        pass


def robot_control():
    global CONTROL_FLAG
    global Robot
    init_flag = True
    while True:
        if CONTROL_FLAG:
            if init_flag:
                # Robot.trajectory_define()  # trajectory parameter define
                Robot.initialize()  # init the robot state
                init_flag = False
                pass
            else:
                Robot.run()
                # sleep(0.02)
                pass
        pass


global Robot
h0 = 0.66
a = 0.03
b = 0.00

# Robot = RobotControl('simulation', h0, a, b, period=5, amp=(0 * pi / 12), phase=(0 * pi / 2), flag_up=False,
#                      sample_num=150,
#                      flag_real_time=True)
Robot = RobotControl('real', h0, a, b, period=1.3, amp=(0 * pi / 12), phase=(0 * pi / 2), flag_up=True,
                     sample_num=150,
                     flag_real_time=True)
global CONTROL_FLAG  # this is an global control flag to indicate whether the robot is enabled to be control
CONTROL_FLAG = False


def main():
    """
    main function, a global keyboard listener
    :return: change the global flag
    """
    global CONTROL_FLAG  # control flag, true, robot run, false, robot stop
    global Robot  # robot structure

    t = threading.Thread(target=robot_control, args=())  # create an robot control threading
    t.setDaemon(True)
    t.start()

    key.add_hotkey('enter', flag_change, args=[1])
    key.add_hotkey('space', flag_change, args=[2])
    key.wait('esc')
    CONTROL_FLAG = False
    Robot.stop()
    Robot.close()


if __name__ == "__main__":
    main()
