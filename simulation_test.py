from RobotControl import RobotControl
from math import pi
from time import sleep

h0 = 0.66
a = 0.03
b = 0.04
Robot = RobotControl('simulation', h0, a, b, period=1.5, amp=(0*pi / 12), phase=(0*pi/2), flag_up=True, sample_num=150,
                     flag_real_time=True)
Robot.initialize()
# Robot.trajectory.display()
# sleep(1)
while True:
    Robot.run()
    print(Robot.current_time)
    sleep(0.015)
    # sleep(1.5 / 100)
