from PCANBasic import *
from time import sleep

pcan = PCANBasic()
pcan.Initialize(PCAN_USBBUS1, PCAN_BAUD_1M)

while True:

    try:
        flag, msg, _ = pcan.Read(PCAN_USBBUS1)
        # print(msg.DATA[0], msg.DATA[1], msg.DATA[2], msg.DATA[3], msg.DATA[4], msg.DATA[5], msg.DATA[6], msg.DATA[7])
        if not flag:
            print(msg.DATA[0], msg.DATA[1], msg.DATA[2], msg.DATA[3], msg.DATA[4], msg.DATA[5], msg.DATA[6], msg.DATA[7])

        pass

    except:
        pass

    sleep(0.001)
