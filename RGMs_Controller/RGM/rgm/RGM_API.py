# RGM_API.py
# Author Wooden jin
# V0.5 9 Apr, 2018

"""
RGM_API is a micropython api for the rgm motor control

ZJU license
Copyright @ 2018 Wooden Jin, yongbinjin@zju.edu.cn
"""

from PCANBasic import *  # peak can api
from time import sleep   # tiem function
from math import *       # math function


class RGM(object):

    def __init__(self, ID=[1538, 1537]):
        self.__pcan = PCANBasic()
        self.__pcan.Initialize(PCAN_USBBUS1, PCAN_BAUD_1M)
        self.__id = ID

    def servo_Init(self, id):

        errorflag = 1
        for i in range(0, len(self.__id)):
            if self.__id[i] == id:
                errorflag = 0

        if(errorflag):
            print("==============================")
            print("Motor ID isn't matched")
            print("please check the motor's id")
        else:
            mass1 = TPCANMsg()
            mass1.ID = id
            mass1.LEN = 8
            mass1.DATA[0] = TPCANMessageType(0x2F)
            mass1.DATA[1] = TPCANMessageType(0x40)
            mass1.DATA[2] = TPCANMessageType(0x60)
            mass1.DATA[3] = TPCANMessageType(0x00)
            mass1.DATA[4] = TPCANMessageType(0x06)
            mass1.DATA[5] = TPCANMessageType(0x00)
            mass1.DATA[6] = TPCANMessageType(0x00)
            mass1.DATA[7] = TPCANMessageType(0x00)
            temp = self.__pcan.Write(PCAN_USBBUS1, mass1)
            sleep(1)
            mass1.DATA[4] = TPCANMessageType(0x07)
            temp = self.__pcan.Write(PCAN_USBBUS1, mass1)
            sleep(1)
            mass1.DATA[4] = TPCANMessageType(0x0f)
            temp = self.__pcan.Write(PCAN_USBBUS1, mass1)
            sleep(1)
            mass1.DATA[1] = TPCANMessageType(0x60)
            mass1.DATA[4] = TPCANMessageType(0x01)
            temp = self.__pcan.Write(PCAN_USBBUS1, mass1)
            sleep(1)
            print("=======================================")
            print("Motor ", id, " initialze success")

    def servo_Uninit(self, id):
        errorflag = 1

        for i in range(0, len(self.__id)):
            if self.__id[i] == id:
                errorflag = 0

        if(errorflag):
            print("==============================")
            print("Motor ID isn't matched")
            print("please check the motor's id")
        else:
            mass1 = TPCANMsg()
            mass1.ID = id
            mass1.LEN = 8
            mass1.DATA[0] = TPCANMessageType(0x2F)
            mass1.DATA[1] = TPCANMessageType(0x40)
            mass1.DATA[2] = TPCANMessageType(0x60)
            mass1.DATA[3] = TPCANMessageType(0x00)
            mass1.DATA[4] = TPCANMessageType(0x06)
            mass1.DATA[5] = TPCANMessageType(0x00)
            mass1.DATA[6] = TPCANMessageType(0x00)
            mass1.DATA[7] = TPCANMessageType(0x00)
            temp = self.__pcan.Write(PCAN_USBBUS1, mass1)
            # temp = self.__pcan.Read(PCAN_USBBUS1)
            # print(temp[1].DATA[2])
            sleep(1)

    def servo_position_control(self, id, angle, dt):
        errorflag = 1

        for i in range(0, len(self.__id)):
            if self.__id[i] == id:
                errorflag = 0
        if(errorflag):
            print("==============================")
            print("Motor ID isn't matched")
            print("please check the motor's id")
        else:
            # start position control
            mass = TPCANMsg()
            mass.ID = id
            mass.LEN = 8
            mass.DATA[0] = TPCANMessageType(0x23)
            mass.DATA[1] = TPCANMessageType(0x7a)
            mass.DATA[2] = TPCANMessageType(0x60)
            mass.DATA[3] = TPCANMessageType(0x00)

            if type(angle) == int:
                angle_temp = RGM.angle2mass(angle)
                mass.DATA[4] = angle_temp[3]
                mass.DATA[5] = angle_temp[2]
                mass.DATA[6] = angle_temp[1]
                mass.DATA[7] = angle_temp[0]
                temp = self.__pcan.Write(PCAN_USBBUS1, mass)
                sleep(dt)
                print("==============================")
                print("angle write success")

            elif type(angle) == list:
                for i in range(len(angle)):
                    angle_temp = RGM.angle2mass(angle[i])
                    mass.DATA[4] = angle_temp[3]
                    mass.DATA[5] = angle_temp[2]
                    mass.DATA[6] = angle_temp[1]
                    mass.DATA[7] = angle_temp[0]
                    temp = self.__pcan.Write(PCAN_USBBUS1, mass)
                    sleep(dt)
                print("==================================")
                print("angle list write success")

            else:
                print("ERROR")
                print("the data type of angle is not int or list")
                print("please check")

    def angle2mass(angle):
        temp = []
        angle = angle * 524288 / 360
        temp.append(TPCANMessageType(0))
        temp.append(TPCANMessageType(floor(angle / 65536)))
        temp.append(TPCANMessageType(floor((angle % 65536) / 256)))
        temp.append(TPCANMessageType(floor((angle % 65536)) % 256))
        # temp.append(TPCANMessageType(0))
        return temp


def main():
    rgm = RGM()
    id = 1538
    # try:
    #     rgm.servo_Uninit(id)
    # except:
    #     pass
    rgm.servo_Init(id)
    # rgm.servo_position_control(id, [40], 1)

    sleep(5)
    rgm.servo_Uninit(id)


if __name__ == "__main__":
    main()
