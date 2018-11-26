from .PCANBasic import *  # peak can interface
from .PCANBasic import TPCANMessageType as MST
from .node import Node
import logging
from time import sleep
import numpy as np
from sys import exit

# status word 0x6041 bitmask and value in the list in the dictionary value
POWER_STATES_402 = {
    'NOT READY TO SWITCH ON': 0x00,
    'SWITCH ON DISABLED': 0x40,
    'READY TO SWITCH ON': 0x21,
    'SWITCHED ON': 0x23,
    'OPERATION ENABLED': 0x27,
    'FAULT': 0x08,
    'QUICK STOP ACTIVE': 0x07
}

POWER_STATES_MARKER = {
    'NOT READY TO SWITCH ON': np.array([0, 1, 0, 0, 1, 1, 1, 1], dtype=bool),
    'SWITCH ON DISABLED': np.array([0, 1, 0, 0, 1, 1, 1, 1], dtype=bool),
    'READY TO SWITCH ON': np.array([0, 1, 1, 0, 1, 1, 1, 1], dtype=bool),
    'SWITCHED ON': np.array([0, 1, 1, 0, 1, 1, 1, 1], dtype=bool),
    'OPERATION ENABLED': np.array([0, 1, 1, 0, 1, 1, 1, 1], dtype=bool),
    'FAULT': np.array([0, 1, 0, 0, 1, 1, 1, 1], dtype=bool),
    'QUICK STOP ACTIVE': np.array([0, 1, 1, 0, 1, 1, 1, 1], dtype=bool)
}

STATE_MACHINE = [
    'NOT READY TO SWITCH ON',
    'SWITCH ON DISABLED',
    'READY TO SWITCH ON',
    'SWITCHED ON',
    'OPERATION ENABLED',
    'FAULT',
    'QUICK STOP ACTIVE'
]

# control word 0x6040
POWER_STATE_COMMANDS = {
    'RESET FAULT': 0x80,
    'DISABLE VOLTAGE': 0x00,
    'SHUT DOWN': 0x06,
    'SWITCH 0N': 0x07,
    'ENABLE OPERATION': 0x0F,
    'QUICK STOP': 0x02,
    'BEGIN HOMING': 0x1F,
    'END HOMING': 0x0F
}

# Mode of Operation 0x6060, 0x6061
MODE_OF_OPERATION = {
    'PROFILE POSITION': 0x01,
    'PROFILE VELOCITY': 0x03,
    'PROFILE TORQUE': 0x04,
    'HOMING': 0x06,
    'INTERPOLATED POSITION': 0X07,
    'CYCLIC SYNCHRONOUS POSITION': 0x08,
    'CYCLIC SYNCHRONOUS VELOCITY': 0x09,
    'CYCLIC SYNCHRONOUS TORQUE': 0x0A,
}


class RgmNetwork(object):

    def __init__(self):
        # INIT the communication
        self.__pcan = PCANBasic()
        self.__channel = PCAN_USBBUS1
        self.__pcan.Initialize(PCAN_USBBUS1, PCAN_BAUD_1M)

        # communication buffer
        self.send_buffer = TPCANMsg()
        self.recv_buffer = TPCANMsg()

        self.ID = []
        self.node = []
        self.op_mode = None
        self.state = None
        sleep(0.5)
        self.recv()  # big bug, in ubuntu driver, after init the can network, can will receive [0,0,0,0,0,0,0,0]

    def add_node(self, motor_id, config):
        """
        :param motor_id: the hardware RGM motor ID
        :param config: config file path
        :return: nothing, add a node in the RgmNetwork
        """
        try:
            motor_id[0]
        except:
            motor_id = [motor_id]
        finally:
            pass

        for i in range(len(motor_id)):
            try:
                self.ID.index(motor_id[i])
                # print("motor with id %d is already added in the network" % motor_id)
                logging.warning("motor with id %d is already added in the network" % motor_id[i])
            except:
                # if we can't find the motor_id in list, add it !!!
                self.node.append(Node(motor_id[i], config))
                self.ID.append(motor_id[i])
            finally:
                pass

    def control_state(self, motor_id, control_word):
        """
        this method is used to change the node's state
        By 2018/7/6, homing model is disabled
        :param motor_id:
        :param control_word:  for example "DISABLE VOLTAGE"
        :return:
        """
        try:
            motor_id[0]
        except:
            motor_id = [motor_id]
        finally:
            pass

        try:
            POWER_STATE_COMMANDS[control_word]
            # print(POWER_STATE_COMMANDS[control_word])

        except:
            logging.warning("illegal input, control word %s is not allowed" % control_word)
            exit()
        finally:
            pass

        for i in range(len(motor_id)):
            node_id = motor_id[i]
            try:
                self.ID.index(node_id)
                # prepare massage
                msg_temp = TPCANMsg()
                msg_temp.ID = 0x600 + node_id
                msg_temp.LEN = 8
                msg_temp.DATA[0] = MST(0x2F)
                # 2Fh used by host when writing exactly 1 data byte. byte 5 contains data
                msg_temp.DATA[1] = MST(0x40)
                msg_temp.DATA[2] = MST(0x60)
                msg_temp.DATA[4] = MST(POWER_STATE_COMMANDS[control_word])
                # finish data prepare
                logging.info("In control_state, the message is well prepared")
                self.send_buffer = msg_temp

                self.send()
                self.recv()
                # sleep(0.1)
                # print(self.recv_buffer.DATA[1], self.recv_buffer.DATA[2], self.recv_buffer.DATA[3])
                self.get_state(node_id)


            except:
                # print("motor %d is not added in current network" % motor_id)
                logging.warning("motor %d is not added in current network" % node_id)
                pass
            finally:
                pass

    def get_state(self, motor_id):
        try:
            self.ID.index(motor_id)
        except:
            logging.warning("motor %d is not added in current network" % motor_id)
            return
        finally:
            pass
        # prepare parameter
        msg_temp = TPCANMsg()
        msg_temp.ID = 0x600 + motor_id
        msg_temp.LEN = 8
        msg_temp.DATA[0] = MST(0x40)
        msg_temp.DATA[1] = MST(0x41)
        msg_temp.DATA[2] = MST(0x60)
        self.send_buffer = msg_temp
        self.send()
        self.recv()
        state = self.recv_buffer.DATA[4]
        state_bool = np.unpackbits(np.array([state], dtype=np.uint8))
        # print(state_bool)
        flag = 1
        for i in range(7):
            marker = POWER_STATES_MARKER[STATE_MACHINE[i]]
            state_bool_temp = marker * state_bool
            state = np.packbits(state_bool_temp)
            temp = POWER_STATES_402[STATE_MACHINE[i]]
            # print("state = ", state, "temp = ", temp)
            # temp_bool = np.unpackbits(np.array([temp], dtype=np.uint8))
            # temp_bool = marker * temp_bool
            # temp = np.packbits(temp_bool)
            if state == temp:
                flag = 0
                print("node %d, the stateword is %s" % (motor_id, STATE_MACHINE[i]))
                self.state = STATE_MACHINE[i]
                break
        if flag:
            logging.warning("no matched state, please check")
        # sleep(0.05)

    def operation_mode(self, operation_word):
        """
        Only when the motor is enabled can we use this method to set the operation_mode
        this function is used to set the operation mode of motor
        all motor in the network is setted as the same mode
        :param operation_word:  example: "PROFILE POSITION" , "PROFILE VELOCITY", "PROFILE TORQUE"
        :return:  none
        """
        # logging.warning("motor's state must be ENABLED, so that can we change the operation mode")
        try:
            temp_value = MODE_OF_OPERATION[operation_word]
        except:
            logging.warning("%s is not an illegal operation mode, please check" % (operation_word))
            return
        finally:
            pass
        self.op_mode = temp_value
        temp_msg = TPCANMsg()
        temp_msg.LEN = 8
        temp_msg.DATA[0] = MST(0x2F)
        temp_msg.DATA[1] = MST(0x60)
        temp_msg.DATA[2] = MST(0x60)
        temp_msg.DATA[4] = MST(temp_value)
        for i in range(len(self.ID)):
            temp_msg.ID = 0x600 + self.ID[i]
            self.send_buffer = temp_msg
            self.send()
            self.recv()

    def pro_pos_init(self, node_id):
        """
        This method is used to init the position control pdo
        :param node_id: a node_id array
        :return: none
        """
        flag = self.is_node_added(node_id)
        if not flag:
            logging.error("there is some node not added in the RgmNet, please check")
            exit()

        # self.get_state()
        # here is some bug, the state only present one motor's power state
        if self.state != 'OPERATION ENABLED':
            # print(self.state)
            logging.error("the POWER STATE must be OPERATION ENABLED, but current state is %s" % self.state)
            exit()
        logging.info("node_id is already added, continue to init")
        # temp = TPCANMsg()
        # node_id = list(node_id)
        try:
            node_id[0]
        except:
            node_id = [node_id]
        finally:
            pass
        for i in range(len(node_id)):
            temp = TPCANMsg()
            temp.ID = 0x600 + node_id[i]
            temp.LEN = 8
            temp.DATA[0] = MST(0x22)
            temp.DATA[1] = MST(0x14)
            temp.DATA[2] = MST(0x14)
            temp.DATA[3] = MST(0x01)
            temp_id = self.node[i].pro_pos_rpdo.cob_id.to_bytes(2, byteorder='little')  # define the pdo cob-id
            temp.DATA[4] = MST(temp_id[0])
            temp.DATA[5] = MST(temp_id[1])
            self.send_buffer = temp
            self.send()
            temp.DATA[3] = MST(0x02)
            temp.DATA[4] = MST(0xFE)  # define the type of pdo, response immediately
            temp.DATA[5] = MST(0x00)
            self.send_buffer = temp
            self.send()

            temp = TPCANMsg()
            temp.ID = 0x000
            temp.LEN = 2
            temp.DATA[0] = MST(0x01)
            temp.DATA[1] = MST(node_id[i])
            self.send_buffer = temp
            self.send()
            sleep(0.1)
        logging.info("RPDO init success")

    def pro_pos_ctrl(self, node_id, pos):
        """
        This method is used to control the node_idth motor's position
        node_id can be an array, and pos must be the same size with node_id
        :param node_id: the motor's id, example: [1,2,3]
        :param pos: the pos of the designed angle. pos is in degree unit, example: [200, 120, 0]
        :return:
        """

        if MODE_OF_OPERATION['PROFILE POSITION'] != self.op_mode:
            logging.warning("the operation mode is not setted as PROFILE POSITION, please check")
            return
        # node_id = list(node_id)
        try:
            node_id[0]
        except:
            node_id = [node_id]
        finally:
            pass
        # pos = list(pos)
        try:
            pos[0]
        except:
            pos = [pos]
        finally:
            pass
        for i in range(len(node_id)):
            pos_data = (int(pos[i] / 360 * 0x00080000)).to_bytes(4, byteorder='little', signed=True)  # int32 to bytes
            # here, the pos need to be processed
            temp = TPCANMsg()
            temp.ID = self.node[i].pro_pos_rpdo.cob_id
            temp.LEN = 4
            temp.DATA[0] = MST(pos_data[0])
            temp.DATA[1] = MST(pos_data[1])
            temp.DATA[2] = MST(pos_data[2])
            temp.DATA[3] = MST(pos_data[3])
            self.send_buffer = temp
            self.send()
        pass

    def syn_pos_init(self, node_id):
        """
        This method is used to init the position control pdo
        :param node_id: a node_id array
        :return: none
        """
        flag = self.is_node_added(node_id)
        if not flag:
            logging.error("there is some node not added in the RgmNet, please check")
            exit()

        # self.get_state()
        # here is some bug, the state only present one motor's power state
        if self.state != 'OPERATION ENABLED':
            # print(self.state)
            logging.error("the POWER STATE must be OPERATION ENABLED, but current state is %s" % self.state)
            exit()

        if self.op_mode != MODE_OF_OPERATION['CYCLIC SYNCHRONOUS POSITION']:
            logging.warning(
                "the OPERATION MODE must be CYCLIC SYNCHRONOUS POSITION, but current state is %s" % self.op_mode)
            exit()

        logging.info("node_id is already added, continue to init")
        # temp = TPCANMsg()
        # node_id = list(node_id)
        try:
            node_id[0]
        except:
            node_id = [node_id]
        finally:
            pass
        for i in range(len(node_id)):
            temp = TPCANMsg()
            temp.ID = 0x600 + node_id[i]
            temp.LEN = 8
            temp.DATA[0] = MST(0x22)
            temp.DATA[1] = MST(0x14)
            temp.DATA[2] = MST(0x14)
            temp.DATA[3] = MST(0x01)
            temp_id = self.node[i].syn_pos_rpdo.cob_id.to_bytes(2, byteorder='little')
            temp.DATA[4] = MST(temp_id[0])
            temp.DATA[5] = MST(temp_id[1])
            self.send_buffer = temp
            self.send()
            self.recv()
            temp.DATA[3] = MST(0x02)
            temp.DATA[4] = MST(0xFE)  # def the RPDO type, 0xFE means response immediately
            temp.DATA[5] = MST(0x00)
            self.send_buffer = temp
            self.send()
            self.recv()

            temp_index = self.node[i].syn_pos_rpdo.time_Mantissa.index.to_bytes(2, byteorder='little')
            temp.DATA[1] = MST(temp_index[0])
            temp.DATA[2] = MST(temp_index[1])
            temp.DATA[3] = MST(self.node[i].syn_pos_rpdo.time_Mantissa.subindex)
            temp.DATA[4] = MST(self.node[i].syn_pos_rpdo.time_Mantissa.value)
            self.send_buffer = temp
            self.send()
            self.recv()
            temp_index = self.node[i].syn_pos_rpdo.time_Exponent.index.to_bytes(2, byteorder='little')
            temp.DATA[1] = MST(temp_index[0])
            temp.DATA[2] = MST(temp_index[1])
            temp.DATA[3] = MST(self.node[i].syn_pos_rpdo.time_Exponent.subindex)
            temp_value = self.node[i].syn_pos_rpdo.time_Mantissa.value.to_bytes(1, byteorder='little', signed=True)
            temp.DATA[4] = MST(temp_value[0])
            self.send_buffer = temp
            self.send()
            self.recv()

            # define the angle limitation
            temp = TPCANMsg()
            temp.ID = 0x600 + node_id[i]
            temp.LEN = 8
            temp.DATA[0] = MST(0x22)
            temp_index = self.node[i].syn_pos_rpdo.max_pos_target_limit.index.to_bytes(2, byteorder='little')
            temp.DATA[1] = MST(temp_index[0])
            temp.DATA[2] = MST(temp_index[1])
            temp.DATA[3] = MST(self.node[i].syn_pos_rpdo.max_pos_target_limit.subindex)
            temp_value = self.node[i].syn_pos_rpdo.max_pos_target_limit.value.to_bytes(4, byteorder='little',
                                                                                       signed=True)
            temp.DATA[4] = MST(temp_value[0])
            temp.DATA[5] = MST(temp_value[1])
            temp.DATA[6] = MST(temp_value[2])
            temp.DATA[7] = MST(temp_value[3])
            self.send_buffer = temp
            self.send()
            self.recv()

            temp.DATA[0] = MST(0x22)
            temp_index = self.node[i].syn_pos_rpdo.min_pos_target_limit.index.to_bytes(2, byteorder='little')
            temp.DATA[1] = MST(temp_index[0])
            temp.DATA[2] = MST(temp_index[1])
            temp.DATA[3] = MST(self.node[i].syn_pos_rpdo.min_pos_target_limit.subindex)
            temp_value = self.node[i].syn_pos_rpdo.min_pos_target_limit.value.to_bytes(4, byteorder='little',
                                                                                       signed=True)
            temp.DATA[4] = MST(temp_value[0])
            temp.DATA[5] = MST(temp_value[1])
            temp.DATA[6] = MST(temp_value[2])
            temp.DATA[7] = MST(temp_value[3])
            self.send_buffer = temp
            self.send()
            self.recv()

            temp.DATA[0] = MST(0x22)
            temp_index = self.node[i].syn_pos_rpdo.pos_limit_enable.index.to_bytes(2, byteorder='little')
            temp.DATA[1] = MST(temp_index[0])
            temp.DATA[2] = MST(temp_index[1])
            temp.DATA[3] = MST(self.node[i].syn_pos_rpdo.pos_limit_enable.subindex)
            temp_value = self.node[i].syn_pos_rpdo.pos_limit_enable.value.to_bytes(1, byteorder='little', signed=True)
            temp.DATA[4] = MST(temp_value[0])
            temp.DATA[5] = MST(0)
            temp.DATA[6] = MST(0)
            temp.DATA[7] = MST(0)
            self.send_buffer = temp
            self.send()
            self.recv()

            temp = TPCANMsg()
            temp.ID = 0x000
            temp.LEN = 2
            temp.DATA[0] = MST(0x01)
            temp.DATA[1] = MST(node_id[i])
            self.send_buffer = temp
            self.send()
            sleep(0.1)
        logging.info("RPDO init success")
        pass

    def syn_pos_ctrl(self, node_id, pos):
        """
        This method is used to control the node_idth motor's position
        node_id can be an array, and pos must be the same size with node_id
        :param node_id: the motor's id, example: [1,2,3]
        :param pos: the pos of the designed angle. pos is in degree unit, example: [200, 120, 0]
        :return:
        """

        if MODE_OF_OPERATION['CYCLIC SYNCHRONOUS POSITION'] != self.op_mode:
            logging.warning("the operation mode is not setted as CYCLIC SYNCHRONOUS POSITION, please check")
            return
        # node_id = list(node_id)
        try:
            node_id[0]
        except:
            node_id = [node_id]
        finally:
            pass
        # pos = list(pos)
        # print(node_id[0])
        try:
            pos[0]
        except:
            pos = [pos]
        finally:
            pass
        for i in range(len(node_id)):
            pos_data = (int(pos[i] / 360 * 0x00080000)).to_bytes(4, byteorder='little', signed=True)  # int32 to bytes
            # here, the pos need to be processed
            temp = TPCANMsg()
            temp.ID = self.node[self.ID.index(node_id[i])].syn_pos_rpdo.cob_id
            """here is an important change
               if we don't use this way,
               we can't choose only part of motor to move
               only the previous part well be controlled
               Jul 28, 2018"""
            temp.LEN = 4
            temp.DATA[0] = MST(pos_data[0])
            temp.DATA[1] = MST(pos_data[1])
            temp.DATA[2] = MST(pos_data[2])
            temp.DATA[3] = MST(pos_data[3])
            self.send_buffer = temp
            self.send()
        pass

    def pro_vel_init(self, node_id):
        pass

    def pro_vel_ctrl(self, node_id, vel):
        pass

    def pro_tor_init(self, node_id):
        pass

    def pro_tor_ctrl(self, node_id, tor):
        pass

    def syn_vel_init(self, node_id):
        """
        This method is used to init the velocity control pdo
        :param node_id: a node_id array
        :return:
        """
        flag = self.is_node_added(node_id)
        if not flag:
            logging.error("there is some node not added in the RgmNet, please check")
            exit()

        if self.state != 'OPERATION ENABLED':
            # print(self.state)
            logging.error("the POWER STATE must be OPERATION ENABLED, but current state is %s" % self.state)
            exit()

        if self.op_mode != MODE_OF_OPERATION['CYCLIC SYNCHRONOUS VELOCITY']:
            logging.warning(
                "the OPERATION MODE must be CYCLIC SYNCHRONOUS VELOCITY, but current state is %s" % self.op_mode)
            exit()
        logging.info("node_id is already added, power_state is right, and op mode is right, continue")
        try:
            node_id[0]
        except:
            node_id = [node_id]
        finally:
            pass
        pass
        for i in range(len(node_id)):
            temp = TPCANMsg()
            temp.ID = 0x600 + node_id[i]
            temp.LEN = 8
            temp_id = self.node[i].syn_vel_rpdo.cob_id.to_bytes(2, byteorder='little')
            # temp.DATA[0:8] = [MST(0x22), MST(0x15), MST(0x14), MST(0x01),
            #                   MST(temp_id[0]), MST(temp_id[1]), MST(0), MST(0)]
            temp.DATA[0] = MST(0x22)
            temp.DATA[1] = MST(0x15)
            temp.DATA[2] = MST(0x14)
            temp.DATA[3] = MST(0x01)
            temp.DATA[4] = MST(temp_id[0])
            temp.DATA[5] = MST(temp_id[1])
            self.send_buffer = temp
            self.send()
            self.recv()
            # temp.DATA[0:8] = [MST(0x22), MST(0x15), MST(0x14), MST(0x02),
            #                   MST(0xFE), MST(0), MST(0), MST(0)]
            temp.DATA[3] = MST(0x02)
            temp.DATA[4] = MST(0xFE)
            temp.DATA[5] = MST(0)
            self.send_buffer = temp
            self.send()
            self.recv()

            # temp_index = self.node[i].syn_vel_rpdo.time_Mantissa.index.to_bytes(2, byteorder='little')
            # temp.DATA[0:8] = [MST(temp_index[0]), MST(temp_index[1]),
            #                   MST(self.node[i].syn_vel_rpdo.time_Mantissa.subindex),
            #                   MST(self.node[i].syn_vel_rpdo.time_Mantissa.value),
            #                   MST(0), MST(0), MST(0), MST(0)]
            # temp.DATA[1] = MST(temp_index[0])
            # temp.DATA[2] = MST(temp_index[1])
            # temp.DATA[3] = MST(self.node[i].syn_vel_rpdo.time_Mantissa.subindex)
            # temp.DATA[4] = MST(self.node[i].syn_vel_rpdo.time_Mantissa.value)
            # self.send_buffer = temp
            # self.send()
            # temp_index = self.node[i].syn_vel_rpdo.time_Exponent.index.to_bytes(2, byteorder='little')
            # temp.DATA[0:8] = [MST(temp_index[0]), MST(temp_index[1]),
            #                   MST(self.node[i].syn_vel_rpdo.time_Exponent.subindex),
            #                   MST(self.node[i].syn_vel_rpdo.time_Exponent.value),
            #                   MST(0), MST(0), MST(0), MST(0)]
            # temp_index = self.node[i].syn_vel_rpdo.time_Exponent.index.to_bytes(2, byteorder='little')
            # temp.DATA[1] = MST(temp_index[0])
            # temp.DATA[2] = MST(temp_index[1])
            # temp.DATA[3] = MST(self.node[i].syn_vel_rpdo.time_Exponent.subindex)
            # temp_value = self.node[i].syn_vel_rpdo.time_Mantissa.value.to_bytes(1, byteorder='little', signed=True)
            # temp.DATA[4] = MST(temp_value[0])
            # self.send_buffer = temp
            # self.send()

            temp = TPCANMsg()
            temp.ID = 0x000
            temp.LEN = 2
            # temp.DATA[0:2] = [MST(0x01), MST(node_id[i])]
            temp.DATA[0] = MST(0x01)
            temp.DATA[1] = MST(node_id[i])
            self.send_buffer = temp
            self.send()
            sleep(0.1)
        logging.info("RPDO init success")
        pass

    def syn_vel_ctrl(self, node_id, vel):
        """
        This method is used to control the node_idth motor's velocity
        node_id can be an array, and velocity must be the same size with node_id
        :param node_id: the motor's id, example: [1,2,3]
        :param vel: the vel of the assigned motor, vel's unit is degree/s, example:[0,30,120]
        :return:
        """
        if MODE_OF_OPERATION['CYCLIC SYNCHRONOUS VELOCITY'] != self.op_mode:
            logging.warning("the operation mode is not setted as CYCLIC SYNCHRONOUS POSITION, please check")
            return
        try:
            node_id[0]
        except:
            node_id = [node_id]
        finally:
            pass
        # pos = list(pos)
        try:
            vel[0]
        except:
            vel = [vel]
        finally:
            pass

        for i in range(len(node_id)):
            vel_data = (int(vel[i] * 3.28 * 10 ** 4)).to_bytes(4, byteorder='little', signed=True)
            temp = TPCANMsg()
            temp.ID = self.node[i].syn_vel_rpdo.cob_id
            # print(temp.ID)
            temp.LEN = 4
            # temp.DATA[0:4] = [MST(vel_data[0]), MST(vel_data[1]), MST(vel_data[2]), MST(vel_data[3])]
            temp.DATA[0] = MST(vel_data[0])
            temp.DATA[1] = MST(vel_data[1])
            temp.DATA[2] = MST(vel_data[2])
            temp.DATA[3] = MST(vel_data[3])
            self.send_buffer = temp
            self.send()
            pass

    def syn_tor_init(self, node_id):
        """
        Prepare for cyclic synchronous torque control, init this control model
        :param node_id: the node_id of the RGM motor, for example: 1 or [1,2,3]
        :return: none
        """
        # check whether the node is added in the RgmNetwork
        flag = self.is_node_added(node_id)
        if not flag:
            logging.error("there is some node not added in the RgmNet, please check")
            exit()
        pass

        if self.state != 'OPERATION ENABLED':
            # print(self.state)
            logging.error("the POWER STATE must be OPERATION ENABLED, but current state is %s" % self.state)
            exit()

        if self.op_mode != MODE_OF_OPERATION['CYCLIC SYNCHRONOUS TORQUE']:
            logging.warning(
                "the OPERATION MODE must be CYCLIC SYNCHRONOUS TORQUE, but current state is %s" % self.op_mode)
            exit()

        logging.info("node_id is already added, and the mode is right, continue to init")
        try:
            node_id[0]
        except:
            node_id = [node_id]
        finally:
            pass

        for i in range(len(node_id)):
            temp = TPCANMsg()
            temp.ID = 0x600 + node_id[i]
            temp.LEN = 8
            temp.DATA[0] = MST(0x22)
            temp.DATA[1] = MST(0x16)
            temp.DATA[2] = MST(0x14)
            temp.DATA[3] = MST(0x01)
            temp_id = self.node[i].syn_tor_rpdo.cob_id.to_bytes(2, byteorder='little')
            temp.DATA[4] = MST(temp_id[0])
            temp.DATA[5] = MST(temp_id[1])
            self.send_buffer = temp
            self.send()
            self.recv()
            temp.DATA[3] = MST(0x02)
            temp.DATA[4] = MST(0xFE)  # def the RPDO type, 0xFE means response immediately
            temp.DATA[5] = MST(0x00)
            self.send_buffer = temp
            self.send()
            self.recv()

            # temp_index = self.node[i].syn_tor_rpdo.time_Mantissa.index.to_bytes(2, byteorder='little')
            # temp.DATA[1] = MST(temp_index[0])
            # temp.DATA[2] = MST(temp_index[1])
            # temp.DATA[3] = MST(self.node[i].syn_tor_rpdo.time_Mantissa.subindex)
            # temp.DATA[4] = MST(self.node[i].syn_tor_rpdo.time_Mantissa.value)
            # self.send_buffer = temp
            # self.send()
            # temp_index = self.node[i].syn_tor_rpdo.time_Exponent.index.to_bytes(2, byteorder='little')
            # temp.DATA[1] = MST(temp_index[0])
            # temp.DATA[2] = MST(temp_index[1])
            # temp.DATA[3] = MST(self.node[i].syn_tor_rpdo.time_Exponent.subindex)
            # temp_value = self.node[i].syn_tor_rpdo.time_Mantissa.value.to_bytes(1, byteorder='little', signed=True)
            # temp.DATA[4] = MST(temp_value[0])
            # self.send_buffer = temp
            # self.send()

            temp = TPCANMsg()
            temp.ID = 0x000
            temp.LEN = 2
            temp.DATA[0] = MST(0x01)
            temp.DATA[1] = MST(node_id[i])
            self.send_buffer = temp
            self.send()
            sleep(0.1)
        logging.info("RPDO init success")
        pass

    def syn_tor_ctrl(self, node_id, tor):
        """
        This method is used to control the node_idth motor's torque(current)
        node_id can be an array, and pos must be the same size with node_id
        :param node_id: the method's id, example: [1,2,3]
        :param tor: the designed torque of specific motor, torque is in Nm unit, example: [0.5 0.9 1]
        :return: none
        """

        if MODE_OF_OPERATION['CYCLIC SYNCHRONOUS TORQUE'] != self.op_mode:
            logging.warning("the operation mode is not setted as CYCLIC SYNCHRONOUS TORQUE, please check")
            return
        try:
            node_id[0]
        except:
            node_id = [node_id]
        finally:
            pass
        # pos = list(pos)
        try:
            tor[0]
        except:
            tor = [tor]
        finally:
            pass
        pass
        for i in range(len(node_id)):
            tor_data = (int(tor[i] * 2 ** 15 / 20)).to_bytes(2, byteorder='little', signed=True)  # int32 to bytes
            # print(tor_data)
            # here, the pos need to be processed
            temp = TPCANMsg()
            temp.ID = self.node[i].syn_tor_rpdo.cob_id
            temp.LEN = 2
            temp.DATA[0] = MST(tor_data[0])
            temp.DATA[1] = MST(tor_data[1])
            # print(tor_data[0], tor_data[1])
            # temp.DATA[2] = MST(tor_data[2])
            # temp.DATA[3] = MST(tor_data[3])
            self.send_buffer = temp
            self.send()
        pass

    def get_motion_para(self, node_id):
        """
        this method is used to get the position, velocity and acceleration
        :param node_id: the node_id array, define get get which motor'(s') parameter
        :return: a list, {str(node_id),[position, velocity, acceleration]}
        """
        # check whether the node is added in the RgmNetwork
        flag = self.is_node_added(node_id)
        if not flag:
            logging.error("there is some node not added in the RgmNet, please check")
            exit()
        pass
        # start to get parameter
        try:
            node_id[0]
        except:
            node_id = [node_id]
        finally:
            pass
        parameter = {}
        sleep(0.05)
        for i in range(len(node_id)):
            temp = TPCANMsg()
            temp.ID = 0x600 + node_id[i]
            temp.LEN = 8

            # get the position
            # 6064h is the position_actual_value
            temp.DATA[0] = MST(0x40)
            temp.DATA[1] = MST(0x64)
            temp.DATA[2] = MST(0x60)
            self.send_buffer = temp
            self.send()
            self.recv()
            temp_msg = self.recv_buffer
            # print(temp_msg.LEN)
            # print(temp_msg.DATA[4],bytes(temp_msg.DATA[4]))
            # print(temp_msg.DATA[5],bytes(temp_msg.DATA[5]))
            # print(temp_msg.DATA[6],bytes(temp_msg.DATA[6]))
            # print(temp_msg.DATA[7],bytes(temp_msg.DATA[7]))
            # print(temp_msg.DATA[0], temp_msg.DATA[1], temp_msg.DATA[2], temp_msg.DATA[3], temp_msg.DATA[4],
            #       temp_msg.DATA[5], temp_msg.DATA[6], temp_msg.DATA[7])
            temp_msg = bytes(MST(temp_msg.DATA[4])) + bytes(MST(temp_msg.DATA[5])) + bytes(
                MST(temp_msg.DATA[6])) + bytes(MST(temp_msg.DATA[7]))
            # print(temp_msg)
            position = int.from_bytes(temp_msg, byteorder='little', signed=True)
            # print(position)
            position = position / 0x00080000 * 360
            # get velocity
            # 606ch is the actual velocity
            temp.DATA[1] = MST(0x6C)
            temp.DATA[2] = MST(0x60)
            self.send_buffer = temp
            self.send()
            self.recv()
            temp_msg = self.recv_buffer
            # print(temp_msg.DATA[0], temp_msg.DATA[1], temp_msg.DATA[2])
            temp_msg = bytes(MST(temp_msg.DATA[4])) + bytes(MST(temp_msg.DATA[5])) + bytes(
                MST(temp_msg.DATA[6])) + bytes(MST(temp_msg.DATA[7]))
            velocity = int.from_bytes(temp_msg, byteorder='little', signed=True)
            velocity = velocity * 3.049 * 10 ** -5
            # get current
            # 6077h is the actual current
            temp.DATA[1] = MST(0x77)
            temp.DATA[2] = MST(0x60)
            self.send_buffer = temp
            self.send()
            self.recv()
            temp_msg = self.recv_buffer
            # print(temp_msg.DATA[0], temp_msg.DATA[1], temp_msg.DATA[2])
            temp_msg = bytes(MST(temp_msg.DATA[4])) + bytes(MST(temp_msg.DATA[5]))
            current = int.from_bytes(temp_msg, byteorder='little', signed=True)
            current = current * 20 / 2 ** 15
            parameter.update({str(node_id[i]): [position, velocity, current]})
            # parameter.update({str(node_id[i]): [position, position, position]})

        return parameter

    def send(self):
        """
        this method is used to send the data prepare
        :return:
        """
        try:
            self.__pcan.Write(self.__channel, self.send_buffer)
            sleep(0.003)
            # sleep(0.003)
            logging.info("success to send data, congratulation")
        except:
            logging.warning("fail to send data, please check")
        finally:
            pass

    def recv(self):
        try:
            state, msg, timestamp = self.__pcan.Read(self.__channel)
            # print("state is ", state)
            # print(msg.DATA[0], msg.DATA[1], msg.DATA[2], msg.DATA[3],
            #       msg.DATA[4], msg.DATA[5], msg.DATA[6], msg.DATA[7])
            self.recv_buffer = msg
            # sleep(0.005)
            sleep(0.003)
            logging.info("success to read data, congratulation")
        except:
            logging.warning("fail to read data, please check")
        finally:
            pass

    def is_node_added(self, node_id):
        """
        This method is used to check whether the node is already added in the self.ID
        :param node_id:
        :return:
        """
        try:
            node_id[0]
        except:
            node_id = [node_id]
        finally:
            pass
        flag = True
        for i in range(len(node_id)):
            try:
                self.ID.index(node_id[i])
            except:
                flag = 0
                logging.warning("node %d is not added in the RgmNet, please check" % node_id)
            finally:
                pass
        return flag
