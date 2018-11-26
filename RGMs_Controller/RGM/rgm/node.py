import configparser

MODE_OF_OPERATION = {
    'PROFILE POSITION': 0x0180,
    'PROFILE VELOCITY': 0x0200,
    'PROFILE TORQUE': 0x0220,
    'CYCLIC SYNCHRONOUS POSITION': 0x0240,
    'CYCLIC SYNCHRONOUS VELOCITY': 0x0260,
    'CYCLIC SYNCHRONOUS TORQUE': 0x0280
}


class Node(object):

    def __init__(self, motor_id, configname):
        self.configfile = configname
        self.id = motor_id
        self.pro_pos_rpdo = PdoConfig(motor_id, 'PROFILE POSITION')
        self.syn_pos_rpdo = PdoConfig(motor_id, 'CYCLIC SYNCHRONOUS POSITION')
        self.syn_vel_rpdo = PdoConfig(motor_id, 'CYCLIC SYNCHRONOUS VELOCITY')
        self.syn_tor_rpdo = PdoConfig(motor_id, 'CYCLIC SYNCHRONOUS TORQUE')

    def load_config(self):
        config = configparser.ConfigParser()
        config.read(self.configfile)
        """
        set the parameter needed by node
        """

    def save_config(self):
        pass


class PdoConfig(object):

    def __init__(self, motor_id, mode):
        self.cob_id = MODE_OF_OPERATION[mode] + motor_id
        # self.time_Mantissa = SdoVariable(0x60C2, 0x01, 0x10, 1)
        self.time_Mantissa = SdoVariable(0x60C2, 0x01, 0x03, 1)
        # self.time_Exponent = SdoVariable(0x60C2, 0x02, -0x02, 1)
        self.time_Exponent = SdoVariable(0x60C2, 0x02, -0x02, 1)
        self.max_pos_target_limit = SdoVariable(0x2039, 0x08, 0x00080000, 4)
        self.min_pos_target_limit = SdoVariable(0x2039, 0x09, 0x00000000, 4)
        self.pos_limit_enable = SdoVariable(0x2039, 0x0a, 0x00, 4)
        # self.pos_limit_enable = SdoVariable(0x2039, 0x0a, 0x03, 4)


class SdoVariable(object):

    def __init__(self, index, subindex, value, length):
        self.index = index
        self.subindex = subindex
        self.value = value
        self.length = length
