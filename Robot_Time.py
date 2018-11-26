from time import sleep, time


class Robot_Time(object):

    def __init__(self, period=1.0, sample_num=30, flag_real_time=True):
        """
        init the Robot_TIme system
        :param period:  period for single motion cycle
        :param sample_num:  sample_num for single motion cycle
        :param flag_real_time: flag, true, using real time mode. false, step time mode
        :output nothing
        """
        self.period = period
        self.sample_num = sample_num
        self.flag = flag_real_time

        self._current_time = 0
        self._current_index = 0
        self._start_stamp = 0

    def start(self):
        """
        this function can start or restart recording time
        :return: nothing
        """
        self._start_stamp = time()
        self._current_time = 0
        self._current_index = 0

    def restart(self):
        """
        restart timer for robot
        :return:
        """
        self.start()

    def current(self):
        """
        this time return current time in one period
        :return: current_time
        """
        current_time = 0
        if self.flag:
            current_time = (time() - self._start_stamp) % self.period
            self._current_time = current_time
        else:
            self._current_index += 1
            self._current_index = self._current_index % self.sample_num
            current_time = self._current_index / self.sample_num * self.period
            self._current_time = current_time
        return current_time

    def stop(self):
        """since now, do nothing"""
        pass
