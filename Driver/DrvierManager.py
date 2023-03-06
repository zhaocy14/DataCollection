from Sensors.SensorFunctions import *
import time
import numpy as np
import queue


class DriverManager(object):

    def __init__(self):
        super().__init__()
        self.lock_key = True
        self.v = 0  # linear velocity
        self.w = 0  # angular velocity
        self.r = 0  # turning radius
        self.queue = queue.Queue()
        self.obstacle = np.zeros((100,100))
