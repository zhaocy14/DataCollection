#!/usr/bin/env python
# -*- encoding: utf-8 -*-
import threading
import time
import numpy as np
import cv2
from PIL import Image
from Sensors.SensorConfig import *
from Sensors.SensorFunctions import *


class IRCamera(object):
    """
    The thermal camera port. In the early version, we call the camera as it use infrared light as the optical
    information. So the IRCamera stands for infrared camera = thermal camera.
    This module will automatically detect the serial of the thermal camera and decode the bytes data to a list. The
    thermal camera is an MLX90640 thermal camera. It will output a 24x32 images at a frequency of 4Hz.Among all the
    sensors we use, this thermal camera has the slowest communication frequency.
    When collecting data or do the inference, we recommend to read the thermal camera data first then require other
    sensors.

    """

    def __init__(self, ):
        # serial information
        self.port_name, self.port_list = detect_serials(port_key=CAMERA_LOCATION, sensor_name="Thermal Camera")
        self.serial = serial.Serial(self.port_name, CAMERA_BAUDRATE, timeout=None)

        # data processing
        self.head_size = 4
        self.head_self = []
        self.data_self = []

        # data output
        self.temperature = []

        # reading threading
        self.reading_thread = threading.Thread(target=self.continuous_reading, args=(False,))
        self.reading_thread.start()
        return

    def check_head_data(self, head_data):
        """
        This function is to detect the head of frame from IR Camera.
        :param
            head_data: The read data could be one frame.
        :return:
            a bool value to show whether the head data is correct
        """
        if head_data is None:
            print("check_head_data: the head data is None")
            head_data = []
        head = [0x5A, 0x5A, 0x02, 0x06]
        for i in range(self.head_size):
            if head_data[i] != head[i]:
                return False

        return True

    def __fix_pixel(self, ir_list=[]) -> list:
        """
        A function to fix broken pixel using method of interpolation.

        :param
            ir_list: One frame of thermal image with 24*32=768 pixels.

        :return:
            The fixed frame of data list.

        """
        # 768个像素依次排列的坐标，经验所得：坏的像素会因环境变化突出，在白背景下
        # 坏的像素的值会超过600
        need_fix_index = [183, 202, 303, 601]
        for i in need_fix_index:
            x = i % 32
            y = i // 32
            temp = (ir_list[x - 1 + (y - 1) * 32] +
                    ir_list[x + (y - 1) * 32] +
                    ir_list[x + 1 + (y - 1) * 32] +
                    ir_list[x - 1 + y * 32] +
                    ir_list[x + 1 + y * 32] +
                    ir_list[x - 1 + (y + 1) * 32] +
                    ir_list[x + (y + 1) * 32] +
                    ir_list[x + 1 + (y + 1) * 32]) / 8
            ir_list.insert(x + y * 32, temp)
            ir_list.pop(x + y * 32 + 1)
        return ir_list

    def get_irdata_once(self, demo: bool = False) -> list:
        """
        Read the serial. Return one frame of thermal image.

        :param
            demo: Display the thermal image using OpenCV.

        :return:
            One frame of data in a list.
        """
        temperature = []
        rest_num = 5
        while True:
            s = self.serial.read(1).hex()
            if s != "":
                s = int(s, 16)
            self.head_self.append(s)
            # 对头数据必须如此嵌套，否则无法区分上一帧的数据
            if len(self.head_self) == self.head_size:
                if self.check_head_data(self.head_self):
                    temp = self.serial.read(1540)
                    self.data_self.append(temp.hex())
                    self.head_self.clear()
                else:
                    self.head_self.pop(0)

                if len(self.data_self) == rest_num:
                    ir_data = self.data_self[rest_num - 1]
                    if len(ir_data) != 1540 * 2:
                        # 正常传过来一个字节 0xa5 是一个字节，一个元素表示4位， 然后用string表示一个字母就是一个字节
                        print("the array of ir_data is not 1540", len(ir_data))

                    for i in range(769):
                        t = (int(ir_data[i * 4 + 2:i * 4 + 4], 16) * 256 + int(ir_data[i * 4:i * 4 + 2], 16)) / 100
                        temperature.append(t)

                    """环境温度"""
                    temperature.pop()
                    temperature = self.__fix_pixel(temperature)

                    self.data_self.pop(rest_num - 1)
                    self.data_self.pop(0)
                if len(temperature) > 0:
                    self.temperature = temperature
                    break
            if demo:
                self.demonstrate_data()
        return temperature

    def continuous_reading(self, demo: bool = False) -> None:
        """
        Continuously reading the serial and updating the thermal images. Not recommend.

        :param
            demo: a bool value to decide whether to display the thermal image using OpenCV

        :return:
            None
        """
        head = []
        data = []
        rest_num = 5
        # time_previous = time.time()
        while True:
            s = self.serial.read(1).hex()
            if s != "":
                s = int(s, 16)
            head.append(s)

            if len(head) == self.head_size:
                if self.check_head_data(head):
                    temp = self.serial.read(1540)
                    data.append(temp.hex())
                    head.clear()
                else:
                    head.pop(0)

                # 将读到的数据进行展示
                if len(data) == rest_num:
                    ir_data = data[rest_num - 1]
                    if len(ir_data) != 1540 * 2:
                        # 正常传过来一个字节 0xa5 是一个字节，一个元素表示4位， 然后用string表示一个字母就是一个字节
                        print("the array of ir_data is not 1540", len(ir_data))

                    temperature = []

                    for i in range(769):
                        t = (int(ir_data[i * 4 + 2:i * 4 + 4], 16) * 256 + int(ir_data[i * 4:i * 4 + 2], 16)) / 100
                        temperature.append(t)

                    """环境温度不用"""
                    temperature.pop()
                    temperature = self.__fix_pixel(temperature)

                    self.temperature = temperature
                    data.pop(rest_num - 1)
                    data.pop(0)

                    if demo:
                        self.demonstrate_data()

    def demonstrate_data(self, scope=20):
        """
        Display the thermal image with OpenCV.
        :param
            scope: Picture magnification.
        :return:
            None
        """
        temperature = []

        if len(self.temperature) != 0:
            for i in self.temperature:
                temperature.append(i)
            if len(self.temperature) == 769:
                """leave time stamp"""
                temperature.pop(0)
            maxtemp = max(temperature)
            mintemp = min(temperature)
            for i in range(len(temperature)):
                temperature[i] = (temperature[i] - mintemp) / (maxtemp - mintemp)
            temperature = np.array(temperature, np.float32).reshape(24, 32)
            im = Image.fromarray(temperature)
            im = im.resize((32 * scope, 24 * scope), Image.BILINEAR)
            im = np.array(im)
            cv2.imshow("Foot", im)
            cv2.waitKey(1)


if __name__ == '__main__':
    ir_data = IRCamera()
    while True:
        ir_data.get_irdata_once(demo=True)
