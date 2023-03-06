#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   ControlDriver.py
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2019/10/16 21:46   msliu      1.0

@Description
------------
这是一个包含odoemtry和control的数字伺服器控制器

接受一个线速度v和角速度w，旋转半径r

根据线速度和角速度，结算实际转速(RPM-Revolutions Per Minute)

"""
import datetime
import threading
import time
from threading import Thread
from Driver import DriverSerial as DsD
from Driver import DriverMonitor as DM
from Driver import Odometry as odo
import matplotlib.pyplot as plt
import serial
import math
import os
import sys


# def singleton(cls, *args, **kw):
#     instances = {}
#
#     def _singleton():
#         if cls not in instances:
#             instances[cls] = cls(*args, **kw)
#         return instances[cls]
#
#     return _singleton
#
#
# @singleton

class ControlDriver(Thread):

    def __init__(self, radius_wheel=85.00, record_mode=False, radius=0, left_right=1):
        """
        :param radius_wheel:
        :param record_mode:
        :param radius:
        :param left_right:
            如果发现 左右轮数据反了
            将 0 改为 1
            或 1 改为 0
        """
        # radius_wheel = 52.55
        Thread.__init__(self)
        self.radius_wheel = radius_wheel
        self.record_mode = record_mode
        self.radius = radius
        self.speed = 0
        self.omega = 0.0
        self.position = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.count = 0
        driver = DsD.DigitalServoDriver(left_right=left_right)
        self.left_right = left_right
        baud_rate = driver.baud_rate
        self.ser_l = serial.Serial(driver.left, baud_rate, timeout=0.05)
        self.ser_r = serial.Serial(driver.right, baud_rate, timeout=0.05)
        self.monitor_l = DM.DriverMonitor()
        self.monitor_r = DM.DriverMonitor()
        self.plot_x = [0.0]
        self.plot_y = [0.0]

        self.DriverOnOff = False  # whether the driver is on(True) or off(False)
        self.StopMotorFlag = False  # a flag to stop the driver
        self.StartMotorFlag = False  # a flag to start the motor

        # 初始化时读取一次驱动器监控信息，记录初始时encoder位置
        self.CheckStatus()
        # 初始化Odometry
        # print('-------------------------------------------------------------------------------------------------------')
        # print('Initial LEFT monitor: ', self.monitor_l.monitorData)
        # print('Initial RIGHT monitor:', self.monitor_r.monitorData)
        Odo_l_init = self.monitor_l.monitorData['FeedbackPosition']
        Odo_r_init = self.monitor_r.monitorData['FeedbackPosition']
        # print('init: ', Odo_l_init, Odo_r_init)
        # print('-------------------------------------------------------------------------------------------------------')
        self.odo = odo.Odometry(X=0.0, Y=0.0, THETA=0.0, Odo_l=Odo_l_init, Odo_r=Odo_r_init)
        # time.sleep(2)

    def CheckStatus(self):
        # 读取一次驱动器监控信息
        # 读取左轮监控信息
        self.ser_l.write(bytes([0x80, 0x00, 0x80]))
        read_byte_l = self.ser_l.read(5)
        if read_byte_l[4] == 0x80:
            read_byte_l += self.ser_l.read(31)
        else:
            read_byte_l += self.ser_l.read(27)
        # 读取右轮监控信息
        self.ser_r.write(bytes([0x80, 0x00, 0x80]))
        read_byte_r = self.ser_r.read(5)
        if read_byte_r[4] == 0x80:
            read_byte_r += self.ser_r.read(31)
        else:
            read_byte_r += self.ser_r.read(27)
        self.monitor_l.processData(read_byte_l)
        self.monitor_r.processData(read_byte_r)

    def get_rpm_byte(self, rpm):
        rpm_byte = [0x06, 0x00, 0x88, 0x8e]
        rpm_hex = int(rpm / 6000 * 16384)
        if rpm_hex >= 0:
            rpm = [(rpm_hex & 0xFF00) >> 8, (rpm_hex & 0x00FF)]
        else:
            temp = 0xFFFF
            rpm_hex = temp + rpm_hex
            rpm = [(rpm_hex & 0xFF00) >> 8, (rpm_hex & 0x00FF)]
        rpm_byte[1] = rpm[0]
        rpm_byte[2] = rpm[1]
        rpm_byte.pop(3)
        last = 0
        for item in rpm_byte:
            last = last + item
        if last > 256:
            last = last & 0xFF
        rpm_byte.append(last)
        return rpm_byte

    def get_speed_rpm(self, w):
        rpm = w / (2 * math.pi * self.radius_wheel / 1000) * 60
        # print(int(rpm))
        return int(rpm)

    def get_rpm_Omega(self):
        """
        r * w = v = l (vr + vl)
                    -----------
                    2 (vr - vl)
        :return:
        """
        if self.omega > 0:
            vl = (self.radius + (56 / 2)) / 100 * self.omega
            vr = (self.radius - (56 / 2)) / 100 * self.omega
        else:
            vl = -(self.radius - (56 / 2)) / 100 * self.omega
            vr = -(self.radius + (56 / 2)) / 100 * self.omega
        # print(vl,vr)
        return vl, vr

    def control_part(self):
        # print("\n===================================== Start control part! =====================================")
        self.startMotor()
        # 如果 record_mode 是 True，则停掉电机，只记录数据
        if self.record_mode:
            self.stopMotor()

        while True:
            # the starting command, driver parameter update command, ending command will be activated one by one
            # the check status command sometimes cannot read data, so this part may cause error
            try:
                if self.StartMotorFlag:
                    # begin to start the motor
                    start = [0x00, 0x00, 0x01, 0x01]
                    pc_mode = [0x02, 0x00, 0xc4, 0xc6]
                    self.ser_l.write(bytes(start))
                    self.ser_l.read(2)
                    self.ser_r.write(bytes(start))
                    self.ser_r.read(2)
                    self.ser_l.write(bytes(pc_mode))
                    self.ser_l.read(2)
                    self.ser_r.write(bytes(pc_mode))
                    self.ser_r.read(2)
                if self.DriverOnOff:
                    vl, vr = self.get_rpm_Omega()
                    # 这里是个bug没修复，需要确保 self.speed 和 self.omega 只有一个有值（另一个需要为0）
                    if self.left_right == 1:
                        left = self.get_rpm_byte(self.get_speed_rpm(vl) + self.get_speed_rpm(self.speed))
                        right = self.get_rpm_byte(-(self.get_speed_rpm(vr) + self.get_speed_rpm(self.speed)))
                    else:
                        # print((self.get_speed_rpm(vl) + self.get_speed_rpm(self.speed)))
                        left = self.get_rpm_byte(self.get_speed_rpm(vl) + self.get_speed_rpm(self.speed))
                        right = self.get_rpm_byte(-(self.get_speed_rpm(vr) + self.get_speed_rpm(self.speed)))
                    # print(left, right)
                    self.ser_l.write(bytes(left))
                    self.ser_l.flush()
                    self.ser_l.read(2)
                    self.ser_r.write(bytes(right))
                    self.ser_r.flush()
                    self.ser_r.read(2)
                    time.sleep(0.1)
                    # update monitor and the odometry
                    self.CheckStatus()
                    self.odo.Odo_l = self.monitor_l.monitorData['FeedbackPosition']
                    self.odo.Odo_r = self.monitor_r.monitorData['FeedbackPosition']
                    # 更新位置
                    self.position = self.odo.updatePose(-self.odo.Odo_l, self.odo.Odo_r)
                    if math.sqrt(
                        (self.position[0] - self.plot_x[-1]) ** 2 + (self.position[1] - self.plot_y[-1]) ** 2) > 0.1:
                        self.plot_x.append(self.position[0])
                        self.plot_y.append(self.position[1])
                    # 若有故障
                    if self.monitor_l.monitorData["Malfunction"] or self.monitor_r.monitorData["Malfunction"]:
                        pass
                        # print('Left motor malfunction:  ' + self.monitor_l.monitorData["Malfunction"])
                        # print('Right motor malfunction: ' + self.monitor_r.monitorData["Malfunction"])
                        # self.flag_end = 1

                if self.StopMotorFlag:
                    end = [0x00, 0x00, 0x00, 0x00]
                    self.ser_l.write(bytes(end))
                    self.ser_l.read(2)
                    self.ser_r.write(bytes(end))
                    self.ser_r.read(2)
            except:
                pass
            self.ser_l.reset_input_buffer()
            self.ser_r.reset_input_buffer()
        pass

    def stopMotor(self):  # 关闭电机，同时关闭刹车
        """
            This will deactivate the motor so that we can manually push the walker.
        :return: None
        """
        # when stopping, there should not be communication between the NUC and the driver
        if self.DriverOnOff:
            print("Driver is off. Manual mode activated.")
            self.StopMotorFlag = True
            self.StartMotorFlag = False
            self.DriverOnOff = False

    def startMotor(self):  # 开启电机
        """
            This will activate the motor. This means that the motor is power on. There will be resistance if we
        manually push the walker.
        :return: None
        """
        if not self.DriverOnOff:
            print("Driver is on")
            self.StartMotorFlag = True
            self.StopMotorFlag = False
            self.DriverOnOff = True

    def clear_error(self):
        """
        Send the error clearing bytes to the wheels. Sometimes the wheel will be overloaded and will not respond.
        :return: None
        """
        # must manually activate
        clear = [0x4a, 0x00, 0x00, 0x4a]
        self.ser_l.write(bytes(clear))
        self.ser_l.read(2)
        self.ser_r.write(bytes(clear))
        self.ser_r.read(2)

    def destroyMotor(self):
        sys.exit()

    def run(self):
        self.control_part()

    def get_driver_data(self):
        """
        Get the driver's data.
        :return:
        """
        return self.position


if __name__ == '__main__':
    cd = ControlDriver(record_mode=False, left_right=1)
    # you can initialize the threading of control driver like:
    cd.start()

    # just update the parameter and the main thread will change the driver setting
    cd.speed = cd.omega = cd.radius = 0

    # !!!! must manually, this is for clear error
    # cd.ClearError()
