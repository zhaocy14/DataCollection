import numpy as np
import threading
import serial
import time
from Sensors.SensorConfig import *
from Sensors.SensorFunctions import *


class IMU(object):

    def __init__(self, is_bluetooth_version: bool = False, is_show: bool = False):
        """
        The IMU class. Equipped with serial to read the IMU sensor.
        You can access the IMU data by using function: get_imu_data()
        """
        self.port_name, self.port_list = detect_serials(port_key=IMU_LOCATION, sensor_name="IMU")
        self.serial = serial.Serial(self.port_name, IMU_BAUDRATE, timeout=None)

        """For data processing"""
        self.ACCData = [0.0] * 8
        self.GYROData = [0.0] * 8
        self.AngleData = [0.0] * 8
        self.GEOMAGNETData = [0.0] * 8
        self.FrameState = 0  # 通过0x后面的值判断属于哪一种情况
        self.Bytenum = 0  # 读取到这一段的第几位
        self.CheckSum = 0  # 求和校验位

        """Real data """
        self.a = [0.0] * 3  # angular acceleration speed
        self.w = [0.0] * 3  # omega
        self.Angle = [0.0] * 3  # absolute angle starting from the initialization
        self.GEOMagnet = [0.0] * 3  # Bluetooth version IMU can offer these data like a compass

        """bluetooth version or not"""
        self.bt_version = is_bluetooth_version

        """updating threading"""
        self.imu_threading = threading.Thread(target=self.read_record, args=(is_show,))
        self.imu_threading.start()

    def collect_all(self, show=False) -> None:  # 新增的核心程序，对读取的数据进行划分，各自读到对应的数组里
        """
        core function: decode the original bytes

        :param
            show: to display the acceleration:a, angular speed:w , absolute angle:Angle

        :return
            None

        """
        inputdata = self.serial.read(333)
        for data in inputdata:  # 在输入的数据进行遍历
            # data = ord(data)  # python 自动转换了
            if self.FrameState == 0:  # 当未确定状态的时候，进入以下判断
                if data == 0x55 and self.Bytenum == 0:  # 0x55位于第一位时候，开始读取数据，增大bytenum
                    self.CheckSum = data
                    self.Bytenum = 1
                    continue
                elif data == 0x51 and self.Bytenum == 1:  # 在byte不为0 且 识别到 0x51 的时候，改变frame
                    """acceleration"""
                    self.CheckSum += data
                    self.FrameState = 1
                    self.Bytenum = 2
                elif data == 0x52 and self.Bytenum == 1:  # 同理
                    """omega"""
                    self.CheckSum += data
                    self.FrameState = 2
                    self.Bytenum = 2
                elif data == 0x53 and self.Bytenum == 1:
                    """angle"""
                    self.CheckSum += data
                    self.FrameState = 3
                    self.Bytenum = 2
                elif data == 0x54 and self.Bytenum == 1:
                    """geomagnetism"""
                    self.CheckSum += data
                    self.FrameState = 4
                    self.Bytenum = 2
            elif self.FrameState == 1:  # acc    #已确定数据代表加速度
                if self.Bytenum < 10:  # 读取8个数据
                    self.ACCData[self.Bytenum - 2] = data  # 从0开始
                    self.CheckSum += data
                    self.Bytenum += 1
                else:
                    if data == (self.CheckSum & 0xff):  # 假如校验位正确
                        self.a = self.get_acc(self.ACCData)
                    self.CheckSum = 0  # 各数据归零，进行新的循环判断
                    self.Bytenum = 0
                    self.FrameState = 0
            elif self.FrameState == 2:  # gyro 角速度
                if self.Bytenum < 10:
                    self.GYROData[self.Bytenum - 2] = data
                    self.CheckSum += data
                    self.Bytenum += 1
                else:
                    if data == (self.CheckSum & 0xff):
                        self.w = self.get_gyro(self.GYROData)
                    self.CheckSum = 0
                    self.Bytenum = 0
                    self.FrameState = 0
            elif self.FrameState == 3:  # angle
                if self.Bytenum < 10:  # 读取8个数据
                    self.AngleData[self.Bytenum - 2] = data  # 从0开始
                    self.CheckSum += data
                    self.Bytenum += 1
                else:
                    if data == (self.CheckSum & 0xff):  # 假如校验位正确
                        self.Angle = self.get_angle(self.AngleData)
                    self.CheckSum = 0  # 各数据归零，进行新的循环判断
                    self.Bytenum = 0
                    self.FrameState = 0
            elif self.FrameState == 4:  # 地磁
                if self.Bytenum < 10:
                    self.GEOMAGNETData[self.Bytenum - 2] = data
                    self.CheckSum += data
                    self.Bytenum += 1
                else:
                    if data == (self.CheckSum & 0xff):
                        self.GEOMagnet = self.get_geo_magnet(self.GEOMAGNETData)
                    self.CheckSum = 0
                    self.Bytenum = 0
                    self.FrameState = 0
            if show:
                pass
                print("\rAcceleration:", self.a, "Omega:", self.w, "Angle:", self.Angle, end="")

    def collect_all_bluetooth_version(self, show: bool = False) -> None:
        """
        Bluetooth version of core function to decode the original data, basically we don't use this.
        The bluetooth version can read the Geomagnetic Angle. Seldom use.

        :param
            To display the decoded data.

        :return:
            None
        """
        self.serial.flushInput()
        # magnet_data = b'\xFF\xAA\x27\x3A\x00'
        # self.serial.write(magnet_data)
        inputdata = self.serial.read(20)
        for data in inputdata:
            if self.FrameState == 0:  # 当未确定状态的时候，进入以下判断
                if data == 0x55 and self.Bytenum == 0:  # 0x55位于第一位时候，开始读取数据，增大bytenum
                    self.Bytenum = 1
                    continue
                elif data == 0x61 and self.Bytenum == 1:  # 在byte不为0 且 识别到 0x51 的时候，改变frame
                    """acceleration omega angle """
                    self.FrameState = 1
                    self.Bytenum = 2
                elif data == 0x71 and self.Bytenum == 1:
                    """geo_magnet"""
                    self.FrameState = 2
                    self.Bytenum = 2
            elif self.FrameState == 1:
                if self.Bytenum < 8:  # 读取6个数据 acc
                    self.ACCData[self.Bytenum - 2] = data  # 从0开始
                    self.Bytenum += 1
                elif self.Bytenum < 14:
                    self.GYROData[self.Bytenum - 8] = data  # 从0开始
                    self.Bytenum += 1
                elif self.Bytenum < 20:
                    self.AngleData[self.Bytenum - 14] = data
                    self.Bytenum += 1
                else:
                    self.a = self.get_acc(self.ACCData)
                    self.w = self.get_gyro(self.GYROData)
                    self.Angle = self.get_angle(self.AngleData)
                    self.FrameState = 0
                    self.Bytenum = 0
        if show:
            print(self.a, self.w, self.Angle)
            pass

    """The following functions convert the bytes to the data. No need to understand and revise."""

    def get_acc(self, datahex):
        axl = datahex[0]
        axh = datahex[1]
        ayl = datahex[2]
        ayh = datahex[3]
        azl = datahex[4]
        azh = datahex[5]

        k_acc = 16.0

        acc_x = (axh << 8 | axl) / 32768.0 * k_acc
        acc_y = (ayh << 8 | ayl) / 32768.0 * k_acc
        acc_z = (azh << 8 | azl) / 32768.0 * k_acc
        if acc_x >= k_acc:
            acc_x -= 2 * k_acc
        if acc_y >= k_acc:
            acc_y -= 2 * k_acc
        if acc_z >= k_acc:
            acc_z -= 2 * k_acc

        return acc_x, acc_y, acc_z

    def get_gyro(self, datahex):
        wxl = datahex[0]
        wxh = datahex[1]
        wyl = datahex[2]
        wyh = datahex[3]
        wzl = datahex[4]
        wzh = datahex[5]
        k_gyro = 2000.0

        gyro_x = (wxh << 8 | wxl) / 32768.0 * k_gyro
        gyro_y = (wyh << 8 | wyl) / 32768.0 * k_gyro
        gyro_z = (wzh << 8 | wzl) / 32768.0 * k_gyro
        if gyro_x >= k_gyro:
            gyro_x -= 2 * k_gyro
        if gyro_y >= k_gyro:
            gyro_y -= 2 * k_gyro
        if gyro_z >= k_gyro:
            gyro_z -= 2 * k_gyro
        return gyro_x, gyro_y, gyro_z

    def get_angle(self, datahex):
        rxl = datahex[0]
        rxh = datahex[1]
        ryl = datahex[2]
        ryh = datahex[3]
        rzl = datahex[4]
        rzh = datahex[5]
        k_angle = 180.0

        angle_x = (rxh << 8 | rxl) / 32768.0 * k_angle
        angle_y = (ryh << 8 | ryl) / 32768.0 * k_angle
        angle_z = (rzh << 8 | rzl) / 32768.0 * k_angle
        if angle_x >= k_angle:
            angle_x -= 2 * k_angle
        if angle_y >= k_angle:
            angle_y -= 2 * k_angle
        if angle_z >= k_angle:
            angle_z -= 2 * k_angle
        return angle_x, angle_y, angle_z

    def get_geo_magnet(self, datahex):
        gxl = datahex[0]
        gxh = datahex[1]
        gyl = datahex[2]
        gyh = datahex[3]
        gzl = datahex[4]
        gzh = datahex[5]
        k_geo_magnet = 1
        geo_magnet_x = (gxh << 8 | gxl)
        geo_magnet_y = (gyh << 8 | gyl)
        geo_magnet_z = (gzh << 8 | gzl)
        # if geo_magnet_x >= k_geo_magnet:
        #     geo_magnet_x -= 2 * k_geo_magnet
        # if geo_magnet_y >= k_geo_magnet:
        #     geo_magnet_y -= 2 * k_geo_magnet
        # if geo_magnet_z >= k_geo_magnet:
        #     geo_magnet_z -= 2 * k_geo_magnet
        return geo_magnet_x, geo_magnet_y, geo_magnet_z

    def read_record(self, show: bool = False) -> None:
        """
        The threading function to continuously reading data from the serial port.

        :param
            show: to demo the data

        :return:
            None
        """
        while True:
            if self.bt_version:
                self.collect_all_bluetooth_version(show=show)
            else:
                self.collect_all(show=show)

    def get_imu_data(self) -> list:
        """
        You can achieve the imu data from this function.

        :return:
            The list combination of the acceleration x3, angular speed x3, absolute angle x3
        """
        return self.a + self.w + self.Angle


if __name__ == '__main__':
    """bluetooth version not recommended"""
    IMU_instance = IMU(is_bluetooth_version=False, is_show=False)
    time.sleep(3)
    data = IMU_instance.get_imu_data()
    print(data)
