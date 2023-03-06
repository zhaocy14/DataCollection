import os, sys
import numpy as np
#   DATA PATH
PWD = os.path.abspath(os.path.abspath(__file__))
FATHER_PATH = os.path.abspath(os.path.dirname(PWD) + os.path.sep + "..")
sys.path.append(FATHER_PATH)
DATA_PATH = os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + os.path.sep + ".." + os.path.sep + "Data")
# TODO: configuration parameter should not be updated by the github
#   PORT Location or description
CAMERA_LOCATION = "3-7"
CAMERA_BAUDRATE = 460800
IMU_LOCATION = "3-3.4"
# IMU_BAUDRATE = 9600 # 9 axis
IMU_BAUDRATE = 115200 # 6 axis
STM32_SERIAL_NUM = "0669"
STM32_BAUDRATE = 115200
STM32_SERIAL_DESCRIPTION = "STM32 STLink - ST-Link VCP Ctrl"
LIDAR_LOCATION_HIGH = "3-2.2.2"
LIDAR_LOCATION_LOW = "3-3.2"
SOFTSKIN_LOCATION = "3-2.1:1.0"
SOFTSKIN_BAUDRATE = 115200
INFRARED_LOCATION = "3-2.3"
INFRARED_BAUDRATE = 115200
GPS_LOCATION = "3-2.1:1.3"
GPS_BAUDRATE_v1 = 9600
GPS_BAUDRATE_v2 = 38400
BRAKE_LOCATION = "3-2.4"
BRAKE_BAUDRATE = 9600
HEARTRATE_V1_LOCATION = "3-2.1:1.2"
HEARTRATE_V1_BAUDRATE = 9600
POWER_LOCATION = "3-2.1:1.1"
POWER_BAUDRATE = 9600

# =======================================================================

#   Softskin Configuration
SKIN_SENSOR_NUM = 7
SKIN_TABLE_PRESSURE = [0, 0.5, 1, 1.5, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]  # for AC to pressure converting
SKIN_TABLE_AC = [51, 1912, 2724, 3011, 3163, 3340, 3455, 3522, 3572, 3608, 3633, 3656, 3680, 3697]
SKIN_MAX_THRESHOLD = 8  # Abnormal maximum pressure
SKIN_SAFE_CHANGE_RATE = 10  # Safe pressure change rate for unlocking the walker
SKIN_EMERGENCY_CHANGE_RATE = 50     # Abnormal pressure change rate for locking the walker

# Infrared Sensor
INFRARED_SENSOR_NUM = 11

#   LiDAR Configuration
# scanning configurations
SCAN_SIZE = 300
HALF_SIZE = int(SCAN_SIZE / 2)
# old version of filtering useless data
COLUMN_BOUNDARY = HALF_SIZE - 20
BOTTOM_BOUNDARY = HALF_SIZE - 100
FILTER_THETA = 150
# new version of filtering useless data
WALKER_TOP_BOUNDARY = 9
WALKER_BOTTOM_BOUNDARY = 50 + 50
WALKER_LEFT_BOUNDARY = 19
WALKER_RIGHT_BOUNDARY = 19
CENTER_TO_LIDAR = 50
# obstacle part
OBSTACLE_DISTANCE = 15  # 15 cm detection

#   Speaker USB port
SPEAKER_LOCATION_LOW_1 = "3-3.1:1.4"
SPEAKER_LOCATION_LOW_2 = "3-3.1:1.2"
SPEAKER_LOCATION_LOW_3 = "3-3.1:1.0"
SPEAKER_LOCATION_LOW_4 = "3-3.1:1.6"
SPEAKER_LOCATION_HIGH_1 = "3-3.3:1.2"
SPEAKER_LOCATION_HIGH_2 = "3-3.3:1.1"
SPEAKER_LOCATION_HIGH_3 = "3-3.3:1.0"
SPEAKER_LOCATION_HIGH_4 = "3-3.3:1.3"


SPEAKER_BAUDRATE = 9600
SPEAKER_DICT = {
    # command
    "redraw map": 1,
    "charge": 2,
    "start": 3,
    "sleep": 4,
    "voice menu off": 5,
    "hand operation": 6,
    # broadcast
    "Time_Out": 7,
    "Menu_Start": 8,
    "Sure?": 9,
    "Will": 10,
    "Half_Minute": 11,
    "Completed": 12,
    "help":13
}
SPEAKER_TIME_DICT = {
    # command
    "redraw map": 2,
    "charge": 1,
    "start": 2,
    "sleep": 2,
    "voice menu off": 2,
    "hand operation": 4,
    # broadcast
    "Time_Out": 4,
    "Menu_Start": 6,
    "Sure?": 2,
    "Will": 1,
    "Half_Minute": 7,
    "Completed": 2,
    "help":3
}
#   Speaker Command
PLAY_SONG = b'\xAA\x07\x02\x00'
PLAY = b'\xAA\x02\x00\xAC'
MIC_INI_TIME_GAP = 1 # unit:second
MAX_VOLUME = 30
MIN_VOLUME = 1
VOLUME_SET = b'\xAA\x13\x01'

# Microphone ports Note: The last number of MIC_SSL_port must be different from that of MIC_Traditional_SSL_port.
MIC_SSL_port="3-2.2.1"
MIC_Traditional_SSL_port="3-2.4"
RECORD_DEVICE_NAME = "USB Camera-B4.09.24.1"
