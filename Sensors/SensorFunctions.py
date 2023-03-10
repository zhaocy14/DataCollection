import time

import serial
import serial.tools.list_ports


# for checksum in sensor information transmission
def char_checksum(data, byteorder='little'):
    """
    char_checksum 按字节计算校验和。每个字节被翻译为带符号整数
    @param data: 字节串
    @param byteorder: 大/小端
    """
    length = len(data)
    checksum = 0
    for i in range(0, length):
        x = int.from_bytes(data[i:i + 1], byteorder, signed=True)
        if x > 0 and checksum > 0:
            checksum += x
            if checksum > 0x7F:  # 上溢出
                checksum = (checksum & 0x7F) - 0x80  # 取补码就是对应的负数值
        elif x < 0 and checksum < 0:
            checksum += x
            if checksum < -0x80:  # 下溢出
                checksum &= 0x7F
        else:
            checksum += x  # 正负相加，不会溢出
        # print(checksum)
    return checksum


def uchar_checksum(data, byteorder='little', is_negate: bool = False):
    '''
    char_checksum 按字节计算校验和。每个字节被翻译为无符号整数
    @param data: 字节串
    @param byteorder: 大/小端
    '''
    length = len(data)
    checksum = 0
    for i in range(0, length):
        checksum += int.from_bytes(data[i:i + 1], byteorder, signed=False)
        checksum &= 0xFF  # 强制截断
    if is_negate:
        checksum ^= 0xFF
    return checksum


def print_serial(port):
    """Print the port information"""
    print("---------------[ %s ]---------------" % port.name)
    print("Path: %s" % port.device)
    print("Descript: %s" % port.description)
    print("HWID: %s" % port.hwid)
    if not None == port.manufacturer:
        print("Manufacture: %s" % port.manufacturer)
    if not None == port.product:
        print("Product: %s" % port.product)
    if not None == port.interface:
        print("Interface: %s" % port.interface)
    print()


def detect_serials(port_key: str, sensor_name: str):
    """
    detect and find the port
    return the port to open it
    """
    ports = serial.tools.list_ports.comports()
    for port in ports:
        # print(port.location)
        # print(port.description)
        # print(port.name)
        # print("\n\n")
        if port.description.__contains__(port_key):
            port_list = port.description
            port_path = port.device
            # print_serial(port)
            return port_path, port_list
        elif port.location.__contains__(port_key):
            port_list = port.description
            port_path = port.device
            # print_serial(port)
            return port_path, port_list
        elif port.serial_number is not None:
            if port.serial_number.__contains__(port_key):
                port_list = port.description
                port_path = port.device
                # print_serial(port)
                return port_path, port_list
    # if all ports do not match the target
    print("Cannot find the device: %s" % sensor_name)
    return None, None

def detect_ACM(ACM_number:int = 0):
    """
    detect and find the port
    return the port to open it
    """
    ports = serial.tools.list_ports.comports()
    port_name = "ACM" + str(ACM_number)
    for port in ports:
        if port.device.__contains__(port_name):
            print(port.location)
            print(port.description)
            print(port.name)
            print("\n")
            device = port.device

def detect_USB(USB_number: int = 0):
    """
    detect and find the port
    return the port to open it
    """
    ports = serial.tools.list_ports.comports()
    port_name = "USB" + str(USB_number)
    for port in ports:
        if port.device.__contains__(port_name):
            print(port.location)
            print(port.description)
            print(port.name)
            print("\n")
            device = port.device


if __name__ == "__main__":
    for i in range(5):
        detect_ACM(i)
