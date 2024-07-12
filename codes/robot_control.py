import serial
import struct
import socket
import numpy as np
import time
import runs
from itertools import islice

# 机械臂IP地址和端口，不变量
HOST = "192.168.3.6"
PORT = 30003

# 定义机械臂的常量
tool_acc = 0.4  # Safe: 0.5
tool_vel = 0.05  # Safe: 0.2
PI = 3.141592653589
fmt1 = '<I'
fmt2 = '<6d'
BUFFER_SIZE = 1108
buffsize = 1108


class Robot:
    def __init__(self, host=None, port=None):
        # 创建socket对象，然后连接
        self.recv_buf = []
        if host is None and port is None:
            host = HOST
            port = PORT
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((HOST, PORT))

    # 控制机械臂末端位姿
    def robot_pose_control(self, target_tcp):
        tcp_command = "movel(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0)\n" % \
                      (target_tcp[0] / 1000, target_tcp[1] / 1000, target_tcp[2] / 1000,
                       target_tcp[3], target_tcp[4], target_tcp[5],
                       tool_acc, tool_vel)

        print(tcp_command)
        # 字符串发送
        self.tcp_socket.send(str.encode(tcp_command))

    # 控制机械臂关节角
    def robot_angle_control(self, target_tcp):
        tcp_command = "movej([%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0)\n" % \
                      (target_tcp[0] * PI / 180.0, target_tcp[1] * PI / 180.0, target_tcp[2] * PI / 180.0,
                       target_tcp[3] * PI / 180.0, target_tcp[4] * PI / 180.0, target_tcp[5] * PI / 180.0,
                       tool_acc, tool_vel)
        print(tcp_command)
        # 字符串发送
        self.tcp_socket.send(str.encode(tcp_command))

    # 接收机械臂信息
    def robot_msg_recv(self):
        self.recv_buf = []
        self.recv_buf = self.tcp_socket.recv(BUFFER_SIZE)
        # 解析数据
        if len(self.recv_buf) == 1108:
            pack_len = struct.unpack(fmt1, self.byte_swap(self.recv_buf[:4]))[0]
            # print("pack_len: ", pack_len)

            # 解析机器人位置数据
            pos1 = 12  # 第13个字节的位置为12
            pos2 = pos1 + 48  # 第60个字节的位置为pos1+48
            data1 = self.byte_swap(self.recv_buf[pos1:pos2])
            data2 = np.frombuffer(data1, dtype=fmt2)
            new_data1 = np.around(np.rad2deg(data2[::-1]), 2)
            new_data1_str = [str(value) for value in new_data1[0][::-1]]

            # 机器人关节角度
            # print(new_data1_str)

            # 解析机器人关节角度数据
            pos3 = 444  # 第445个字节的位置为444
            pos4 = pos3 + 48  # 第492个字节的位置为pos3+48
            data3 = self.byte_swap(self.recv_buf[pos3:pos4])
            data4 = np.frombuffer(data3, dtype=fmt2)
            new_data2 = np.around(data4[::-1] * 1000, 2)
            # new_data2_str = [str(value) for value in new_data2[0][::-1]]
            new_data2_str = [f"{value/1000:.2f}" if i >= len(new_data2[0]) - 3
                             else str(value) for i, value in enumerate(new_data2[0][::-1])]

            # 机器人末端位姿
            # print(new_data2_str)

            return new_data1_str, new_data2_str

    def byte_swap(self, data):
        return data[::-1]

    def robot_close(self):
        self.tcp_socket.close()


class InspireHandR:
    # def __init__(self):
    #     # 串口设置
    #     plist = list(serial.tools.list_ports.comports())
    #     if len(plist) <= 0:
    #         print("串口没找到")
    #     else:
    #         plist_0 = list(plist[0])
    #         serialName = plist_0[0]
    #         self.ser = serial.Serial(serialName, 115200)
    #     self.ser.isOpen()
    #     self.hand_id = 1
    #     self.reset()
    def __init__(self):
        # 串口设置
        self.f1_init_pos = 0  # 小拇指伸直0，弯曲2000
        self.f2_init_pos = 0  # 无名指伸直0，弯曲2000
        self.f3_init_pos = 0  # 中指伸直0，弯曲2000
        self.f4_init_pos = 0  # 食指伸直0，弯曲2000
        self.f5_init_pos = 0  # 大拇指伸直0，弯曲2000
        self.f6_init_pos = 0  # 大拇指侧摆0，2000
        # self.ser = serial.Serial('/dev/tty' + serial_signal, 115200)
        self.ser = serial.Serial('COM3', 115200)
        self.ser.isOpen()

        self.hand_id = 1
        power1 = 200
        power2 = 200
        power3 = 200
        power4 = 200
        power5 = 200
        power6 = 200
        self.setpower(power1, power2, power3, power4, power5, power6)
        speed1 = 200
        speed2 = 200
        speed3 = 200
        speed4 = 200
        speed5 = 200
        speed6 = 200
        self.setspeed(speed1, speed2, speed3, speed4, speed5, speed6)
        self.reset()

    # 把数据分成高字节和低字节
    def data2bytes(self, data):
        rdata = [0xff] * 2
        if data == -1:
            rdata[0] = 0xff
            rdata[1] = 0xff
        else:
            rdata[0] = data & 0xff
            rdata[1] = (data >> 8) & 0xff
        return rdata

    # 把十六进制或十进制的数转成bytes
    def num2str(self, num):
        str = hex(num)
        str = str[2:4]
        if len(str) == 1:
            str = '0' + str
        str = bytes.fromhex(str)
        # print(str)
        return str

    # 求校验和
    def checknum(self, data, leng):
        result = 0
        for i in range(2, leng):
            result += data[i]
        result = result & 0xff
        # print(result)
        return result

    # 设置电机驱动位置
    def setpos(self, pos1, pos2, pos3, pos4, pos5, pos6):
        global hand_id
        if pos1 < -1 or pos1 > 2000:
            print('数据超出正确范围：-1-2000')
            return
        if pos2 < -1 or pos2 > 2000:
            print('数据超出正确范围：-1-2000')
            return
        if pos3 < -1 or pos3 > 2000:
            print('数据超出正确范围：-1-2000')
            return
        if pos4 < -1 or pos4 > 2000:
            print('数据超出正确范围：-1-2000')
            return
        if pos5 < -1 or pos5 > 2000:
            print('数据超出正确范围：-1-2000')
            return
        if pos6 < -1 or pos6 > 2000:
            print('数据超出正确范围：-1-2000')
            return

        datanum = 0x0F
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90

        # hand_id号
        b[2] = self.hand_id

        # 数据个数
        b[3] = datanum

        # 写操作
        b[4] = 0x12

        # 地址
        b[5] = 0xC2
        b[6] = 0x05

        # 数据
        b[7] = self.data2bytes(pos1)[0]
        b[8] = self.data2bytes(pos1)[1]

        b[9] = self.data2bytes(pos2)[0]
        b[10] = self.data2bytes(pos2)[1]

        b[11] = self.data2bytes(pos3)[0]
        b[12] = self.data2bytes(pos3)[1]

        b[13] = self.data2bytes(pos4)[0]
        b[14] = self.data2bytes(pos4)[1]

        b[15] = self.data2bytes(pos5)[0]
        b[16] = self.data2bytes(pos5)[1]

        b[17] = self.data2bytes(pos6)[0]
        b[18] = self.data2bytes(pos6)[1]

        # 校验和
        b[19] = self.checknum(b, datanum + 4)

        # 向串口发送数据
        putdata = b''

        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
        self.ser.write(putdata)
        # print('发送的数据：',putdata)

        print('发送的实际十六进制数据：')
        for i in range(1,datanum+6):
            print(hex(putdata[i-1]))

        # getdata = self.ser.read(9)
        # print('返回的数据：',getdata)
        # print('返回的数据：')
        # for i in range(1,10):
        # print(hex(getdata[i-1]))
        return

    # 设置弯曲角度
    def setangle(self, angle1, angle2, angle3, angle4, angle5, angle6):
        if angle1 < -1 or angle1 > 1000:
            print('数据超出正确范围：-1-1000')
            return
        if angle2 < -1 or angle2 > 1000:
            print('数据超出正确范围：-1-1000')
            return
        if angle3 < -1 or angle3 > 1000:
            print('数据超出正确范围：-1-1000')
            return
        if angle4 < -1 or angle4 > 1000:
            print('数据超出正确范围：-1-1000')
            return
        if angle5 < -1 or angle5 > 1000:
            print('数据超出正确范围：-1-1000')
            return
        if angle6 < -1 or angle6 > 1000:
            print('数据超出正确范围：-1-1000')
            return

        datanum = 0x0F
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90

        # hand_id号
        b[2] = self.hand_id

        # 数据个数
        b[3] = datanum

        # 写操作
        b[4] = 0x12

        # 地址
        b[5] = 0xCE
        b[6] = 0x05

        # 数据
        b[7] = self.data2bytes(angle1)[0]
        b[8] = self.data2bytes(angle1)[1]

        b[9] = self.data2bytes(angle2)[0]
        b[10] = self.data2bytes(angle2)[1]

        b[11] = self.data2bytes(angle3)[0]
        b[12] = self.data2bytes(angle3)[1]

        b[13] = self.data2bytes(angle4)[0]
        b[14] = self.data2bytes(angle4)[1]

        b[15] = self.data2bytes(angle5)[0]
        b[16] = self.data2bytes(angle5)[1]

        b[17] = self.data2bytes(angle6)[0]
        b[18] = self.data2bytes(angle6)[1]

        # 校验和
        b[19] = self.checknum(b, datanum + 4)

        # 向串口发送数据
        putdata = b''

        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
        self.ser.write(putdata)
        print('发送的实际十六进制数据：')
        for i in range(1, datanum + 6):
            print(hex(putdata[i - 1]))

        getdata = self.ser.read(9)
        print('返回的数据：')
        for i in range(1, 10):
            print(hex(getdata[i - 1]))

    # 设置小拇指弯曲角度
    def setlittleangle(self, angle1):
        if angle1 < -1 or angle1 > 1000:
            print('数据超出正确范围：-1-1000')
            return

        datanum = 0x05
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90

        # hand_id号
        b[2] = self.hand_id

        # 数据个数
        b[3] = datanum

        # 写操作
        b[4] = 0x12

        # 地址
        b[5] = 0xCE
        b[6] = 0x05

        # 数据
        b[7] = self.data2bytes(angle1)[0]
        b[8] = self.data2bytes(angle1)[1]

        # 校验和
        b[9] = self.checknum(b, datanum + 4)

        # 向串口发送数据
        putdata = b''

        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
        self.ser.write(putdata)
        print('发送的实际十六进制数据：')
        for i in range(1, datanum + 6):
            print(hex(putdata[i - 1]))

        getdata = self.ser.read(9)
        print('返回的数据：')
        for i in range(1, 10):
            print(hex(getdata[i - 1]))

    # 设置食指弯曲角度
    def setringangle(self, angle1):
        if angle1 < -1 or angle1 > 1000:
            print('数据超出正确范围：-1-1000')
            return

        datanum = 0x05
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90

        # hand_id号
        b[2] = self.hand_id

        # 数据个数
        b[3] = datanum

        # 写操作
        b[4] = 0x12

        # 地址
        b[5] = 0xD0
        b[6] = 0x05

        # 数据
        b[7] = self.data2bytes(angle1)[0]
        b[8] = self.data2bytes(angle1)[1]

        # 校验和
        b[9] = self.checknum(b, datanum + 4)

        # 向串口发送数据
        putdata = b''

        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
        self.ser.write(putdata)
        print('发送的实际十六进制数据：')
        for i in range(1, datanum + 6):
            print(hex(putdata[i - 1]))

        getdata = self.ser.read(9)
        print('返回的数据：')
        for i in range(1, 10):
            print(hex(getdata[i - 1]))

    # 设置中指弯曲角度
    def setmiddleangle(self, angle1):
        if angle1 < -1 or angle1 > 1000:
            print('数据超出正确范围：-1-1000')
            return

        datanum = 0x05
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90

        # hand_id号
        b[2] = self.hand_id

        # 数据个数
        b[3] = datanum

        # 写操作
        b[4] = 0x12

        # 地址
        b[5] = 0xD2
        b[6] = 0x05

        # 数据
        b[7] = self.data2bytes(angle1)[0]
        b[8] = self.data2bytes(angle1)[1]

        # 校验和
        b[9] = self.checknum(b, datanum + 4)

        # 向串口发送数据
        putdata = b''

        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
        self.ser.write(putdata)
        print('发送的实际十六进制数据：')
        for i in range(1, datanum + 6):
            print(hex(putdata[i - 1]))

        getdata = self.ser.read(9)
        print('返回的数据：')
        for i in range(1, 10):
            print(hex(getdata[i - 1]))

    # 设置食指弯曲角度
    def setindexangle(self, angle1):
        if angle1 < -1 or angle1 > 1000:
            print('数据超出正确范围：-1-1000')
            return

        datanum = 0x05
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90

        # hand_id号
        b[2] = self.hand_id

        # 数据个数
        b[3] = datanum

        # 写操作
        b[4] = 0x12

        # 地址
        b[5] = 0xD4
        b[6] = 0x05

        # 数据
        b[7] = self.data2bytes(angle1)[0]
        b[8] = self.data2bytes(angle1)[1]

        # 校验和
        b[9] = self.checknum(b, datanum + 4)

        # 向串口发送数据
        putdata = b''

        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
        self.ser.write(putdata)
        print('发送的实际十六进制数据：')
        for i in range(1, datanum + 6):
            print(hex(putdata[i - 1]))

        getdata = self.ser.read(9)
        print('返回的数据：')
        for i in range(1, 10):
            print(hex(getdata[i - 1]))

    # 设置大拇指弯曲角度
    def setthumbangle(self, angle1):
        if angle1 < -1 or angle1 > 1000:
            print('数据超出正确范围：-1-1000')
            return

        datanum = 0x05
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90

        # hand_id号
        b[2] = self.hand_id

        # 数据个数
        b[3] = datanum

        # 写操作
        b[4] = 0x12

        # 地址
        b[5] = 0xD6
        b[6] = 0x05

        # 数据
        b[7] = self.data2bytes(angle1)[0]
        b[8] = self.data2bytes(angle1)[1]

        # 校验和
        b[9] = self.checknum(b, datanum + 4)

        # 向串口发送数据
        putdata = b''

        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
        self.ser.write(putdata)
        print('发送的实际十六进制数据：')
        for i in range(1, datanum + 6):
            print(hex(putdata[i - 1]))

        getdata = self.ser.read(9)
        print('返回的数据：')
        for i in range(1, 10):
            print(hex(getdata[i - 1]))

    # 设置侧摆角度
    def setswingangle(self, angle1):
        if angle1 < -1 or angle1 > 1000:
            print('数据超出正确范围：-1-1000')
            return

        datanum = 0x05
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90

        # hand_id号
        b[2] = self.hand_id

        # 数据个数
        b[3] = datanum

        # 写操作
        b[4] = 0x12

        # 地址
        b[5] = 0xD8
        b[6] = 0x05

        # 数据
        b[7] = self.data2bytes(angle1)[0]
        b[8] = self.data2bytes(angle1)[1]

        # 校验和
        b[9] = self.checknum(b, datanum + 4)

        # 向串口发送数据
        putdata = b''

        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
        self.ser.write(putdata)
        print('发送的实际十六进制数据：')
        for i in range(1, datanum + 6):
            print(hex(putdata[i - 1]))

        getdata = self.ser.read(9)
        print('返回的数据：')
        for i in range(1, 10):
            print(hex(getdata[i - 1]))

    # 设置力控阈值 安全值200
    def setpower(self, power1, power2, power3, power4, power5, power6):
        if power1 < 0 or power1 > 1000:
            print('数据超出正确范围：0-1000')
            return
        if power2 < 0 or power2 > 1000:
            print('数据超出正确范围：0-1000')
            return
        if power3 < 0 or power3 > 1000:
            print('数据超出正确范围：0-1000')
            return
        if power4 < 0 or power4 > 1000:
            print('数据超出正确范围：0-1000')
            return
        if power5 < 0 or power5 > 1000:
            print('数据超出正确范围：0-1000')
            return
        if power6 < 0 or power6 > 1000:
            print('数据超出正确范围：0-1000')
            return

        datanum = 0x0F
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90

        # hand_id号
        b[2] = self.hand_id

        # 数据个数
        b[3] = datanum

        # 写操作
        b[4] = 0x12

        # 地址
        b[5] = 0xDA
        b[6] = 0x05

        # 数据
        b[7] = self.data2bytes(power1)[0]
        b[8] = self.data2bytes(power1)[1]

        b[9] = self.data2bytes(power2)[0]
        b[10] = self.data2bytes(power2)[1]

        b[11] = self.data2bytes(power3)[0]
        b[12] = self.data2bytes(power3)[1]

        b[13] = self.data2bytes(power4)[0]
        b[14] = self.data2bytes(power4)[1]

        b[15] = self.data2bytes(power5)[0]
        b[16] = self.data2bytes(power5)[1]

        b[17] = self.data2bytes(power6)[0]
        b[18] = self.data2bytes(power6)[1]

        # 校验和
        b[19] = self.checknum(b, datanum + 4)

        # 向串口发送数据
        putdata = b''

        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
        self.ser.write(putdata)
        print('发送的数据：')
        for i in range(1, datanum + 6):
            print(hex(putdata[i - 1]))

        getdata = self.ser.read(9)
        print('返回的数据：')
        for i in range(1, 10):
            print(hex(getdata[i - 1]))

    # 设置运动速度 安全值200
    def setspeed(self, speed1, speed2, speed3, speed4, speed5, speed6):
        if speed1 < 0 or speed1 > 1000:
            print('数据超出正确范围：0-1000')
            return
        if speed2 < 0 or speed2 > 1000:
            print('数据超出正确范围：0-1000')
            return
        if speed3 < 0 or speed3 > 1000:
            print('数据超出正确范围：0-1000')
            return
        if speed4 < 0 or speed4 > 1000:
            print('数据超出正确范围：0-1000')
            return
        if speed5 < 0 or speed5 > 1000:
            print('数据超出正确范围：0-1000')
            return
        if speed6 < 0 or speed6 > 1000:
            print('数据超出正确范围：0-1000')
            return

        datanum = 0x0F
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90

        # hand_id号
        b[2] = self.hand_id

        # 数据个数
        b[3] = datanum

        # 写操作
        b[4] = 0x12

        # 地址
        b[5] = 0xF2
        b[6] = 0x05

        # 数据
        b[7] = self.data2bytes(speed1)[0]
        b[8] = self.data2bytes(speed1)[1]

        b[9] = self.data2bytes(speed2)[0]
        b[10] = self.data2bytes(speed2)[1]

        b[11] = self.data2bytes(speed3)[0]
        b[12] = self.data2bytes(speed3)[1]

        b[13] = self.data2bytes(speed4)[0]
        b[14] = self.data2bytes(speed4)[1]

        b[15] = self.data2bytes(speed5)[0]
        b[16] = self.data2bytes(speed5)[1]

        b[17] = self.data2bytes(speed6)[0]
        b[18] = self.data2bytes(speed6)[1]

        # 校验和
        b[19] = self.checknum(b, datanum + 4)

        # 向串口发送数据
        putdata = b''

        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
        self.ser.write(putdata)
        # print('发送的数据：')
        # for i in range(1, datanum + 6):
        #     print(hex(putdata[i - 1]))
        #
        getdata = self.ser.read(9)
        # print('返回的数据：')
        # for i in range(1, 10):
        #     print(hex(getdata[i - 1]))

    # 读取驱动器实际的位置值
    def get_setpos(self):
        datanum = 0x04
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90

        # hand_id号
        b[2] = self.hand_id

        # 数据个数
        b[3] = datanum

        # 读操作
        b[4] = 0x11

        # 地址
        b[5] = 0xC2
        b[6] = 0x05

        # 读取寄存器的长度
        b[7] = 0x0C

        # 校验和
        b[8] = self.checknum(b, datanum + 4)

        # 向串口发送数据
        putdata = b''

        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
        self.ser.write(putdata)
        # print('发送的数据：')
        # for i in range(1, datanum + 6):
        #     print(hex(putdata[i - 1]))

        getdata = self.ser.read(20)
        # print('返回的数据：')
        # for i in range(1, 21):
        #     print(hex(getdata[i - 1]))

        setpos = [0] * 6
        for i in range(1, 7):
            if getdata[i * 2 + 5] == 0xff and getdata[i * 2 + 6] == 0xff:
                setpos[i - 1] = -1
            else:
                setpos[i - 1] = getdata[i * 2 + 5] + (getdata[i * 2 + 6] << 8)
        print("驱动器实际值： ", setpos)
        return setpos

    # 读取设置角度
    def get_setangle(self):
        datanum = 0x04
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90

        # hand_id号
        b[2] = self.hand_id

        # 数据个数
        b[3] = datanum

        # 读操作
        b[4] = 0x11

        # 地址
        b[5] = 0xCE
        b[6] = 0x05

        # 读取寄存器的长度
        b[7] = 0x0C

        # 校验和
        b[8] = self.checknum(b, datanum + 4)

        # 向串口发送数据
        putdata = b''

        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
        self.ser.write(putdata)
        print('发送的数据：')
        for i in range(1, datanum + 6):
            print(hex(putdata[i - 1]))

        getdata = self.ser.read(20)
        print('返回的数据：')
        for i in range(1, 21):
            print(hex(getdata[i - 1]))

        setangle = [0] * 6
        for i in range(1, 7):
            if getdata[i * 2 + 5] == 0xff and getdata[i * 2 + 6] == 0xff:
                setangle[i - 1] = -1
            else:
                setangle[i - 1] = getdata[i * 2 + 5] + (getdata[i * 2 + 6] << 8)
        return setangle

    # 读取驱动器设置的力控阈值
    def get_setpower(self):
        datanum = 0x04
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90

        # hand_id号
        b[2] = self.hand_id

        # 数据个数
        b[3] = datanum

        # 读操作
        b[4] = 0x11

        # 地址
        b[5] = 0xDA
        b[6] = 0x05

        # 读取寄存器的长度
        b[7] = 0x0C

        # 校验和
        b[8] = self.checknum(b, datanum + 4)

        # 向串口发送数据
        putdata = b''

        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
        self.ser.write(putdata)
        print('发送的数据：')
        for i in range(1, datanum + 6):
            print(hex(putdata[i - 1]))

        getdata = self.ser.read(20)
        print('返回的数据：')
        for i in range(1, 21):
            print(hex(getdata[i - 1]))

        setpower = [0] * 6
        for i in range(1, 7):
            if getdata[i * 2 + 5] == 0xff and getdata[i * 2 + 6] == 0xff:
                setpower[i - 1] = -1
            else:
                setpower[i - 1] = getdata[i * 2 + 5] + (getdata[i * 2 + 6] << 8)
        return setpower

    # 读取驱动器实际的位置值
    def get_actpos(self):
        datanum = 0x04
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90

        # hand_id号
        b[2] = self.hand_id

        # 数据个数
        b[3] = datanum

        # 读操作
        b[4] = 0x11

        # 地址
        b[5] = 0xFE
        b[6] = 0x05

        # 读取寄存器的长度
        b[7] = 0x0C

        # 校验和
        b[8] = self.checknum(b, datanum + 4)

        # 向串口发送数据
        putdata = b''

        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
        self.ser.write(putdata)
        # print('发送的数据：')
        # for i in range(1, datanum + 6):
        #     print(hex(putdata[i - 1]))

        getdata = self.ser.read(20)
        # print('返回的数据：')
        # for i in range(1, 21):
        #     print(hex(getdata[i - 1]))

        actpos = [0] * 6
        for i in range(1, 7):
            if getdata[i * 2 + 5] == 0xff and getdata[i * 2 + 6] == 0xff:
                actpos[i - 1] = -1
            else:
                actpos[i - 1] = getdata[i * 2 + 5] + (getdata[i * 2 + 6] << 8)
        return actpos

    # 读取力度信息
    def get_actforce(self):
        datanum = 0x04
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90
        # hand_id号
        b[2] = self.hand_id
        # 数据个数
        b[3] = datanum
        # 读操作
        b[4] = 0x11
        # 地址
        b[5] = 0x2E
        b[6] = 0x06
        # 读取寄存器的长度
        b[7] = 0x0C
        # 校验和
        b[8] = self.checknum(b, datanum + 4)
        # 向串口发送数据
        putdata = b''
        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
        self.ser.write(putdata)
        # print('发送的数据：')
        # for i in range(1, datanum + 6):
        #     print(hex(putdata[i - 1]))

        getdata = self.ser.read(20)
        # 返回的十六进制值
        # print('返回的数据：')
        # for i in range(1, 21):
        #     print(hex(getdata[i - 1]))

        actforce = [0] * 6
        for i in range(1, 7):
            if getdata[i * 2 + 6] == 0xff:
                actforce[i - 1] = getdata[i * 2 + 5] + (getdata[i * 2 + 6] << 8) - 65536
            else:
                actforce[i - 1] = getdata[i * 2 + 5] + (getdata[i * 2 + 6] << 8)
        print("实际力度值：")
        print(actforce)
        return actforce

    # 读取实际的角度值
    def get_actangle(self):
        datanum = 0x04
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90
        # hand_id号
        b[2] = self.hand_id
        # 数据个数
        b[3] = datanum
        # 读操作
        b[4] = 0x11
        # 地址
        b[5] = 0x0A
        b[6] = 0x06
        # 读取寄存器的长度
        b[7] = 0x0C
        # 校验和
        b[8] = self.checknum(b, datanum + 4)
        # 向串口发送数据
        putdata = b''
        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
        self.ser.write(putdata)
        # print('发送的数据：')
        # for i in range(1, datanum+6):
        #     print(hex(putdata[i-1]))

        getdata = self.ser.read(20)
        # 返回的十六进制值
        # print('返回的数据：')
        # for i in range(1, 21):
        #     print(hex(getdata[i - 1]))

        actangle = [0] * 6
        for i in range(1, 7):
            if getdata[i * 2 + 5] == 0xff and getdata[i * 2 + 6] == 0xff:
                actangle[i - 1] = -1
            else:
                actangle[i - 1] = getdata[i * 2 + 5] + (getdata[i * 2 + 6] << 8)
        # print("实际角度值：")
        # print(actangle)
        return actangle

    # 读取小拇指实际的受力
    def get_little_actforce(self):
        datanum = 0x04
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90

        # hand_id号
        b[2] = self.hand_id

        # 数据个数
        b[3] = datanum

        # 读操作
        b[4] = 0x19

        # 地址
        b[5] = 0x00
        b[6] = 0x00

        # 读取寄存器的长度
        b[7] = 0x20

        # 校验和
        b[8] = self.checknum(b, datanum + 4)

        # 向串口发送数据
        putdata = b''

        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
        self.ser.write(putdata)
        # print('发送的数据：')
        # for i in range(1,datanum+6):
        #     print(hex(putdata[i-1]))

        getdata = self.ser.read(40)
        # print('返回的数据：')
        # for i in range(1,21):
        #     print(hex(getdata[i-1]))

        actforce = [0] * 16
        for i in range(1, 17):
            if getdata[i * 2 + 5] == 0xff and getdata[i * 2 + 6] == 0xff:
                actforce[i - 1] = -1
            else:
                actforce[i - 1] = getdata[i * 2 + 5] + (getdata[i * 2 + 6] << 8)

        # 串口收到的为又两个字节组成的无符号十六进制数，十进制的表示范围为0～65536，而实际数据为有符号的数据，表示力的不同方向，范围为-32768~32767，
        # 因此需要对收到的数据进行处理，得到实际力传感器的数据：当读数大于32767时，次数减去65536即可。
        for i in range(len(actforce)):
            if actforce[i] > 32767:
                actforce[i] = actforce[i]
        print("小拇指的触觉信息：  ", actforce)
        return actforce

    # 读取无名指实际的受力
    def get_ring_actforce(self):
        datanum = 0x04
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90

        # hand_id号
        b[2] = self.hand_id

        # 数据个数
        b[3] = datanum

        # 读操作
        b[4] = 0x19

        # 地址
        b[5] = 0x3A
        b[6] = 0x00

        # 读取寄存器的长度
        b[7] = 0x20

        # 校验和
        b[8] = self.checknum(b, datanum + 4)

        # 向串口发送数据
        putdata = b''

        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
        self.ser.write(putdata)
        # print('发送的数据：')
        # for i in range(1,datanum+6):
        #     print(hex(putdata[i-1]))

        getdata = self.ser.read(40)
        # print('返回的数据：')
        # for i in range(1,21):
        #     print(hex(getdata[i-1]))

        actforce = [0] * 16
        for i in range(1, 17):
            if getdata[i * 2 + 5] == 0xff and getdata[i * 2 + 6] == 0xff:
                actforce[i - 1] = -1
            else:
                actforce[i - 1] = getdata[i * 2 + 5] + (getdata[i * 2 + 6] << 8)

        # 串口收到的为又两个字节组成的无符号十六进制数，十进制的表示范围为0～65536，而实际数据为有符号的数据，表示力的不同方向，范围为-32768~32767，
        # 因此需要对收到的数据进行处理，得到实际力传感器的数据：当读数大于32767时，次数减去65536即可。
        for i in range(len(actforce)):
            if actforce[i] > 32767:
                actforce[i] = actforce[i] - 65536
        print("无名指的触觉信息：  ", actforce)
        return actforce

    # 读取中指实际的受力
    def get_middle_actforce(self):
        datanum = 0x04
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90

        # hand_id号
        b[2] = self.hand_id

        # 数据个数
        b[3] = datanum

        # 读操作
        b[4] = 0x19

        # 地址
        b[5] = 0x74
        b[6] = 0x00

        # 读取寄存器的长度
        b[7] = 0x20

        # 校验和
        b[8] = self.checknum(b, datanum + 4)

        # 向串口发送数据
        putdata = b''

        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
        self.ser.write(putdata)
        # print('发送的数据：')
        # for i in range(1,datanum+6):
        #     print(hex(putdata[i-1]))

        getdata = self.ser.read(40)
        # print('返回的数据：')
        # for i in range(1,21):
        #     print(hex(getdata[i-1]))

        actforce = [0] * 16
        for i in range(1, 17):
            if getdata[i * 2 + 5] == 0xff and getdata[i * 2 + 6] == 0xff:
                actforce[i - 1] = -1
            else:
                actforce[i - 1] = getdata[i * 2 + 5] + (getdata[i * 2 + 6] << 8)

        # 串口收到的为又两个字节组成的无符号十六进制数，十进制的表示范围为0～65536，而实际数据为有符号的数据，表示力的不同方向，范围为-32768~32767，
        # 因此需要对收到的数据进行处理，得到实际力传感器的数据：当读数大于32767时，次数减去65536即可。
        for i in range(len(actforce)):
            if actforce[i] > 32767:
                actforce[i] = actforce[i] - 65536
        print("中指的触觉信息：  ", actforce)
        return actforce

    # 读取食指实际的受力
    def get_index_actforce(self):
        datanum = 0x04
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90

        # hand_id号
        b[2] = self.hand_id

        # 数据个数
        b[3] = datanum

        # 读操作
        b[4] = 0x19

        # 地址
        b[5] = 0xAE
        b[6] = 0x00

        # 读取寄存器的长度
        b[7] = 0x20

        # 校验和
        b[8] = self.checknum(b, datanum + 4)

        # 向串口发送数据
        putdata = b''

        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
        self.ser.write(putdata)
        # print('发送的数据：')
        # for i in range(1,datanum+6):
        #     print(hex(putdata[i-1]))

        getdata = self.ser.read(40)
        # print('返回的数据：')
        # for i in range(1,21):
        #     print(hex(getdata[i-1]))

        actforce = [0] * 16
        for i in range(1, 17):
            if getdata[i * 2 + 5] == 0xff and getdata[i * 2 + 6] == 0xff:
                actforce[i - 1] = -1
            else:
                actforce[i - 1] = getdata[i * 2 + 5] + (getdata[i * 2 + 6] << 8)

        # 串口收到的为又两个字节组成的无符号十六进制数，十进制的表示范围为0～65536，而实际数据为有符号的数据，表示力的不同方向，范围为-32768~32767，
        # 因此需要对收到的数据进行处理，得到实际力传感器的数据：当读数大于32767时，次数减去65536即可。
        for i in range(len(actforce)):
            if actforce[i] > 32767:
                actforce[i] = actforce[i] - 65536
        print("食指的触觉信息：  ", actforce)
        return actforce

    # 读取大拇指实际的受力
    def get_thumb_actforce(self):
        datanum = 0x04
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90

        # hand_id号
        b[2] = self.hand_id

        # 数据个数
        b[3] = datanum

        # 读操作
        b[4] = 0x19

        # 地址
        b[5] = 0xE8
        b[6] = 0x00

        # 读取寄存器的长度
        b[7] = 0x20

        # 校验和
        b[8] = self.checknum(b, datanum + 4)

        # 向串口发送数据
        putdata = b''

        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
        self.ser.write(putdata)
        # print('发送的数据：')
        # for i in range(1,datanum+6):
        #     print(hex(putdata[i-1]))

        getdata = self.ser.read(40)
        # print('返回的数据：')
        # for i in range(1,21):
        #     print(hex(getdata[i-1]))

        actforce = [0] * 16
        for i in range(1, 17):
            if getdata[i * 2 + 5] == 0xff and getdata[i * 2 + 6] == 0xff:
                actforce[i - 1] = -1
            else:
                actforce[i - 1] = getdata[i * 2 + 5] + (getdata[i * 2 + 6] << 8)

        # 串口收到的为又两个字节组成的无符号十六进制数，十进制的表示范围为0～65536，而实际数据为有符号的数据，表示力的不同方向，范围为-32768~32767，
        # 因此需要对收到的数据进行处理，得到实际力传感器的数据：当读数大于32767时，次数减去65536即可。
        for i in range(len(actforce)):
            if actforce[i] > 32767:
                actforce[i] = actforce[i] - 65536
        print("大拇指的触觉信息：  ", actforce)
        return actforce

    # 读取手掌实际的受力
    def get_palm_actforce(self):
        datanum = 0x04
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90

        # hand_id号
        b[2] = self.hand_id

        # 数据个数
        b[3] = datanum

        # 读操作
        b[4] = 0x19

        # 地址
        b[5] = 0x22
        b[6] = 0x01

        # 读取寄存器的长度
        b[7] = 0x7E

        # 校验和
        b[8] = self.checknum(b, datanum + 4)

        # 向串口发送数据
        putdata = b''

        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
        self.ser.write(putdata)
        # print('发送的数据：')
        # for i in range(1,datanum+6):
        #     print(hex(putdata[i-1]))

        getdata = self.ser.read(134)
        # print('返回的数据：')
        # for i in range(1,21):
        #     print(hex(getdata[i-1]))

        actforce = [0] * 63
        for i in range(1, 64):
            if getdata[i * 2 + 5] == 0xff and getdata[i * 2 + 6] == 0xff:
                actforce[i - 1] = -1
            else:
                actforce[i - 1] = getdata[i * 2 + 5] + (getdata[i * 2 + 6] << 8)

        # 串口收到的为又两个字节组成的无符号十六进制数，十进制的表示范围为0～65536，而实际数据为有符号的数据，表示力的不同方向，范围为-32768~32767，
        # 因此需要对收到的数据进行处理，得到实际力传感器的数据：当读数大于32767时，次数减去65536即可。
        # for i in range(len(actforce)):
        #     if actforce[i] > 32767:
        #         actforce[i] = actforce[i] - 65536
        print("手掌的触觉信息：  ", actforce)
        return actforce

    # 读取电流
    def get_current(self):
        datanum = 0x04
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90

        # hand_id号
        b[2] = self.hand_id

        # 数据个数
        b[3] = datanum

        # 读操作
        b[4] = 0x11

        # 地址
        b[5] = 0x3A
        b[6] = 0x06

        # 读取寄存器的长度
        b[7] = 0x0C

        # 校验和
        b[8] = self.checknum(b, datanum + 4)

        # 向串口发送数据
        putdata = b''

        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
        self.ser.write(putdata)
        print('发送的数据：')
        for i in range(1, datanum + 6):
            print(hex(putdata[i - 1]))

        getdata = self.ser.read(20)
        print('返回的数据：')
        for i in range(1, 21):
            print(hex(getdata[i - 1]))

        current = [0] * 6
        for i in range(1, 7):
            if getdata[i * 2 + 5] == 0xff and getdata[i * 2 + 6] == 0xff:
                current[i - 1] = -1
            else:
                current[i - 1] = getdata[i * 2 + 5] + (getdata[i * 2 + 6] << 8)
        return current

    # 读取故障信息
    def get_error(self):
        datanum = 0x04
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90

        # hand_id号
        b[2] = self.hand_id

        # 数据个数
        b[3] = datanum

        # 读操作
        b[4] = 0x11

        # 地址
        b[5] = 0x46
        b[6] = 0x06

        # 读取寄存器的长度
        b[7] = 0x06

        # 校验和
        b[8] = self.checknum(b, datanum + 4)

        # 向串口发送数据
        putdata = b''

        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
        self.ser.write(putdata)
        print('发送的数据：')
        for i in range(1, datanum + 6):
            print(hex(putdata[i - 1]))

        getdata = self.ser.read(14)
        print('返回的数据：')
        for i in range(1, 15):
            print(hex(getdata[i - 1]))

        error = [0] * 6
        for i in range(1, 7):
            error[i - 1] = getdata[i + 6]
        return error

    # 读取状态信息
    def get_status(self):
        datanum = 0x04
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90

        # hand_id号
        b[2] = self.hand_id

        # 数据个数
        b[3] = datanum

        # 读操作
        b[4] = 0x11

        # 地址
        b[5] = 0x4C
        b[6] = 0x06

        # 读取寄存器的长度
        b[7] = 0x06

        # 校验和
        b[8] = self.checknum(b, datanum + 4)

        # 向串口发送数据
        putdata = b''

        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
            self.ser.write(putdata)
        print('发送的数据：')
        for i in range(1, datanum + 6):
            print(hex(putdata[i - 1]))

        getdata = self.ser.read(14)
        print('返回的数据：')
        for i in range(1, 15):
            print(hex(getdata[i - 1]))

        status = [0] * 6
        for i in range(1, 7):
            status[i - 1] = getdata[i + 6]
        return status

    # 读取温度信息
    def get_temp(self):
        datanum = 0x04
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90

        # hand_id号
        b[2] = self.hand_id

        # 数据个数
        b[3] = datanum

        # 读操作
        b[4] = 0x11

        # 地址
        b[5] = 0x52
        b[6] = 0x06

        # 读取寄存器的长度
        b[7] = 0x06

        # 校验和
        b[8] = self.checknum(b, datanum + 4)

        # 向串口发送数据
        putdata = b''

        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
        self.ser.write(putdata)
        print('发送的数据：')
        for i in range(1, datanum + 6):
            print(hex(putdata[i - 1]))

        getdata = self.ser.read(14)
        print('返回的数据：')
        for i in range(1, 15):
            print(hex(getdata[i - 1]))

        temp = [0] * 6
        for i in range(1, 7):
            temp[i - 1] = getdata[i + 6]
        return temp

    # 清除错误
    def set_clear_error(self):
        datanum = 0x04
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90

        # hand_id号
        b[2] = self.hand_id

        # 数据个数
        b[3] = datanum

        # 写操作
        b[4] = 0x12

        # 地址
        b[5] = 0xEC
        b[6] = 0x03

        # 数据
        b[7] = 0x01

        # 校验和
        b[8] = self.checknum(b, datanum + 4)

        # 向串口发送数据
        putdata = b''

        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
        self.ser.write(putdata)
        print('发送的数据：')
        for i in range(1, datanum + 6):
            print(hex(putdata[i - 1]))

        getdata = self.ser.read(9)
        print('返回的数据：')
        for i in range(1, 10):
            print(hex(getdata[i - 1]))

    # 保存参数到FLASH
    def set_save_flash(self):
        datanum = 0x04
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90

        # hand_id号
        b[2] = self.hand_id

        # 数据个数
        b[3] = datanum

        # 写操作
        b[4] = 0x12

        # 地址
        b[5] = 0xED
        b[6] = 0x03

        # 数据
        b[7] = 0x01

        # 校验和
        b[8] = self.checknum(b, datanum + 4)

        # 向串口发送数据
        putdata = b''

        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
        self.ser.write(putdata)
        print('发送的数据：')
        for i in range(1, datanum + 6):
            print(hex(putdata[i - 1]))

        getdata = self.ser.read(18)
        print('返回的数据：')
        for i in range(1, 19):
            print(hex(getdata[i - 1]))

    # 力传感器校准
    def gesture_force_clb(self):
        datanum = 0x04
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90

        # hand_id号
        b[2] = self.hand_id

        # 数据个数
        b[3] = datanum

        # 写操作
        b[4] = 0x12

        # 地址
        b[5] = 0xF1
        b[6] = 0x03

        # 数据
        b[7] = 0x01

        # 校验和
        b[8] = self.checknum(b, datanum + 4)

        # 向串口发送数据
        putdata = b''

        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
        self.ser.write(putdata)
        print('发送的数据：')
        for i in range(1, datanum + 6):
            print(hex(putdata[i - 1]))

        getdata = self.ser.read(18)
        print('返回的数据：')
        for i in range(1, 19):
            print(hex(getdata[i - 1]))

    # 设置上电速度
    def setdefaultspeed(self, speed1, speed2, speed3, speed4, speed5, speed6):
        if speed1 < 0 or speed1 > 1000:
            print('数据超出正确范围：0-1000')
            return
        if speed2 < 0 or speed2 > 1000:
            return
        if speed3 < 0 or speed3 > 1000:
            return
        if speed4 < 0 or speed4 > 1000:
            return
        if speed5 < 0 or speed5 > 1000:
            return
        if speed6 < 0 or speed6 > 1000:
            return

        datanum = 0x0F
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90

        # hand_id号
        b[2] = self.hand_id

        # 数据个数
        b[3] = datanum

        # 写操作
        b[4] = 0x12

        # 地址
        b[5] = 0x08
        b[6] = 0x04

        # 数据
        b[7] = self.data2bytes(speed1)[0]
        b[8] = self.data2bytes(speed1)[1]

        b[9] = self.data2bytes(speed2)[0]
        b[10] = self.data2bytes(speed2)[1]

        b[11] = self.data2bytes(speed3)[0]
        b[12] = self.data2bytes(speed3)[1]

        b[13] = self.data2bytes(speed4)[0]
        b[14] = self.data2bytes(speed4)[1]

        b[15] = self.data2bytes(speed5)[0]
        b[16] = self.data2bytes(speed5)[1]

        b[17] = self.data2bytes(speed6)[0]
        b[18] = self.data2bytes(speed6)[1]

        # 校验和
        b[19] = self.checknum(b, datanum + 4)

        # 向串口发送数据
        putdata = b''

        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
        self.ser.write(putdata)

        print('发送的数据：')
        for i in range(1, datanum + 6):
            print(hex(putdata[i - 1]))

        getdata = self.ser.read(9)
        print('返回的数据：')
        for i in range(1, 10):
            print(hex(getdata[i - 1]))

    # 设置上电力控阈值
    def setdefaultpower(self, power1, power2, power3, power4, power5, power6):
        if power1 < 0 or power1 > 1000:
            print('数据超出正确范围：0-1000')
            return
        if power2 < 0 or power2 > 1000:
            return
        if power3 < 0 or power3 > 1000:
            return
        if power4 < 0 or power4 > 1000:
            return
        if power5 < 0 or power5 > 1000:
            return
        if power6 < 0 or power6 > 1000:
            return

        datanum = 0x0F
        b = [0] * (datanum + 5)
        # 包头
        b[0] = 0xEB
        b[1] = 0x90

        # hand_id号
        b[2] = self.hand_id

        # 数据个数
        b[3] = datanum

        # 写操作
        b[4] = 0x12

        # 地址
        b[5] = 0x14
        b[6] = 0x04

        # 数据
        b[7] = self.data2bytes(power1)[0]
        b[8] = self.data2bytes(power1)[1]

        b[9] = self.data2bytes(power2)[0]
        b[10] = self.data2bytes(power2)[1]

        b[11] = self.data2bytes(power3)[0]
        b[12] = self.data2bytes(power3)[1]

        b[13] = self.data2bytes(power4)[0]
        b[14] = self.data2bytes(power4)[1]

        b[15] = self.data2bytes(power5)[0]
        b[16] = self.data2bytes(power5)[1]

        b[17] = self.data2bytes(power6)[0]
        b[18] = self.data2bytes(power6)[1]

        # 校验和
        b[19] = self.checknum(b, datanum + 4)

        # 向串口发送数据
        putdata = b''

        for i in range(1, datanum + 6):
            putdata = putdata + self.num2str(b[i - 1])
        self.ser.write(putdata)
        print('发送的数据：')
        for i in range(1, datanum + 6):
            print(hex(putdata[i - 1]))

        getdata = self.ser.read(9)
        print('返回的数据：')
        for i in range(1, 10):
            print(hex(getdata[i - 1]))

    def soft_setpos(self, pos1, pos2, pos3, pos4, pos5, pos6):
        value0 = 0
        temp_value = [0, 0, 0, 0, 0, 0]
        is_static = [0, 0, 0, 0, 0, 0]
        static_value = [0, 0, 0, 0, 0, 0]
        pos_value = [pos1, pos2, pos3, pos4, pos5, pos6]
        n = 5
        diffpos = pos1 - self.f1_init_pos
        tic = time.time()
        for ii in range(5):
            #  self.setpos(pos1,pos2,pos3,pos4,pos5,pos6)
            #  print('==========================')
            actforce = self.get_actforce()
            print('actforce: ', actforce)
            for i, f in enumerate(actforce[0:5]):
                if is_static[i]:
                    continue
                if f > 1000:
                    continue
                if i == 5:  # 大拇指
                    if f > 100:  # 如果手指受力大于100，就维持之前的位置
                        is_static[i] = 1  # 标记为静态手指，手指保持该位置不再动
                        static_value[i] = temp_value[i]  # 上一步的第i个手指位置
                else:
                    if f > 50:  # 如果手指受力大于100，就维持之前的位置
                        is_static[i] = 1  # 标记为静态手指，手指保持该位置不再动
                        static_value[i] = temp_value[i]  # 上一步的第i个手指位置
            temp_value = pos_value.copy()
            for i in range(6):
                if is_static[i]:
                    pos_value[i] = static_value[i]
            pos1 = pos_value[0]  # 小拇指伸直0，弯曲2000
            pos2 = pos_value[1]  # 无名指伸直0，弯曲2000
            pos3 = pos_value[2]  # 中指伸直0，弯曲2000
            pos4 = pos_value[3]  # 食指伸直0，弯曲2000
            pos5 = pos_value[4]  # 大拇指伸直0，弯曲2000
            pos6 = pos_value[5]  # 大拇指转向掌心 2000
            self.setpos(pos1, pos2, pos3, pos4, pos5, pos6)
            toc = time.time()
            print('ii: %d,toc=%f' % (ii, toc - tic))

    def reset(self):
        pos1 = self.f1_init_pos  # 小拇指伸直0，弯曲2000
        pos2 = self.f2_init_pos  # 无名指伸直0，弯曲2000
        pos3 = self.f3_init_pos  # 中指伸直0，弯曲2000
        pos4 = self.f4_init_pos  # 食指伸直0，弯曲2000
        pos5 = self.f5_init_pos  # 大拇指伸直0，弯曲2000
        pos6 = self.f6_init_pos  # 大拇指转向掌心 2000
        self.setpos(pos1, pos2, pos3, pos4, pos5, pos6)
        return

    def reset_0(self):
        pos1 = 0  # 小拇指伸直0，弯曲2000
        pos2 = 0  # 无名指伸直0，弯曲2000
        pos3 = 0  # 中指伸直0，弯曲2000
        pos4 = 0  # 食指伸直0，弯曲2000
        pos5 = 0  # 大拇指伸直0，弯曲2000
        pos6 = 0  # 大拇指转向掌心 2000
        self.setpos(pos1, pos2, pos3, pos4, pos5, pos6)
        return

    def hand_close(self):
        self.ser.close()


if __name__ == "__main__":
    hand = InspireHandR()

    hand.reset_0()
    angles, fingerindex, forceindex, componentindex = runs()

    threshold = 100  # 动作力度阈值

    while True:
        # 更新手指角度
        hand.setangle(*angles)

        # 获取当前动作力度
        pow = hand.get_actforce()
        # 检查是否有两个以上动作力度超过阈值
        forces = [pow[0], pow[1], pow[2], pow[3], pow[4], pow[5]]
        if sum(force > threshold for force in forces) >= 2:
            break

        for i in range(len(angles)):
            if i == 4:
                angles[i] = max(0, angles[i] - 25)
            if i != 3 and i != 4 and i != 5:
                angles[i] = max(0, angles[i] - 50)

    time.sleep(5)
    if fingerindex == 'thumb':
        hand.setthumbangle(200)
    if fingerindex == 'index':
        hand.setindexangle(200)
    print(forces)
    # hand.setpower(7, 200, 20, -6, 652, -159)
    # ur5 = Robot()
    # ur5.robot_msg_recv()
