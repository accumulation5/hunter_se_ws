#!/usr/bin/env python
# -*- coding:UTF-8 -*-
from __future__ import print_function
import socket

# 定义一个函数来解析数据包
def parse_data(data):
    speed = 60/((data[3] * 256 + data[4])*0.000024)  # 计算转速
    start_angle = (data[5] * 256 + data[6]) / 100.0  # 计算起始角度
    distances_and_intensities = []  # 初始化距离和光强列表

    # 遍历数据包中的距离和光强数据
    for x in range(7, 98, 3):
        distance = data[x] * 256 + data[x + 1]  # 计算距离
        intensity = data[x + 2]  # 获取光强
        #打印某个角度的距离
        # if start_angle >80 and start_angle <100:
        #     print(distance)
        distances_and_intensities.append((distance, intensity))  # 添加到列表

    end_angle = (data[105] * 256 + data[106])/100.0  # 计算结束角度

    return speed, start_angle, distances_and_intensities, end_angle

# 定义一个函数来打印数据包的内容
def print_data(speed, start_angle, distances_and_intensities, end_angle, last_angle):
    if last_angle - start_angle > 100:
        print("*******************************")

    # 打印转速、起始角度和数据点
    print("转速（圈/每分钟）：", speed, end="\t")
    print("起始角度:", start_angle, end="\t")
    print("数据【距离（mm）|光强】*32个点：", end="\t")

    # 打印每个数据点的距离和光强
    for distance, intensity in distances_and_intensities:
        print(distance, "|", intensity, end="\t")

    print("结束角度:", end_angle, end="\n\n")


# 主程序开始
if __name__ == '__main__':
    last_angle = 0  # 初始化上一个角度
    # 创建 socket 对象
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # 目标雷达的 IP 和端口
    radar_ip = "192.168.1.200"
    radar_port = 2369

    # 本地电脑的 IP 和端口
    local_ip = "192.168.1.102"
    local_port = 2368

    try:
        # 绑定本地地址
        client_socket.bind((local_ip, local_port))
        # 连接到雷达
        # client_socket.connect((radar_ip, radar_port))
        print("已连接到雷达:", radar_ip, ":", radar_port)
        # 循环读取数据
        while True:
            try:
                data , addr = client_socket.recvfrom(108)  # 读取1个字节的数据
                if data[0] == 0xA5:
                    # 检查数据包的头部
                    if data[1] == 0x5A:
                        if data[2] == 0x6C:
                            # 解析和打印数据包
                            speed, start_angle, distances_and_intensities, end_angle = parse_data(data)
                            print_data(speed, start_angle, distances_and_intensities, end_angle, last_angle)
                            last_angle = start_angle  # 更新上一个角度
            except Exception as e:
                pass  # 忽略异常
    except Exception as e:
        pass  # 忽略异常