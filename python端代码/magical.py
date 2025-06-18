# -*- coding: utf-8 -*-
import os
import sys
import socket
import threading
import time
import DobotDllType as dType

# 创建socket客户端变量
socket_client1 = ''
# 定义全局变量
i = 0
j = 0
k = 0

# 定义机械臂状态,用键值存储
CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}
# 加载机械臂动态链接库DobotDll.dll
api = dType.load()
# 连接机械臂
state = dType.ConnectDobot(api, "COM9", 115200)[0]
# 打印机械臂连接状态
print("Connect status:",CON_STR[state])
# 设置机械臂PTP运动模式参数(门型运动时抬升高度为100,最大高度为110)
dType.SetPTPJumpParams(api, 100, 110, isQueued=0)
# 设置机械臂末端为夹爪
dType.SetEndEffectorParams(api, 59.7, 0, 0, 1)
# 初始化清空机械臂的指令
dType.SetQueuedCmdClear(api)
# 开始执行队列指令
dType.SetQueuedCmdStartExec(api)
# 机械臂初始位置
dType.SetPTPCmd(api, 0, 142, -240, 65, -60, isQueued=1)
print('Starting...')

# 光电到物料到位检测
def target_reach():
    # 使能光电传感器
    dType.SetInfraredSensor(api, 1, 2, version=0)       #GP1 GP2 GP4 GP5    0 1 2 3
    time_start = time.time()
    while True:
        i = [0]
        # 获取光电传感器的值
        i = dType.GetInfraredSensor(api, 2)  #修改
        # print(i[0])
        time_end = time.time()
        # 检测光电信号判断物料到位，25秒之后取消等待
        if i[0] == 1:
            for count in range(3):
                i = dType.GetInfraredSensor(api, 2)   #修改
                print(i[0])
            if i[0] == 1:
                print('stop')
                break
        elif (time_end - time_start) > 25:
            break
        else:
            pass

# 光电物料离位检测
def target_leave():
    dType.SetInfraredSensor(api, 1, 2, version=0)   #修改
    time_start = time.time()
    while True:
        i = dType.GetInfraredSensor(api,2)   #修改
        time_end = time.time()
        if i[0] == 1 and (time_end - time_start) < 25:
            time.sleep(0.2)
            pass
        else:
            break

# 码垛堆叠多个方块
def pileup():
    global i,j,k
    # 每次上料前检测拍照位是否有物料
    # if dType.GetInfraredSensor(api, 1)[0] == 1:                                
    #     socket_client.send("arrive".encode('utf-8'))

    # 默认释放夹爪
    dType.SetEndEffectorGripper(api, 1, 0, isQueued=1)
    # 抓取(按照行列式)
    # 一层
    dType.SetPTPCmdEx(api, 0, (204+j*46), (-165+i*46), (-12 ), -109, isQueued=1)    #设置抓取初始点
    # 抓取
    dType.SetEndEffectorGripper(api, 1, 1, isQueued=1)
    # 延时1000ms
    dType.dSleep(1000)
    # 放置                   
    dType.SetPTPCmdEx(api, 0, -5, -290, 60, -110, isQueued=1)
    # 释放
    dType.SetEndEffectorGripper(api, 1, 0, isQueued=1)
    # 抬起              
    dType.SetPTPCmdEx(api, 0, -22, -275, 90,-110,isQueued=1)
    # 夹爪未使能
    dType.SetEndEffectorGripper(api, 0, 0, isQueued=1)  

    # 传送带动作 7500pulse/s
    dType.SetEMotorEx(api, 0, 1, 10000, isQueued=1)
    # 光电物料到位阻塞检测，限时25s
    target_reach()
    # 传送带停止，触发相机拍照定位                                                           
    dType.SetEMotorEx(api, 0, 0, 0, isQueued=1)
    socket_client.send("arrive".encode('utf-8'))
    # 光电物料离位阻塞检测，限时25s                                      
    target_leave()   

    # 2*4矩阵(一层)
    if j < 1 and i <= 3:
        j += 1
    elif j >= 1 and i < 3:
        j = 0
        i += 1
    # elif j >= 1 and i == 3 and k<1 :
    #     j = 0
    #     i = 0
    #     k += 1

    else:
        print('机械臂结束上料')
        sys.exit()

# 初始化TCP客户端，连接视觉服务端，等待信号
def TCPClient_Vision():
    global socket_client
    socket_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        ip_Vision = "127.0.0.1"
        port = 4001
        socket_client.connect((ip_Vision, port))
        print('Successful to initialize tcpclient_Vision.')
    except Exception as e:
        print(e)
    while True:
        try:
            data = socket_client.recv(1024).decode('utf-8')
            print(data)
            checkdata = data[:5]
            # 当接收到run指令开始上料
            if checkdata == 'run':
                print('机械臂上料')
                pileup()
                checkdata = 'null'
        except UnicodeDecodeError:
            print('error')
        time.sleep(0.1)

# 主程序入口		
if __name__ == "__main__":
    # 新增子线程，创建TCP客户端，与视觉软件通信
    client_vision = threading.Thread(target=TCPClient_Vision)
    # 设置子线程为守护线程，防止退出主线程时，子线程仍在运行
    client_vision.setDaemon(True)
    # 启动子线程
    client_vision.start()
    time.sleep(0.5)
    print('机械臂开始上料')
    pileup()   
    while True:
        pass