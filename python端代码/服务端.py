import socket
import threading
import math
import numpy as np

client_threads = []  # 保存所有客户端线程
client_sockets = []  # 用于保存所有客户端socket
client_sockets_lock = threading.Lock()  # 客户端socket列表的锁
client_dict = {}  # 用于保存客户端名字到socket和address的映射
point2rob2use = []  # 用于保存点坐标到机器人坐标系的变换矩阵q
num2rob2use = 0  # 用于保存拾取计数
M_Phone = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])  # 初始化手机坐标系，3x3齐次矩阵
M_cam2rob = np.array([
    [-5.90047814e-02, 9.97308092e-01, 2.70781838e+02],
    [-9.86140504e-01, -5.33539085e-02, 8.71809253e+01],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
])  # 初始化相机坐标系到机器人坐标系的变换矩阵，3x3齐次矩阵
# 模板手机各点坐标

def parse_data_to_point_pairs(data):
    """解析数据，返回点对字典"""
    # 拆分数据段
    segments = data.strip().split(';')

    # 提取名称和坐标
    names = segments[0].split(',')
    x_coords = list(map(float, segments[1].split(',')))
    y_coords = list(map(float, segments[2].split(',')))

    # 处理圆心部分
    circle_segment = segments[3].split(',')
    circle_name = circle_segment[0]
    circle_x = float(circle_segment[1])
    circle_y = float(circle_segment[2])

    # 创建点对字典
    point_pairs = {}

    # 遍历名称和坐标，生成点对
    for i in range(len(names)):
        if i < len(x_coords) and i < len(y_coords):
            point_pairs[names[i]] = (x_coords[i], y_coords[i])

    # 添加圆心
    point_pairs[circle_name] = (circle_x, circle_y)

    return point_pairs


def parse_data(input_string):
    """解析输入数据,格式为1,mubiao;x_pos*;y_pos*;point_pair
    第一个表示这是视觉端传入的数据，第二个表示目标模板的顺序，顺序所有x，顺序所有y，当前识别到的圆心点坐标，角度
    """
    # 以分号分割，去除多余空白字符
    result = [item.strip() for item in input_string.split(';') if item.strip()]

    # 检查数据长度，至少要有5个字段
    if len(result) < 5:
        raise ValueError("输入数据格式错误，至少需要5个字段。")

    # 解析第二个元素，按逗号分割为 mubiao 数组
    mubiao = [item.strip() for item in result[1].split(',')]

    # 解析第三个元素，按逗号分割为 x_pos 数组
    x_pos = [float(item.strip()) for item in result[2].split(',') if item.strip()]
    x_pos_array = np.array(x_pos, dtype=float)

    # 解析第四个元素，按逗号分割为 y_pos 数组
    y_pos = [float(item.strip()) for item in result[3].split(',') if item.strip()]
    y_pos_array = np.array(y_pos, dtype=float)

    # 解析第五个元素，按逗号分割为角度
    angle_mubiao = [float(item.strip()) for item in result[4].split(',') if item.strip()]
    angle_mubiao_array = np.array(angle_mubiao, dtype=float)

    # 解析第六个元素，按逗号分割为点对和角度
    point_pair_and_angle = result[5].split(',')
    if len(point_pair_and_angle) < 3:
        raise ValueError("第六个元素格式错误，至少需要三个子元素。")
    point_pair = np.array([float(item.strip()) for item in point_pair_and_angle[:2]], dtype=float)
    angle = float(point_pair_and_angle[2].strip())

    return result, mubiao, x_pos_array, y_pos_array, point_pair, angle_mubiao_array, angle


def affine_to_homogeneous(affine_matrix):
    """将2x3仿射矩阵转换为3x3齐次矩阵"""
    H = np.eye(3)  # 生成单位矩阵
    H[:2, :] = affine_matrix  # 将仿射矩阵填入前两行
    return H


def transform_point(H, point):
    """使用齐次矩阵对二维点进行变换"""
    x, y = point
    homogeneous_point = np.array([x, y, 1])
    transformed_point = np.dot(H, homogeneous_point)  # 矩阵乘法
    return transformed_point[:2]  # 返回二维坐标


def phone_transform_matrix(source_point, target_point, angle):
    """计算 2D 仿射变换矩阵，
    返回3x3矩阵
    # 示例输入
    source_point = (100, 50)   # 源特征点
    target_point = (150, 120)  # 目标特征点
    angle = 30                 # 旋转角度（度）
    """
    # 解包特征点坐标
    x1, y1 = source_point
    x2, y2 = target_point
    # 将角度转换为弧度
    theta = math.radians(angle)

    # 计算旋转矩阵参数
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)

    # 计算旋转后的源点
    x1_rotated = cos_theta * x1 - sin_theta * y1
    y1_rotated = sin_theta * x1 + cos_theta * y1

    # 计算平移量
    tx = x2 - x1_rotated
    ty = y2 - y1_rotated

    # 构建 2D 仿射变换矩阵
    transform_matrix = np.array([
        [cos_theta, -sin_theta, tx],
        [sin_theta, cos_theta, ty]
    ])
    transform_matrix = affine_to_homogeneous(transform_matrix)
    return transform_matrix


def handle_client(client_socket, client_address):
    global point2rob2use, num2rob2use, M_Phone, point_pairs, source_point, M_cam2rob, client_dict
    """处理单个客户端的通信"""
    print(f"客户端 {client_address} 已连接")
    try:
        # 接收客户端名字
        client_name = client_socket.recv(1024).decode('utf-8').strip()
        print(f"客户端名字: {client_name}")
        # 将新socket和名字加入全局字典
        with client_sockets_lock:
            client_dict[client_name] = (client_socket, client_address)
        while True:
            data = client_socket.recv(1024)
            if not data:
                break
            message = data.decode('utf-8')
            print(f"来自 {client_name} 的消息: {message}")
            if message.startswith('1'):
                # 视觉端传入的数据，解析数据
                mubiao = []
                x_pos = []
                y_pos = []
                point_pair = []
                mubiao_angle = []

                _, mubiao, x_pos, y_pos, point_pair, mubiao_angle, phone_angle = parse_data(message)
                mubiao = np.array(mubiao)
                x_pos = np.array(x_pos, dtype=float)
                y_pos = np.array(y_pos, dtype=float)
                point_pair = np.array(point_pair, dtype=float)
                mubiao_angle = np.array(mubiao_angle, dtype=float)
                x = point_pair[0]
                y = point_pair[1]
                point_yuan = (x, y)
                source_point = transform_point(M_cam2rob, source_point)
                point_yuan = transform_point(M_cam2rob, point_yuan)
                M_Phone = phone_transform_matrix(source_point, point_yuan, phone_angle)  # 获取手机的相对变换矩阵
                # print(M_Phone)
                # print(M_cam2rob)
                # 获取按mubiao顺序逐个转换移动后的手机坐标
                for i in range(len(mubiao)):
                    mubiao_pos = np.array([x_pos[i], y_pos[i]], dtype=float)
                    point_name = mubiao[i]
                    point = point_pairs[point_name]
                    point = transform_point(M_cam2rob, point)
                    point = transform_point(M_Phone, point)
                    mubiao_rob = transform_point(M_cam2rob, mubiao_pos)
                    x_rob = mubiao_rob[0]
                    y_rob = mubiao_rob[1]
                    x = point[0]
                    y = point[1]
                    point2rob2use.append(('ok', x_rob, y_rob, -mubiao_angle[i], x, y, phone_angle))
                num2rob2use = 0
            elif message.startswith('go'):
                # 机器人端发送go命令，坐标信息
                if num2rob2use < len(point2rob2use):
                    response = f"ok, {point2rob2use[num2rob2use][1]}, {point2rob2use[num2rob2use][2]}, {point2rob2use[num2rob2use][3]}, {point2rob2use[num2rob2use][4]}, {point2rob2use[num2rob2use][5]}, {point2rob2use[num2rob2use][6]}"
                    client_socket.send(response.encode('utf-8'))
                    print(f"广播坐标信息{point2rob2use[num2rob2use]}")
                    print(f"当前广播次数{num2rob2use + 1}")
                    num2rob2use += 1
                else:
                    print("坐标信息已全部广播完毕")
                    point2rob2use = []
                    response = f"over"
                    client_socket.send(response.encode('utf-8'))
            elif message.startswith('qianduan'):
                if num2rob2use < len(point2rob2use):
                    response = f"ok, {point2rob2use[num2rob2use][1]}, {point2rob2use[num2rob2use][2]}, {point2rob2use[num2rob2use][3]}, {point2rob2use[num2rob2use][4]}, {point2rob2use[num2rob2use][5]}, {point2rob2use[num2rob2use][6]}"
                    broadcast_message(client_socket, response)
                    print(f"广播坐标信息{point2rob2use[num2rob2use]}")
                    print(f"当前广播次数{num2rob2use + 1}")
                    num2rob2use += 1
                else:
                    print("坐标信息已全部广播完毕")
                    point2rob2use = []
                    response = f"over"
                    client_socket.send(response.encode('utf-8'))
            # 新增逻辑，根据客户端名字发送消息
            elif message.startswith('sendto'):
                parts = message.split(',')
                target_client_name = parts[1].strip()
                message_to_send = ','.join(parts[2:]).strip()
                if target_client_name in client_dict:
                    target_socket, _ = client_dict[target_client_name]
                    target_socket.send((message_to_send + '\n').encode('utf-8'))
                else:
                    print(f"客户端 {target_client_name} 不存在")
            # else:
            #     response = f"message received: {message}"
            #     client_socket.send(response.encode('utf-8'))
    except Exception as e:
        print(f"与 {client_address} 通信时发生错误: {e}")
    finally:
        client_socket.close()
        # 从全局列表中移除
        with client_sockets_lock:
            if client_socket in client_sockets:
                client_sockets.remove(client_socket)
                # 同时从字典中移除
                for key, value in client_dict.items():
                    if value[0] == client_socket:
                        del client_dict[key]
                        break
        print(f"客户端 {client_address} 已断开连接")
        client_threads.remove(threading.current_thread())


def broadcast_message(exclude_socket, message):
    """向所有客户端（不包括exclude_socket）广播消息"""
    with client_sockets_lock:
        for sock in client_sockets:
            if sock != exclude_socket:
                try:
                    sock.send(message.encode('utf-8'))
                except Exception as e:
                    print(f"向 {sock.getpeername()} 发送消息时发生错误: {e}")
                    # 如果发送失败，可能是因为客户端已断开连接，移除该socket
                    client_sockets.remove(sock)
                    # 同时从字典中移除
                    for key, value in client_dict.items():
                        if value[0] == sock:
                            del client_dict[key]
                            break


def main():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = ('0.0.0.0', 4001)  # 改为监听所有接口
    server_socket.bind(server_address)
    server_socket.listen(5)
    print("服务端已启动，等待客户端连接...")

    running = True

    def handle_input(server_sock):  # 接收server_socket参数
        nonlocal running
        while running:
            user_input = input()
            if user_input == 'q':
                print("服务端正在关闭...")
                running = False
                # 关闭所有客户端socket
                with client_sockets_lock:
                    for sock in client_sockets:
                        try:
                            sock.close()
                        except:
                            pass
                # 关闭服务端socket
                try:
                    server_sock.close()
                except:
                    pass

    # 将server_socket传递给输入线程
    input_thread = threading.Thread(target=handle_input, args=(server_socket,))
    input_thread.start()

    try:
        while running:
            try:
                client_socket, client_address = server_socket.accept()
                client_thread = threading.Thread(
                    target=handle_client,
                    args=(client_socket, client_address)
                )
                client_thread.start()
                client_threads.append(client_thread)
            except OSError:
                # 当server_socket被主动关闭时捕获异常
                break  # 退出主循环
    finally:
        server_socket.close()
        print("服务端已关闭")
        # 等待所有客户端线程完成
        for thread in client_threads:
            thread.join()
        input_thread.join()


if __name__ == "__main__":
    # 初始化
    data = """
    yuanao,banyuan,liubianxing,tixing,fangao,wubianxing;6.430,32.246,29.818,8.152,57.310,55.811;43.797,14.644,44.032,13.259,17.028,45.356;yuanxin,35.460,30.009;
    """
    point_pairs = parse_data_to_point_pairs(data)
    print("点对储存结果如下：")
    for name, point in point_pairs.items():
        print(f"{name}: {point}")
    source_point = point_pairs['yuanxin']
    # ——————————————————————————————————————————————————————————————————————————
    main()
