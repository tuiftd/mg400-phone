import socket
import threading
import math
import numpy as np
import cv2
running = True
client_threads = []  # 保存所有客户端线程
client_sockets = []  # 用于保存所有客户端socket
client_sockets_lock = threading.Lock()  # 客户端socket列表的锁
client_dict = {}  # 用于保存客户端名字到socket和address的映射
point2rob2use = []  # 用于保存点坐标到机器人坐标系的变换矩阵q
num2rob2use = 0  # 用于保存拾取计数
M_Phone = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])  # 初始化手机坐标系，3x3齐次矩阵
M_cam2rob = np.array([
    [ 9.95277167e-01 ,7.66627776e-02 , 2.94389007e+02],
 [-5.70614531e-02,  9.99529958e-01  ,4.31997837e+00],
    [ 0.00000000e+00  ,0.00000000e+00  ,1.00000000e+00]
])  # 初始化相机坐标系到机器人坐标系的变换矩阵，3x3齐次矩阵
# 模板手机各点坐标
# 创建一个FileStorage对象用于读取
fs = cv2.FileStorage("camera_params.xml", cv2.FILE_STORAGE_READ)
# 读取内参矩阵
K = fs.getNode("CameraMatrix").mat()

# 读取畸变系数
dist = fs.getNode("DistCoeffs").mat()

# 读取第一个外参矩阵（旋转向量）
R = fs.getNode("Rotation").mat()

# 读取第一个外参矩阵（位移向量）
t = fs.getNode("TranslationVector").mat()

# 释放FileStorage对象
fs.release()

def pixel_to_world(pixel):
    global K,dist,R,t
    """ 将像素坐标转换为世界坐标（假设 Z = 0） """
    u, v = pixel
    K_inv = np.linalg.inv(K)  # 计算 K 的逆

    # 计算归一化相机坐标
    normalized_pixel = np.dot(K_inv, np.array([[u], [v], [1]], dtype=np.float32))  # (3,1)

    # 计算旋转矩阵 R 的前两列
    R_inv = np.linalg.inv(R)
    r1 = R[:, 0].reshape(3, 1)
    r2 = R[:, 1].reshape(3, 1)

    # 计算世界坐标
    H = np.hstack([r1, r2, t])  # 构造 3×3 变换矩阵 H
    H_inv = np.linalg.inv(H)  # 计算 H 的逆

    world_point = np.dot(H_inv, normalized_pixel)  # 变换到世界坐标
    X, Y = world_point[:2].flatten() / world_point[2, 0]  # 归一化

    return np.array([X, Y])  # 返回世界坐标

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
    circle_r = float(circle_segment[3])

    # 创建点对字典
    point_pairs = {}

    # 遍历名称和坐标，生成点对
    for i in range(len(names)):
        if i < len(x_coords) and i < len(y_coords):
            point_pairs[names[i]] = (x_coords[i], y_coords[i])

    # 添加圆心
    point_pairs[circle_name] = (circle_x, circle_y)
    point_pairs['org_r'] = circle_r

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
    global point2rob2use, num2rob2use, M_Phone, point_pairs, source_point, M_cam2rob, client_dict, running
    print(f"客户端 {client_address} 已连接")
    try:
        # 接收客户端名字
        client_name = client_socket.recv(1024).decode('utf-8').strip()
        print(f"客户端名字: {client_name}")
        # 将新socket和名字加入全局字典
        with client_sockets_lock:
            client_dict[client_name] = (client_socket, client_address)
        while True:
            if not running:  # 检查是否需要停止运行
                break
            data = client_socket.recv(1024)
            if not data:
                break
            message = data.decode('utf-8')
            print(f"来自 {client_name} 的消息: {message}")
            if message.startswith('1'):
                # 视觉端端入的数据，解析数据
                _, mubiao, x_pos, y_pos, point_pair, mubiao_angle, phone_angle = parse_data(message)
                org_r = point_pairs['org_r']
                x = point_pair[0]
                y = point_pair[1]
                point_yuan = [x, y]

                # 计算相对角度
                phone_angle = -(phone_angle - org_r)
                source_point = pixel_to_world(source_point)
                point_yuan = pixel_to_world(point_yuan)
                source_point = transform_point(M_cam2rob, source_point)
                point_yuan = transform_point(M_cam2rob, point_yuan)
                M_Phone = phone_transform_matrix(source_point, point_yuan, phone_angle)  # 获取手机的相对变换矩阵

                # 获取按mubiao顺序逐个转换移动后的手机坐标
                for i in range(len(mubiao)):
                    mubiao_pos = np.array([x_pos[i], y_pos[i]], dtype=float)
                    mubiao_pos = pixel_to_world(mubiao_pos)
                    point_name = mubiao[i]
                    point = point_pairs.get(point_name, None)
                    if point is None:
                        continue
                    point = pixel_to_world(point)
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
                    target_client_name = "rob"
                    if target_client_name in client_dict:
                        target_socket, _ = client_dict[target_client_name]
                        target_socket.send((response + '\n').encode('utf-8'))
                        print(f"广播坐标信息{point2rob2use[num2rob2use]}")
                        print(f"当前广播次数{num2rob2use + 1}")
                        num2rob2use += 1
                    else:
                        print(f"客户端 {target_client_name} 不存在")
                else:
                    print("坐标信息已全部广播完毕")
                    point2rob2use = []
                    response = f"over"
                    client_socket.send(response.encode('utf-8'))
            elif message.startswith('sendto'):
                parts = message.split(',')
                target_client_name = parts[1].strip()
                message_to_send = ','.join(parts[2:]).strip()
                if target_client_name in client_dict:
                    target_socket, _ = client_dict[target_client_name]
                    target_socket.send((message_to_send + '\n').encode('utf-8'))
                else:
                    print(f"客户端 {target_client_name} 不存在")
    except Exception as e:
        print(f"与 {client_address} 通信时发生错误: {e}")
    finally:
        client_socket.close()
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
        global running
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
                        except Exception as e:
                            print(f"关闭socket时发生错误: {e}")
                    # 清空列表和字典
                    client_sockets.clear()
                    client_dict.clear()

                # 关闭服务端socket
                try:
                    server_sock.close()
                except Exception as e:
                    print(f"关闭服务端socket时发生错误: {e}")

                break  # 退出输入循环

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
    tixing,banyuan,liubianxing,wubianxing,yuanao;1015.258,1291.321,1271.461,1574.149,1005.777;823.065,833.598,1168.360,1173.516,1177.063;yuanxin,1334.604,1006.383,0.651;
    """
    point_pairs = parse_data_to_point_pairs(data)
    print("点对储存结果如下：")
    for name, point in point_pairs.items():
        print(f"{name}: {point}")
    source_point = point_pairs['yuanxin']
    # ——————————————————————————————————————————————————————————————————————————
    main()
    print("程序结束")
