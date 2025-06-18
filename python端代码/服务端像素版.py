import socket
import threading
from fenge import parse_data_new
import numpy as np
import json
client_threads = []  # 保存所有客户端线程
client_sockets = []  # 用于保存所有客户端socket
client_sockets_lock = threading.Lock()  # 客户端socket列表的锁
client_dict = {}  # 用于保存客户端名字到socket和address的映射
target_all = []  # 用于保存所有目标数据
num_targets = 0  # 用于保存目标数量
running = False
def format_target_for_send(target_data):
    status, x_rob, y_rob, r_rob, x_phone, y_phone, r_phone = target_data
    return f"{status},{x_rob:.3f},{y_rob:.3f},{r_rob:.3f},{x_phone:.3f},{y_phone:.3f},{r_phone:.3f}"
def load_camera_params():
    """从JSON文件加载相机参数"""
    try:
        with open("camera_params.json", 'r') as f:
            data = json.load(f)
        
        K = np.array(data["CameraMatrix"])
        dist = np.array(data["DistCoeffs"])
        R = np.array(data["Rotation"])
        t = np.array(data["TranslationVector"])
        
        print("成功从camera_params.json加载相机参数")
        return K, dist, R, t
    except FileNotFoundError:
        print("错误：找不到camera_params.json文件")
        return None, None, None, None
    except Exception as e:
        print(f"加载相机参数时出错：{e}")
        return None, None, None, None

K, dist, R, t = load_camera_params()

def load_transform_matrix():
    """从JSON文件加载相机到机械臂的变换矩阵"""
    try:
        with open("cam2robot_transform.json", 'r') as f:
            data = json.load(f)
        
        # 检查是否是2x3仿射矩阵
        if "AffineMatrix" in data:
            affine_matrix = np.array(data["AffineMatrix"])
            # 转换为3x3齐次矩阵
            M_cam2rob = np.vstack([affine_matrix, [0, 0, 1]])
        # 检查是否是3x3齐次矩阵
        elif "HomogeneousMatrix" in data:
            M_cam2rob = np.array(data["HomogeneousMatrix"])
        # 检查是否直接是TransformMatrix
        elif "TransformMatrix" in data:
            M_cam2rob = np.array(data["TransformMatrix"])
        else:
            print("错误：JSON文件中未找到有效的变换矩阵格式")
            return None
        
        print("成功从cam2robot_transform.json加载变换矩阵")
        print(f"变换矩阵:\n{M_cam2rob}")
        return M_cam2rob
        
    except FileNotFoundError:
        print("错误：找不到cam2robot_transform.json文件，使用默认矩阵")
        # 返回默认变换矩阵
        default_matrix = np.array([
            [9.99277962e-01, 3.44535836e-02, 2.91991446e+02],
            [-4.00650989e-02, 1.00289632e+00, -3.02108143e+01],
            [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
        ])
        print(f"使用默认变换矩阵:\n{default_matrix}")
        return default_matrix
    except Exception as e:
        print(f"加载变换矩阵时出错：{e}")
        return None
M_cam2rob = load_transform_matrix()

def process_target_phone_coordinates(target_dict, phone_dict, M_cam2rob):
    """
    处理目标和手机坐标字典，进行坐标变换
    
    Args:
        target_dict (dict): 目标字典，格式为 {name: (x, y, R)}
        phone_dict (dict): 手机字典，格式为 {name: (x, y, R)}
        M_cam2rob (np.array): 相机到机械臂的变换矩阵
    
    Returns:
        list: 包含变换结果的列表，每个元素为 ("ok", x_rob, y_rob, r_rob, x_phone, y_phone, r_phone)
    """
    result_list = []
    
    # 遍历target字典的所有键
    for key_name in target_dict.keys():
        # 检查phone字典中是否有相同的键
        if key_name in phone_dict:
            # 获取target和phone的坐标数据
            target_x, target_y, target_r = target_dict[key_name]
            phone_x, phone_y, phone_r = phone_dict[key_name]
            
            # 将target的像素坐标转换为世界坐标
            target_pixel = (target_x, target_y)
            target_world = pixel_to_world(target_pixel)
            
            # 将世界坐标转换为机械臂坐标
            target_robot = transform_point(M_cam2rob, target_world)
            x_rob, y_rob = target_robot
            phone_pixel = (phone_x, phone_y)
            phone_world = pixel_to_world(phone_pixel)
            phone_robot = transform_point(M_cam2rob, phone_world)
            phone_x, phone_y = phone_robot
            # 角度转换（根据需要调整符号）
            r_rob = target_r  # 根据原代码中的负号
            
            # 组装结果元组
            result_tuple = ("ok", x_rob, y_rob, r_rob, phone_x, phone_y, phone_r)
            result_list.append(result_tuple)
            
            print(f"处理 {key_name}: 目标({target_x}, {target_y}, {target_r}) -> 机械臂({x_rob:.3f}, {y_rob:.3f}, {r_rob:.3f}), 手机({phone_x}, {phone_y}, {phone_r})")
        else:
            print(f"警告: 在phone字典中未找到键 '{key_name}'，发送NG")
            result_tuple = ("ng", x_rob, y_rob, r_rob,0,0,0)
            result_list.append(result_tuple)
    
    return result_list

def send_to_client(target_client_name, message):
    """
    向指定客户端发送消息
    
    Args:
        target_client_name (str): 目标客户端名字
        message (str): 要发送的消息
    
    Returns:
        bool: 发送成功返回True，失败返回False
    """
    with client_sockets_lock:
        if target_client_name in client_dict:
            target_socket, target_address = client_dict[target_client_name]
            try:
                target_socket.send(message.encode('utf-8'))
                print(f"已向客户端 '{target_client_name}' ({target_address}) 发送消息: {message}")
                return True
            except Exception as e:
                print(f"向客户端 '{target_client_name}' 发送消息失败: {e}")
                # 发送失败可能是因为客户端已断开，清理该客户端
                cleanup_disconnected_client(target_client_name)
                return False
        else:
            print(f"客户端 '{target_client_name}' 不存在")
            return False
def cleanup_disconnected_client(client_name):
    """
    清理断开连接的客户端
    
    Args:
        client_name (str): 要清理的客户端名字
    """
    with client_sockets_lock:
        if client_name in client_dict:
            socket_obj, address = client_dict[client_name]
            try:
                socket_obj.close()
            except:
                pass
            del client_dict[client_name]
            
            # 从socket列表中移除
            if socket_obj in client_sockets:
                client_sockets.remove(socket_obj)
            
            print(f"已清理断开的客户端: {client_name} ({address})")
def send_to_client_safe(target_client_name, message, add_newline=True):
    """
    向指定客户端安全发送消息（带换行符）
    
    Args:
        target_client_name (str): 目标客户端名字
        message (str): 要发送的消息
        add_newline (bool): 是否添加换行符
    
    Returns:
        bool: 发送成功返回True，失败返回False
    """
    if add_newline and not message.endswith('\n'):
        message += '\n'
    
    return send_to_client(target_client_name, message)
def pixel_to_world(pixel):
    global K, dist, R, t
    u, v = pixel
    K_inv = np.linalg.inv(K)  # 计算相机内参逆矩阵
    
    # 计算归一化相机坐标
    normalized_pixel = np.dot(K_inv, np.array([[u], [v], [1]], dtype=np.float32))
    
    # 使用外参矩阵R和t，假设Z=0平面
    r1 = R[:, 0].reshape(3, 1)  # 旋转矩阵第一列
    r2 = R[:, 1].reshape(3, 1)  # 旋转矩阵第二列
    H = np.hstack([r1, r2, t])  # 构造单应性矩阵
    H_inv = np.linalg.inv(H)
    
    # 计算世界坐标
    world_point = np.dot(H_inv, normalized_pixel)
    X, Y = world_point[:2].flatten() / world_point[2, 0]  # 归一化
    
    return X, Y

def transform_point(H, point):
    """使用齐次变换矩阵变换点"""
    x, y = point
    # 转换为齐次坐标
    homogeneous_point = np.array([x, y, 1])
    # 应用变换矩阵
    transformed = np.dot(H, homogeneous_point)
    # 返回变换后的x, y坐标
    return transformed[0], transformed[1]
def pixel_to_robot(pixel, M_cam2rob):
    """完整的像素到机械臂坐标变换"""
    # 第一步：像素 → 世界坐标
    world_x, world_y = pixel_to_world(pixel)
    print(f"世界坐标: ({world_x:.3f}, {world_y:.3f})")
    
    # 第二步：世界坐标 → 机械臂坐标
    robot_x, robot_y = transform_point(M_cam2rob, (world_x, world_y))
    print(f"机械臂坐标: ({robot_x:.3f}, {robot_y:.3f})")
    
    return robot_x, robot_y
def handle_client(client_socket, client_address):
    global client_dict, target_all ,M_cam2rob, K, dist, R, t, num_targets
    print(f"客户端 {client_address} 已连接")
    # 设置接收名字的超时时间（秒）
    NAME_TIMEOUT = 3
    
    try:
        # 设置socket超时
        client_socket.settimeout(NAME_TIMEOUT)
        
        try:
            client_name = client_socket.recv(1024).decode('utf-8').strip()
            print(f"客户端名字: {client_name}")
        except socket.timeout:
            # 超时处理
            print(f"客户端 {client_address} 在 {NAME_TIMEOUT} 秒内未发送名字，发送提醒")
            try:
                client_socket.send("name".encode('utf-8'))
                # 再给一次机会，延长超时时间
                client_socket.settimeout(NAME_TIMEOUT)
                client_name = client_socket.recv(1024).decode('utf-8').strip()
                print(f"客户端名字: {client_name}")
            except socket.timeout:
                print(f"客户端 {client_address} 第二次超时，断开连接")
                return
            except Exception as e:
                print(f"发送名字提醒失败: {e}")
                return
        
        # 恢复为非阻塞或较长超时，用于正常通信
        client_socket.settimeout(None)  # 或者设置为更长的超时时间
        # print(f"客户端名字: {client_name}")
        with client_sockets_lock:
            client_dict[client_name] = (client_socket, client_address)

        while running:
            data = client_socket.recv(1024)
            if not data:
                break
            message = data.decode('utf-8')
            print(f"来自 {client_name} 的消息: {message}")
            if client_name == 'vision':
                phone_dict,target_dict = parse_data_new(message)
                if phone_dict and target_dict:
                    target_all=process_target_phone_coordinates(target_dict, phone_dict, M_cam2rob)
                    print(f"处理后的目标数据: {target_all}")
                    # num_targets = len(target_all)
            
            if message.startswith('first'):
                if num_targets < len(target_all):
                    formatted_data = format_target_for_send(target_all[num_targets])
                    send_to_client_safe('qianduan', formatted_data)
                    num_targets += 1
                else:
                    try:
                        send_to_client_safe('qianduan', 'over')
                        # send_to_client_safe('rob', 'over')
                    except Exception as e:
                        print(f"发送结束消息失败: {e}")
                    target_all = []
                    num_targets = 0
            elif message.startswith('go'):
                if num_targets < len(target_all):
                    try:
                        formatted_data = format_target_for_send(target_all[num_targets])
                        send_to_client_safe('qianduan', formatted_data)
                        # send_to_client_safe('rob', formatted_data)
                        num_targets += 1
                    except Exception as e:
                        print(f"发送数据失败: {e}")
                else:
                    try:
                        send_to_client_safe('qianduan', 'over')
                        # send_to_client_safe('rob', 'over')
                    except Exception as e:
                        print(f"发送结束消息失败: {e}")
                    target_all = []
                    num_targets = 0
            elif message.startswith('clean'):
                print(f"接收到清理命令，清空目标数据")
                target_all = []
                num_targets = 0
            elif message.startswith('getimg'):
                try:
                    target_all = []
                    num_targets = 0
                    send_to_client_safe('vision', 'getimg',False)
                    print(f"已向客户端 vision 发送拍照请求")
                except Exception as e:
                    print(f"发送图片请求失败: {e}")
                





    except Exception as e:
        print(f"与 {client_address} 通信时发生错误: {e}")

    finally:
        try:
            client_socket.shutdown(socket.SHUT_RDWR)
        except:
            pass
        client_socket.close()
        print(f"客户端 {client_address} 已断开连接")

        with client_sockets_lock:
            if client_socket in client_sockets:
                client_sockets.remove(client_socket)
            for key, value in list(client_dict.items()):
                if value[0] == client_socket:
                    del client_dict[key]
                    break

        # 安全地移除当前线程
        try:
            client_threads.remove(threading.current_thread())
        except ValueError:
            pass



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
    global running
    running = True
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('0.0.0.0', 7930))
    server_socket.listen(5)
    print("服务端已启动，等待客户端连接...")

    def handle_input(server_sock):
        global running
        while running:
            user_input = input()
            if user_input == 'q':
                print("服务端正在关闭...")
                running = False

                # 关闭所有客户端 socket
                with client_sockets_lock:
                    for sock in client_sockets:
                        try:
                            peer = sock.getpeername()
                            sock.shutdown(socket.SHUT_RDWR)
                            sock.close()
                            print(f"关闭客户端 {peer} 的socket")
                        except Exception as e:
                            print(f"关闭客户端socket时发生错误: {e}")
                    client_sockets.clear()
                    client_dict.clear()

                # 关闭服务端 socket
                try:
                    server_sock.close()
                except:
                    pass
                break

    input_thread = threading.Thread(target=handle_input, args=(server_socket,), daemon=True)
    input_thread.start()

    try:
        while running:
            try:
                client_socket, client_address = server_socket.accept()
                client_thread = threading.Thread(target=handle_client, args=(client_socket, client_address))
                client_thread.start()
                client_threads.append(client_thread)
                with client_sockets_lock:
                    client_sockets.append(client_socket)
            except OSError:
                break
    finally:
        try:
            server_socket.close()
        except:
            pass
        print("服务端已关闭")

        for thread in client_threads:
            thread.join()
        input_thread.join()


if __name__ == "__main__":
    # 初始化
    
    # ——————————————————————————————————————————————————————————————————————————
    main()
    print("程序结束")