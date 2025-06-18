import socket
import threading
from fenge import parse_data_new
import numpy as np
client_threads = []  # 保存所有客户端线程
client_sockets = []  # 用于保存所有客户端socket
client_sockets_lock = threading.Lock()  # 客户端socket列表的锁
client_dict = {}  # 用于保存客户端名字到socket和address的映射
running = False
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
    global client_dict
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