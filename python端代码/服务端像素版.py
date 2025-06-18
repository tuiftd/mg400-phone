import socket
import threading
client_threads = []  # 保存所有客户端线程
client_sockets = []  # 用于保存所有客户端socket
client_sockets_lock = threading.Lock()  # 客户端socket列表的锁
client_dict = {}  # 用于保存客户端名字到socket和address的映射
running = False


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




def handle_client(client_socket, client_address):
    global client_dict
    print(f"客户端 {client_address} 已连接")
    try:
        client_name = client_socket.recv(1024).decode('utf-8').strip()
        print(f"客户端名字: {client_name}")
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