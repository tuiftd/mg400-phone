import socket
import threading
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox, filedialog
import time
import numpy as np
import json
from PIL import Image, ImageTk, ImageDraw
import cv2

class FrontendClient:
    def __init__(self):
        self.socket = None
        self.connected = False
        self.running = False
        self.receive_thread = None
        
        # 图像相关变量
        self.current_image = None
        self.display_image = None
        self.image_path = ""
        self.canvas = None
        
        # 相机参数
        self.K = None
        self.dist = None
        self.R = None
        self.t = None
        self.M_cam2rob = None
        
        # 加载相机参数和变换矩阵
        self.load_camera_params()
        self.load_transform_matrix()
        
        # 创建主窗口
        self.root = tk.Tk()
        self.root.title("机械臂控制前端客户端")
        self.root.geometry("1200x800")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        self.setup_ui()
        
    def load_camera_params(self):
        """从JSON文件加载相机参数"""
        try:
            with open("camera_params.json", 'r') as f:
                data = json.load(f)
            
            self.K = np.array(data["CameraMatrix"])
            self.dist = np.array(data["DistCoeffs"])
            self.R = np.array(data["Rotation"])
            self.t = np.array(data["TranslationVector"])
            
            print("成功从camera_params.json加载相机参数")
        except FileNotFoundError:
            print("错误：找不到camera_params.json文件")
        except Exception as e:
            print(f"加载相机参数时出错：{e}")

    def load_transform_matrix(self):
        """从JSON文件加载相机到机械臂的变换矩阵"""
        try:
            with open("affine_transform.json", 'r') as f:
                data = json.load(f)
            
            # 检查是否是2x3仿射矩阵
            if "AffineMatrix" in data:
                affine_matrix = np.array(data["AffineMatrix"])
                # 转换为3x3齐次矩阵
                self.M_cam2rob = np.vstack([affine_matrix, [0, 0, 1]])
            # 检查是否是3x3齐次矩阵
            elif "HomogeneousMatrix" in data:
                self.M_cam2rob = np.array(data["HomogeneousMatrix"])
            # 检查是否直接是TransformMatrix
            elif "TransformMatrix" in data:
                self.M_cam2rob = np.array(data["TransformMatrix"])
            else:
                print("错误：JSON文件中未找到有效的变换矩阵格式")
                return
            
            print("成功从affine_transform.json加载变换矩阵")
            
        except FileNotFoundError:
            print("错误：找不到affine_transform.json文件，使用默认矩阵")
            # 返回默认变换矩阵
            self.M_cam2rob = np.array([
                [9.99277962e-01, 3.44535836e-02, 2.91991446e+02],
                [-4.00650989e-02, 1.00289632e+00, -3.02108143e+01],
                [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
            ])
            print(f"使用默认变换矩阵")
        except Exception as e:
            print(f"加载变换矩阵时出错：{e}")
        
    def setup_ui(self):
        # 创建主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 连接设置框架
        conn_frame = ttk.LabelFrame(main_frame, text="连接设置", padding="10")
        conn_frame.grid(row=0, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # 服务器地址输入
        ttk.Label(conn_frame, text="服务器地址:").grid(row=0, column=0, sticky=tk.W)
        self.server_ip = tk.StringVar(value="127.0.0.1")
        ttk.Entry(conn_frame, textvariable=self.server_ip, width=15).grid(row=0, column=1, padx=(5, 10))
        
        # 端口输入
        ttk.Label(conn_frame, text="端口:").grid(row=0, column=2, sticky=tk.W)
        self.server_port = tk.StringVar(value="7930")
        ttk.Entry(conn_frame, textvariable=self.server_port, width=8).grid(row=0, column=3, padx=(5, 10))
        
        # 连接按钮
        self.connect_btn = ttk.Button(conn_frame, text="连接", command=self.connect_to_server)
        self.connect_btn.grid(row=0, column=4, padx=(5, 0))
        
        self.disconnect_btn = ttk.Button(conn_frame, text="断开", command=self.disconnect_from_server, state="disabled")
        self.disconnect_btn.grid(row=0, column=5, padx=(5, 0))
        
        # 连接状态显示
        self.status_label = ttk.Label(conn_frame, text="状态: 未连接", foreground="red")
        self.status_label.grid(row=1, column=0, columnspan=6, sticky=tk.W, pady=(5, 0))
        
        # 左侧框架：控制命令和消息
        left_frame = ttk.Frame(main_frame)
        left_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 10))
        
        # 控制命令框架
        cmd_frame = ttk.LabelFrame(left_frame, text="控制命令", padding="10")
        cmd_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 10))
        
        # 图像路径输入
        ttk.Label(cmd_frame, text="图像路径:").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.image_path_var = tk.StringVar(value=r"show\show.bmp")
        image_path_entry = ttk.Entry(cmd_frame, textvariable=self.image_path_var)
        image_path_entry.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=2)
        
        # 基本命令按钮
        ttk.Button(cmd_frame, text="获取图像 (getimg)", 
                  command=self.getimg_command).grid(row=2, column=0, sticky=(tk.W, tk.E), pady=2)
        
        ttk.Button(cmd_frame, text="获取下一个目标 (go)", 
                  command=lambda: self.send_command("go")).grid(row=3, column=0, sticky=(tk.W, tk.E), pady=2)
        
        ttk.Button(cmd_frame, text="清理目标数据 (clean)", 
                  command=lambda: self.send_command("clean")).grid(row=4, column=0, sticky=(tk.W, tk.E), pady=2)
        
        # 自定义命令输入
        ttk.Label(cmd_frame, text="自定义命令:").grid(row=5, column=0, sticky=tk.W, pady=(10, 2))
        self.custom_command = tk.StringVar()
        custom_entry = ttk.Entry(cmd_frame, textvariable=self.custom_command)
        custom_entry.grid(row=6, column=0, sticky=(tk.W, tk.E), pady=2)
        custom_entry.bind("<Return>", lambda e: self.send_custom_command())
        
        ttk.Button(cmd_frame, text="发送自定义命令", 
                  command=self.send_custom_command).grid(row=7, column=0, sticky=(tk.W, tk.E), pady=2)
        
        # 配置列权重
        cmd_frame.columnconfigure(0, weight=1)
        
        # 消息显示框架
        msg_frame = ttk.LabelFrame(left_frame, text="消息日志", padding="10")
        msg_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 消息显示区域
        self.message_text = scrolledtext.ScrolledText(msg_frame, width=40, height=15)
        self.message_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 清空消息按钮
        ttk.Button(msg_frame, text="清空消息", 
                  command=self.clear_messages).grid(row=1, column=0, sticky=tk.E, pady=(5, 0))
        
        # 配置左侧框架权重
        msg_frame.columnconfigure(0, weight=1)
        msg_frame.rowconfigure(0, weight=1)
        left_frame.rowconfigure(0, weight=0)
        left_frame.rowconfigure(1, weight=1)
        left_frame.columnconfigure(0, weight=1)
        
        # 右侧框架：图像显示
        image_frame = ttk.LabelFrame(main_frame, text="图像显示", padding="10")
        image_frame.grid(row=1, column=1, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 图像显示画布
        self.canvas = tk.Canvas(image_frame, width=600, height=480, bg='white')
        self.canvas.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 添加滚动条
        h_scrollbar = ttk.Scrollbar(image_frame, orient="horizontal", command=self.canvas.xview)
        h_scrollbar.grid(row=1, column=0, sticky=(tk.W, tk.E))
        v_scrollbar = ttk.Scrollbar(image_frame, orient="vertical", command=self.canvas.yview)
        v_scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        
        self.canvas.configure(xscrollcommand=h_scrollbar.set, yscrollcommand=v_scrollbar.set)
        
        # 图像信息显示
        self.image_info_label = ttk.Label(image_frame, text="未加载图像")
        self.image_info_label.grid(row=2, column=0, columnspan=2, pady=(5, 0))
        
        # 配置图像框架权重
        image_frame.columnconfigure(0, weight=1)
        image_frame.rowconfigure(0, weight=1)
        
        # 配置主框架权重
        main_frame.columnconfigure(0, weight=0)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(1, weight=1)
        
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

    def getimg_command(self):
        """获取图像命令 - 发送getimg指令并延时加载图像"""
        # 首先发送getimg命令到服务器
        self.send_command("getimg")
        
        # 延时1秒后加载图像
        self.root.after(1000, self.load_image_from_path)

    def load_image_from_path(self):
        """从指定路径加载图像文件"""
        file_path = self.image_path_var.get().strip()
        
        if not file_path:
            self.log_message("请输入图像路径", "ERROR")
            return
        
        try:
            # 使用OpenCV加载图像
            self.current_image = cv2.imread(file_path)
            if self.current_image is None:
                raise ValueError(f"无法加载图像文件: {file_path}")
            
            # 转换为RGB格式
            image_rgb = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2RGB)
            
            # 转换为PIL图像
            pil_image = Image.fromarray(image_rgb)
            
            # 计算显示尺寸（保持比例）
            canvas_width = self.canvas.winfo_width()
            canvas_height = self.canvas.winfo_height()
            
            if canvas_width <= 1 or canvas_height <= 1:
                canvas_width, canvas_height = 600, 480
            
            img_width, img_height = pil_image.size
            scale = min(canvas_width/img_width, canvas_height/img_height, 1.0)
            
            new_width = int(img_width * scale)
            new_height = int(img_height * scale)
            
            # 调整图像大小
            self.display_image = pil_image.resize((new_width, new_height), Image.Resampling.LANCZOS)
            
            # 转换为Tkinter格式
            self.photo = ImageTk.PhotoImage(self.display_image)
            
            # 显示图像
            self.canvas.delete("all")
            self.canvas.create_image(0, 0, anchor=tk.NW, image=self.photo)
            self.canvas.configure(scrollregion=self.canvas.bbox("all"))
            
            # 更新信息
            self.image_path = file_path
            filename = file_path.split('\\')[-1] if '\\' in file_path else file_path.split('/')[-1]
            self.image_info_label.config(text=f"图像: {filename} ({img_width}x{img_height})")
            
            self.log_message(f"成功加载图像: {file_path}")
            
        except Exception as e:
            self.log_message(f"加载图像失败: {e}", "ERROR")

    def load_image(self):
        """保留原有的手动加载图像功能（可选）"""
        file_path = filedialog.askopenfilename(
            title="选择图像文件",
            filetypes=[
                ("图像文件", "*.jpg *.jpeg *.png *.bmp *.tiff"),
                ("所有文件", "*.*")
            ]
        )
        
        if file_path:
            # 更新路径输入框
            self.image_path_var.set(file_path)
            # 加载图像
            self.load_image_from_path()
    
    def world_to_pixel(self, world_point):
        """将世界坐标转换为像素坐标"""
        if self.K is None or self.R is None or self.t is None:
            return None
            
        try:
            X, Y = world_point
            # 构造单应性矩阵
            r1 = self.R[:, 0].reshape(3, 1)
            r2 = self.R[:, 1].reshape(3, 1)
            H = np.hstack([r1, r2, self.t])
            
            # 世界坐标转换为归一化相机坐标
            world_homogeneous = np.array([[X], [Y], [1]], dtype=np.float32)
            normalized_pixel = np.dot(H, world_homogeneous)
            
            # 归一化相机坐标转换为像素坐标
            pixel = np.dot(self.K, normalized_pixel)
            u = pixel[0, 0] / pixel[2, 0]
            v = pixel[1, 0] / pixel[2, 0]
            
            return int(u), int(v)
        except Exception as e:
            print(f"世界坐标转像素坐标失败: {e}")
            return None
    
    def robot_to_world(self, robot_point):
        """将机械臂坐标转换为世界坐标"""
        if self.M_cam2rob is None:
            return None
            
        try:
            x_rob, y_rob = robot_point
            # 计算逆变换矩阵
            M_rob2cam = np.linalg.inv(self.M_cam2rob)
            
            # 转换为齐次坐标
            robot_homogeneous = np.array([x_rob, y_rob, 1])
            
            # 应用逆变换
            world_homogeneous = np.dot(M_rob2cam, robot_homogeneous)
            
            return world_homogeneous[0], world_homogeneous[1]
        except Exception as e:
            print(f"机械臂坐标转世界坐标失败: {e}")
            return None
    
    def robot_to_pixel(self, robot_point):
        """将机械臂坐标转换为像素坐标"""
        # 第一步：机械臂坐标 → 世界坐标
        world_point = self.robot_to_world(robot_point)
        if world_point is None:
            return None
            
        # 第二步：世界坐标 → 像素坐标
        pixel_point = self.world_to_pixel(world_point)
        return pixel_point
    
    def draw_coordinates_on_image(self, target_coords, phone_coords):
        """在图像上绘制目标和手机坐标"""
        if self.current_image is None:
            self.log_message("请先加载图像", "ERROR")
            return
            
        try:
            # 复制原始图像
            image_with_coords = self.current_image.copy()
            
            # 将机械臂坐标转换为像素坐标
            target_pixel = self.robot_to_pixel(target_coords[:2])
            phone_pixel = self.robot_to_pixel(phone_coords[:2])
            
            if target_pixel is None or phone_pixel is None:
                self.log_message("坐标转换失败", "ERROR")
                return
            
            target_u, target_v = target_pixel
            phone_u, phone_v = phone_pixel
            
            # 绘制目标点（红色圆圈）
            cv2.circle(image_with_coords, (target_u, target_v), 10, (0, 0, 255), 2)
            cv2.putText(image_with_coords, f"Target({target_u},{target_v})", 
                       (target_u + 15, target_v - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            # 绘制手机点（蓝色圆圈）
            cv2.circle(image_with_coords, (phone_u, phone_v), 10, (255, 0, 0), 2)
            cv2.putText(image_with_coords, f"Phone({phone_u},{phone_v})", 
                       (phone_u + 15, phone_v + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            
            # 绘制连接线
            cv2.line(image_with_coords, (target_u, target_v), (phone_u, phone_v), (0, 255, 0), 2)
            
            # 转换为RGB并显示
            image_rgb = cv2.cvtColor(image_with_coords, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(image_rgb)
            
            # 计算显示尺寸
            canvas_width = self.canvas.winfo_width()
            canvas_height = self.canvas.winfo_height()
            
            if canvas_width <= 1 or canvas_height <= 1:
                canvas_width, canvas_height = 600, 480
            
            img_width, img_height = pil_image.size
            scale = min(canvas_width/img_width, canvas_height/img_height, 1.0)
            
            new_width = int(img_width * scale)
            new_height = int(img_height * scale)
            
            self.display_image = pil_image.resize((new_width, new_height), Image.Resampling.LANCZOS)
            self.photo = ImageTk.PhotoImage(self.display_image)
            
            # 更新画布
            self.canvas.delete("all")
            self.canvas.create_image(0, 0, anchor=tk.NW, image=self.photo)
            self.canvas.configure(scrollregion=self.canvas.bbox("all"))
            
            self.log_message(f"已在图像上绘制坐标点: Target({target_u},{target_v}), Phone({phone_u},{phone_v})")
            
        except Exception as e:
            self.log_message(f"绘制坐标失败: {e}", "ERROR")
    
    def parse_coordinate_message(self, message):
        """解析坐标消息"""
        try:
            # 消息格式: "ok,x_rob,y_rob,r_rob,x_phone,y_phone,r_phone"
            parts = message.split(',')
            if len(parts) >= 7:
                status = parts[0]
                x_rob = float(parts[1])
                y_rob = float(parts[2])
                r_rob = float(parts[3])
                x_phone = float(parts[4])
                y_phone = float(parts[5])
                r_phone = float(parts[6])
                
                return status, (x_rob, y_rob, r_rob), (x_phone, y_phone, r_phone)
        except:
            pass
        return None, None, None
        
    def log_message(self, message, msg_type="INFO"):
        """在消息区域显示消息"""
        timestamp = time.strftime("%H:%M:%S")
        formatted_msg = f"[{timestamp}] {msg_type}: {message}\n"
        
        self.message_text.insert(tk.END, formatted_msg)
        self.message_text.see(tk.END)
        
        # 根据消息类型设置颜色
        if msg_type == "ERROR":
            start_line = self.message_text.get("1.0", tk.END).count('\n')
            self.message_text.tag_add("error", f"{start_line}.0", f"{start_line}.end")
            self.message_text.tag_config("error", foreground="red")
        elif msg_type == "RECEIVED":
            start_line = self.message_text.get("1.0", tk.END).count('\n')
            self.message_text.tag_add("received", f"{start_line}.0", f"{start_line}.end")
            self.message_text.tag_config("received", foreground="blue")
            
    def clear_messages(self):
        """清空消息显示区域"""
        self.message_text.delete(1.0, tk.END)
        
    def connect_to_server(self):
        """连接到服务器"""
        if self.connected:
            return
            
        try:
            server_ip = self.server_ip.get().strip()
            server_port = int(self.server_port.get().strip())
            
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((server_ip, server_port))
            
            # 发送客户端名字
            client_name = "qianduan"
            self.socket.send(client_name.encode('utf-8'))
            
            self.connected = True
            self.running = True
            
            # 启动接收消息的线程
            self.receive_thread = threading.Thread(target=self.receive_messages, daemon=True)
            self.receive_thread.start()
            
            # 更新UI状态
            self.status_label.config(text=f"状态: 已连接到 {server_ip}:{server_port}", foreground="green")
            self.connect_btn.config(state="disabled")
            self.disconnect_btn.config(state="normal")
            
            self.log_message(f"成功连接到服务器 {server_ip}:{server_port}")
            
        except Exception as e:
            self.log_message(f"连接失败: {e}", "ERROR")
            messagebox.showerror("连接错误", f"无法连接到服务器: {e}")
            
    def disconnect_from_server(self):
        """断开与服务器的连接"""
        if not self.connected:
            return
            
        try:
            self.running = False
            self.connected = False
            
            if self.socket:
                self.socket.close()
                
            # 更新UI状态
            self.status_label.config(text="状态: 未连接", foreground="red")
            self.connect_btn.config(state="normal")
            self.disconnect_btn.config(state="disabled")
            
            self.log_message("已断开与服务器的连接")
            
        except Exception as e:
            self.log_message(f"断开连接时出错: {e}", "ERROR")
            
    def send_command(self, command):
        """发送命令到服务器"""
        if not self.connected:
            messagebox.showwarning("连接警告", "请先连接到服务器")
            return
            
        try:
            self.socket.send(command.encode('utf-8'))
            self.log_message(f"已发送命令: {command}")
        except Exception as e:
            self.log_message(f"发送命令失败: {e}", "ERROR")
            messagebox.showerror("发送错误", f"发送命令失败: {e}")
            
    def send_custom_command(self):
        """发送自定义命令"""
        command = self.custom_command.get().strip()
        if not command:
            return
            
        self.send_command(command)
        self.custom_command.set("")
        
    def receive_messages(self):
        """接收服务器消息的线程函数"""
        while self.running and self.connected:
            try:
                data = self.socket.recv(1024)
                if not data:
                    break
                    
                message = data.decode('utf-8').strip()
                if message:
                    # 在UI线程中处理消息
                    self.root.after(0, lambda msg=message: self.process_received_message(msg))
                    
            except Exception as e:
                if self.running:
                    self.root.after(0, lambda: self.log_message(f"接收消息时出错: {e}", "ERROR"))
                break
                
        # 连接断开时的处理
        if self.connected:
            self.root.after(0, self.handle_connection_lost)
    
    def process_received_message(self, message):
        """处理接收到的消息"""
        self.log_message(f"收到: {message}", "RECEIVED")
        
        # 检查是否是坐标数据消息
        status, target_coords, phone_coords = self.parse_coordinate_message(message)
        if status is not None and target_coords is not None and phone_coords is not None:
            if status == "ok":
                # 在图像上绘制坐标
                self.draw_coordinates_on_image(target_coords, phone_coords)
            else:
                self.log_message(f"收到状态: {status}", "ERROR")
            
    def handle_connection_lost(self):
        """处理连接丢失"""
        self.connected = False
        self.running = False
        
        self.status_label.config(text="状态: 连接丢失", foreground="red")
        self.connect_btn.config(state="normal")
        self.disconnect_btn.config(state="disabled")
        
        self.log_message("与服务器的连接已丢失", "ERROR")
        
    def on_closing(self):
        """关闭窗口时的处理"""
        if self.connected:
            self.disconnect_from_server()
            
        self.root.destroy()
        
    def run(self):
        """启动客户端"""
        self.root.mainloop()

if __name__ == "__main__":
    client = FrontendClient()
    client.run()