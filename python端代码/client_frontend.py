import socket
import threading
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import time

class FrontendClient:
    def __init__(self):
        self.socket = None
        self.connected = False
        self.running = False
        self.receive_thread = None
        
        # 创建主窗口
        self.root = tk.Tk()
        self.root.title("机械臂控制前端客户端")
        self.root.geometry("800x600")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        self.setup_ui()
        
    def setup_ui(self):
        # 创建主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 连接设置框架
        conn_frame = ttk.LabelFrame(main_frame, text="连接设置", padding="10")
        conn_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
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
        
        # 控制命令框架
        cmd_frame = ttk.LabelFrame(main_frame, text="控制命令", padding="10")
        cmd_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 10))
        
        # 基本命令按钮（移除first按钮）
        ttk.Button(cmd_frame, text="获取图像 (getimg)", 
                  command=lambda: self.send_command("getimg")).grid(row=0, column=0, sticky=(tk.W, tk.E), pady=2)
        
        ttk.Button(cmd_frame, text="获取下一个目标 (go)", 
                  command=lambda: self.send_command("go")).grid(row=1, column=0, sticky=(tk.W, tk.E), pady=2)
        
        ttk.Button(cmd_frame, text="清理目标数据 (clean)", 
                  command=lambda: self.send_command("clean")).grid(row=2, column=0, sticky=(tk.W, tk.E), pady=2)
        
        # 自定义命令输入
        ttk.Label(cmd_frame, text="自定义命令:").grid(row=3, column=0, sticky=tk.W, pady=(10, 2))
        self.custom_command = tk.StringVar()
        custom_entry = ttk.Entry(cmd_frame, textvariable=self.custom_command)
        custom_entry.grid(row=4, column=0, sticky=(tk.W, tk.E), pady=2)
        custom_entry.bind("<Return>", lambda e: self.send_custom_command())
        
        ttk.Button(cmd_frame, text="发送自定义命令", 
                  command=self.send_custom_command).grid(row=5, column=0, sticky=(tk.W, tk.E), pady=2)
        
        # 配置列权重
        cmd_frame.columnconfigure(0, weight=1)
        
        # 消息显示框架
        msg_frame = ttk.LabelFrame(main_frame, text="消息日志", padding="10")
        msg_frame.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 消息显示区域
        self.message_text = scrolledtext.ScrolledText(msg_frame, width=50, height=20)
        self.message_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 清空消息按钮
        ttk.Button(msg_frame, text="清空消息", 
                  command=self.clear_messages).grid(row=1, column=0, sticky=tk.E, pady=(5, 0))
        
        # 配置权重
        msg_frame.columnconfigure(0, weight=1)
        msg_frame.rowconfigure(0, weight=1)
        
        # 配置主框架权重
        main_frame.columnconfigure(0, weight=0)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(1, weight=1)
        
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        
    def log_message(self, message, msg_type="INFO"):
        """在消息区域显示消息"""
        timestamp = time.strftime("%H:%M:%S")
        formatted_msg = f"[{timestamp}] {msg_type}: {message}\n"
        
        self.message_text.insert(tk.END, formatted_msg)
        self.message_text.see(tk.END)
        
        # 根据消息类型设置颜色
        if msg_type == "ERROR":
            # 设置错误消息为红色
            start_line = self.message_text.get("1.0", tk.END).count('\n')
            self.message_text.tag_add("error", f"{start_line}.0", f"{start_line}.end")
            self.message_text.tag_config("error", foreground="red")
        elif msg_type == "RECEIVED":
            # 设置接收消息为蓝色
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
            client_name = "qianduan"  # 根据服务端代码，前端客户端名字为qianduan
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
        self.custom_command.set("")  # 清空输入框
        
    def receive_messages(self):
        """接收服务器消息的线程函数"""
        while self.running and self.connected:
            try:
                data = self.socket.recv(1024)
                if not data:
                    break
                    
                message = data.decode('utf-8').strip()
                if message:
                    # 在UI线程中显示消息
                    self.root.after(0, lambda msg=message: self.log_message(f"收到: {msg}", "RECEIVED"))
                    
            except Exception as e:
                if self.running:
                    self.root.after(0, lambda: self.log_message(f"接收消息时出错: {e}", "ERROR"))
                break
                
        # 连接断开时的处理
        if self.connected:
            self.root.after(0, self.handle_connection_lost)
            
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