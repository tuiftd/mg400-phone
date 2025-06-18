import subprocess
import os

# 定义服务端和前端脚本的路径
server_script_path = r"c:\Users\johd\Desktop\vision\txt\服务端.py"
frontend_script_path = r"c:\Users\johd\Desktop\vision\txt\11.py"

# 启动服务端脚本
print("启动服务端...")
server_process = subprocess.Popen(['python', server_script_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

# 启动前端脚本
print("启动前端...")
frontend_process = subprocess.Popen(['python', frontend_script_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

# 可选：等待服务端和前端脚本结束并打印输出
try:
    server_output, server_error = server_process.communicate()
    frontend_output, frontend_error = frontend_process.communicate()
    print("服务端输出:", server_output.decode('utf-8'))
    print("服务端错误:", server_error.decode('utf-8'))
    print("前端输出:", frontend_output.decode('utf-8'))
    print("前端错误:", frontend_error.decode('utf-8'))
except KeyboardInterrupt:
    # 用户按下Ctrl+C，终止进程
    print("用户中断，正在终止服务端和前端...")
    server_process.terminate()
    frontend_process.terminate()
    server_process.wait()
    frontend_process.wait()
    print("服务端和前端已终止。")
