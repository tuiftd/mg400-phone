import tkinter as tk
from tkinter import messagebox
import socket
import threading
import cv2
USERNAME = "1"
PASSWORD = "1"
client_socket=None
def check_login():
    username = username_entry.get()
    password = password_entry.get()
    if username == USERNAME and password == PASSWORD:
        login_frame1.forget()
        login_frame2.forget()
        login_frame3.forget()
        status_frame.place(x=450, y=60)
        x1.place(x=110, y=170, width=120, height=70)
        y1.place(x=230, y=170, width=120, height=70)
        #z1.place(x=270, y=170, width=70, height=70)
        x2.place(x=110, y=255, width=240, height=80)
        # y2.place(x=180, y=255, width=90, height=80)
        # z2.place(x=270, y=255, width=70, height=80)
        start.place(x=110, y=370, width=230, height=30)  # 启动线程，开始通讯
        #start_client()
    else:
        messagebox.showerror(title="登录失败", message="用户名或密码错误")

def start_client():
    global client_socket # 声明全局变量
    client_thread = threading.Thread(target=client_receive)
    client_thread.start()

def client_receive():
    # global client_socket
    # client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # client_socket.connect(('127.0.0.1', 4001))
    while True:
        msg = client_socket.recv(1024).decode('utf-8')
        if not msg:
            continue

        fields = msg.split(',')  # 注意缩进
        first = fields[0]  # 判断流程
        #获取列表长度
        if len(fields)>=2:
           print(fields[1])
        #second = fields[1]  # 可以根据不同的流程处理

        if first == "1":
            date = fields[1][0:6]
            year = date[0:4]
            month = date[4:6]

            check = date
            a = "20290104"
            if check > a:
                    check = "过期"
                    tag_NG = 'red_tag'
                    other_info_text.insert(tk.END, "生产日期：" + year + "年" + month + "月\n" + "是否过期：" + check + "\n", tag_NG)
                    other_info_text.insert(tk.END, ".................................\n")
                    other_info_text.tag_configure(tag_NG, foreground='red')
            else:
                    check = "没过期"
                    tag_OK = 'green_tag'
                    other_info_text.insert(tk.END, "生产日期：" + year + "年" + month + "月\n" + "是否过期：" + check + "\n", tag_OK)
                    other_info_text.insert(tk.END, ".................................\n")
                    other_info_text.tag_configure(tag_OK, foreground='green')

        if first == "2":
            if fields[1] == "GPS":
                output = "GPS"
                other_info_text.insert(tk.END, "产品名称：" + output + "\n")
                other_info_text.insert(tk.END, ".................................\n")

        if first == "ok":
            if fields[1] == "banyuan":
                output = "半圆"
            elif fields[1] == "tixing":
                output = "梯形"
            elif fields[1] == "yuanao":
                output = "圆凹"
            elif fields[1] == "liubianxing":
                output = "六边形"
            elif fields[1] == "wubianxing":
                output = "五边形"
            elif fields[1] == "fangao":
                output = "方凹"
            else:
                output=None
            if output:
                other_info_text.insert(tk.END, "产品名称：" + output + "\n")
                #other_info_text.insert(tk.END, "..........\n")
            fields = [float(field) for field in fields[2:]]
            x_scr = int(fields[7])
            y_scr = int(fields[6])
            x_to = int(fields[8])
            y_to = int(fields[9])
            fields = [round(num, 2) for num in fields]
            fields = [str(x) for x in fields]
            other_info_text.insert(tk.END, "目标坐标：(" + fields[0] + "，"+fields[1]+")"+"目标角度：" +fields[2]+"\n"+"到达坐标：("+fields[3]+"，"+fields[4]+")"+"到达角度：" +fields[5]+"\n")
            other_info_text.insert(tk.END, ".................................\n")
            show(x_scr,y_scr,x_to,y_to)
        if first=="over":
            other_info_text.insert(tk.END, "坐标全部发送完毕\n")



        


def X1():
    client_socket.sendall("X1".encode('utf-8'))
def X2():
    client_socket.sendall("X2".encode('utf-8'))
def Y1():
    # client_socket.sendall("Y1".encode('utf-8'))
    pass
def Y2():
    client_socket.sendall("Y2".encode('utf-8'))
def Z1():
    client_socket.sendall("Z1".encode('utf-8'))
def Z2():
    client_socket.sendall("Z2".encode('utf-8'))
def go():
    # client_socket.sendall("first".encode('utf-8'))
    pass
def login():
    pass
    #client_socket.sendall("qianduan".encode('utf-8'))
def check():
    check_login()
def show(x_scr=0,y_scr=0,x_to=1,y_to=1):
    start_point = (x_scr,y_scr)
    end_point = (x_to,y_to)
    image_path = "show//phone.jpg"
    img = cv2.imread(image_path)
    cv2.arrowedLine(img, start_point, end_point, (0, 255, 0), 2)
    scale_percent = 30  # 缩放到原来大小的30%
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)

    # 使用缩放因子调整图像大小
    img = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)
    cv2.imshow("show", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

root = tk.Tk()
root.title("登录界面")
root.geometry("900x525")

image_path = (r"C:\Users\johd\Desktop\3.png")
try:
    bg = tk.PhotoImage(file=image_path)
    bg_label = tk.Label(root, image=bg)
    bg_label.place(x=0, y=0, relwidth=1, relheight=1)
except tk.TclError as e:
    print(f"加载失败：{e}")

login_frame1 = tk.Frame(root)
login_frame1.place(x=110, y=205, height=30, width=230)
login_frame2 = tk.Frame(root)
login_frame2.place(x=110, y=278, height=30, width=230)


username_entry = tk.Entry(login_frame1, font=('Arial', 14))
username_entry.grid(row=0, column=1, padx=2, pady=2)
password_entry = tk.Entry(login_frame2, font=('Arial', 14))
password_entry.grid(row=1, column=1, padx=2, pady=2)

login_frame3 = tk.Frame(root)
login_frame3.place(x=110, y=370, width=230, height=30)

tk.Button(login_frame3, text="确 认", font=('Arial', 14), bg="lightblue", height=30, width=230, command=check).pack()  # 后面要加pack

status_frame = tk.Frame(root, bg="darkblue")
status_label = tk.Label(status_frame, text="信息栏", font=('Arial', 17), width=15, height=2, bg="skyblue")
status_label.pack(pady=6)

other_info_text = tk.Text(status_frame, height=18, width=35, font=('Arial', 12))
other_info_text.pack(padx=6, pady=6)

x1 = tk.Frame(root)
x2 = tk.Frame(root)
y1 = tk.Frame(root)
z1 = tk.Frame(root)
z2 = tk.Frame(root)
y2 = tk.Frame(root)

start = tk.Frame(root)
button_frame3 = tk.Frame(root)

tk.Button(x1, text="拍照", bg="skyblue", command=login, height=70, width=70, font=('Arial', 17)).pack()
tk.Button(x2, text="显示", bg="skyblue", command=show, height=80, width=70, font=('Arial', 14)).pack()
tk.Button(y1, text="推理", bg="skyblue", command=Y1, height=70, width=90, font=('Arial', 17)).pack()
tk.Button(start, text="全流程", bg="skyblue", command=go, height=80, width=70, font=('Arial', 17)).pack()

root.mainloop()
