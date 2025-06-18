import cv2
import numpy as np
import json
import matplotlib.pyplot as plt

def pixel_to_world(pixel, K, R, t):
    u, v = pixel
    K_inv = np.linalg.inv(K)
    pixel_point = np.array([u, v, 1]).reshape(3, 1)
    # 计算归一化相机坐标
    normalized_point = np.dot(K_inv, pixel_point)

    # 计算世界坐标（假设 Z = 0）
    R_inv = np.linalg.inv(R)
    t_inv = -np.dot(R_inv, t)
    
    world_point = np.dot(R_inv, normalized_point) + t_inv
    print("归一化相机坐标：", normalized_point)
    print("世界坐标：", world_point)
    
    return world_point[:2]

# 从JSON文件中读取相机参数
def read_camera_params_from_json(filename):
    with open(filename, 'r') as f:
        data = json.load(f)
    
    K = np.array(data["CameraMatrix"])
    dist = np.array(data["DistCoeffs"])
    R = np.array(data["Rotation"])
    t = np.array(data["TranslationVector"])
    
    return K, R, t, dist

# 棋盘格参数
chessboard_size = (3, 3)  # 棋盘格内部角点数量（行, 列）
square_size = 25  # 每个方块的大小，单位 mm

# 3D 世界坐标初始化（假设棋盘格在 Z=0 平面上）
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2) * square_size
print("3D 世界坐标点：")
print(objp)

objpoints = []  # 3D 世界坐标点
imgpoints = []  # 2D 图像坐标点

# 设置matplotlib显示
plt.ion()  # 交互模式
fig, ax = plt.subplots(1, 1, figsize=(10, 8))

# 读取标定图像
image_list = [f"vision/biaodingban/1_{j}.bmp" for j in range(1, 11)]
for i, fname in enumerate(image_list):
    print(f"处理图像 {i+1}/10: {fname}")
    img = cv2.imread(fname)
    if img is None:
        print(f"无法读取图像: {fname}")
        continue
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # 角点检测
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None,
                                           flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
    
    if ret:
        # 使用亚像素级精确化角点位置
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), 
                                  criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1))
        
        # 在图像上绘制角点
        cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
        
        # 使用matplotlib显示图像
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        ax.clear()
        ax.imshow(img_rgb)
        ax.set_title(f'图像 {i+1}: {fname}\n按Enter接受，按其他键跳过')
        ax.axis('off')
        plt.draw()
        plt.pause(0.1)
        
        # 等待用户输入
        response = input(f"图像 {i+1} 角点检测成功，按Enter接受，输入'n'跳过: ")
        if response.lower() != 'n':
            objpoints.append(objp)
            imgpoints.append(corners2)
            print(f"已接受图像 {fname}")
        else:
            print(f"已跳过图像 {fname}")
    else:
        print(f"图像 {fname} 角点检测失败")

plt.ioff()  # 关闭交互模式
plt.close()

print(f"成功处理 {len(objpoints)} 张图像用于标定")

if len(objpoints) == 0:
    print("错误：没有成功检测到任何角点，无法进行标定")
    exit()

# 计算相机内参和外参
print("开始计算相机标定参数...")
ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("重投影误差: ", ret)
print("内参矩阵 K:\n", K)
print("畸变系数 dist:\n", dist)

# 假设我们有一个像素点 (u, v)
pixel = (500, 400)  # 示例像素点
R, _ = cv2.Rodrigues(rvecs[0])  # 将旋转向量转换为旋转矩阵
t = tvecs[0].reshape(3, 1)  # 转换为列向量
print("旋转矩阵 R:\n", R)
print("平移向量 t:\n", t)

# 保存参数到JSON文件
camera_params = {
    "CameraMatrix": K.tolist(),
    "DistCoeffs": dist.tolist(),
    "Rotation": R.tolist(),
    "TranslationVector": t.tolist(),
    "ReprojectionError": float(ret)
}

with open("camera_params.json", 'w') as f:
    json.dump(camera_params, f, indent=4)

print("相机参数已保存到 camera_params.json")