import cv2
import numpy as np

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
    #world_point_cartesian= world_point / world_point[2]  # 归一化

    return world_point[:2]

# 从XML文件中读取相机参数
def read_camera_params_from_xml(filename):
    fs = cv2.FileStorage(filename, cv2.FILE_STORAGE_READ)
    K = fs.getNode("K").mat()
    dist = fs.getNode("dist").mat()
    rvec = fs.getNode("rvec").mat()
    tvec = fs.getNode("tvec").mat()
    fs.release()
    
    R, _ = cv2.Rodrigues(rvec)  # 将旋转向量转换为旋转矩阵
    return K, R, tvec, dist

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

# 读取标定图像
image_list = [f"c:/Users/johd/Desktop/vision/biaodingban/1_{j}.bmp" for j in range(1, 11)]
for fname in image_list:
    img = cv2.imread(fname)
    if img is None:
        continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    try:
    # 角点检测
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None,flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ret:
            raise ValueError("角点检测失败")
    except ValueError as e:
        print(f"图像 {fname} 角点检测失败: {e}")
    else:
            # 使用亚像素级精确化角点位置
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1))
        # print("角点坐标：")
        # for corner in corners2:
        #     print(corner[0])
        
        # 在图像上绘制角点
        cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
        
        # 显示结果图像
        cv2.imshow('img', img)
        key = cv2.waitKey(0)
        if key == 13:
            objpoints.append(objp)
            imgpoints.append(corners2)
        else:
            print("舍弃这组值")
# 计算相机内参和外参
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
# 创建一个FileStorage对象用于写入
fs = cv2.FileStorage("camera_params.xml", cv2.FILE_STORAGE_WRITE)
# 写入内参矩阵
fs.write("CameraMatrix", K)
# 写入畸变系数
fs.write("DistCoeffs", dist)
# 写入第一个外参矩阵（旋转矩阵）
fs.write("Rotation", R)
# 写入第一个外参矩阵（位移向量）
fs.write("TranslationVector", t)
# 释放FileStorage对象
fs.release()
# X, Y = pixel_to_world(pixel, K, R, t)
# print(f"世界坐标: ({X}, {Y})")