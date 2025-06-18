import cv2
import numpy as np

# 读取图像
img = cv2.imread('C:/Users/johd/Desktop/vision/txt/标定版标定.jpg')

# 定义棋盘格大小
size = (10, 7)

# 定义世界坐标系中的角点坐标
objp = np.zeros((size[0] * size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:size[0], 0:size[1]].T.reshape(-1, 2)

# 定义图像坐标系中的角点坐标
imgp = np.zeros((size[0] * size[1], 2), np.float32)
imgp[:, :2] = np.mgrid[0:size[0], 0:size[1]].T.reshape(-1, 2)

# 标定
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints=[objp], imgpoints=[imgp], imgsize=(img.shape[1], img.shape[0]), cameraMatrix=None, distCoeffs=None)

# 保存内参和畸变系数
np.savez('C:/Users/johd/Desktop/vision/txt/标定参数.npz', mtx=mtx, dist=dist)

# 绘制棋盘格
for i in range(size[0]):
    for j in range(size[1]):
        cv2.circle(img, (int(imgp[i*size[1]+j][0]), int(imgp[i*size[1]+j][1])), 5, (0, 0, 255), -1)

# 显示标定结果
cv2.imshow('img', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
