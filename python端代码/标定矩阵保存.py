import numpy as np
import cv2
# 假设这是你的变换矩阵
transformation_matrix = np.array([
    [ 9.99277962e-01  ,3.44535836e-02 , 2.91991446e+02],
 [-4.00650989e-02 , 1.00289632e+00 ,-3.02108143e+01],
    [ 0.00000000e+00  ,0.00000000e+00  ,1.00000000e+00]
])

def pixel_to_world(pixel, K, R, t):
    """ 将像素坐标转换为世界坐标（假设 Z = 0） """
    u, v = pixel
    K_inv = np.linalg.inv(K)  # 计算 K 的逆

    # 计算归一化相机坐标
    normalized_pixel = np.dot(K_inv, np.array([[u], [v], [1]], dtype=np.float32))  # (3,1)

    # 计算旋转矩阵 R 的前两列
    R_inv = np.linalg.inv(R)
    r1 = R[:, 0].reshape(3, 1)
    r2 = R[:, 1].reshape(3, 1)

    # 计算世界坐标
    H = np.hstack([r1, r2, t])  # 构造 3×3 变换矩阵 H
    H_inv = np.linalg.inv(H)  # 计算 H 的逆

    world_point = np.dot(H_inv, normalized_pixel)  # 变换到世界坐标
    X, Y = world_point[:2].flatten() / world_point[2, 0]  # 归一化

    return X, Y
# 保存矩阵到文件
np.savetxt('transformation_matrix.txt', transformation_matrix)
# 从文件中读取矩阵
matrix_loaded = np.loadtxt('transformation_matrix.txt')

# 使用读取的矩阵进行验证
def verify_transformation(matrix, point_camera):
    """
    验证变换矩阵是否正确。

    输入:
        matrix: 3x3 的变换矩阵
        point_camera: 相机坐标系中的点 [x, y]
        
    输出:
        point_arm: 变换后的机械臂坐标系中的点 [X, Y]
    """
    point = np.array([point_camera[0], point_camera[1], 1])
    transformed_point = np.dot(matrix, point)
    return [transformed_point[0], transformed_point[1]]

# 示例验证
# 创建一个FileStorage对象用于读取
fs = cv2.FileStorage("camera_params.xml", cv2.FILE_STORAGE_READ)

# 读取内参矩阵
K = fs.getNode("CameraMatrix").mat()

# 读取畸变系数
dist = fs.getNode("DistCoeffs").mat()

# 读取第一个外参矩阵（旋转向量）
R = fs.getNode("Rotation").mat()

# 读取第一个外参矩阵（位移向量）
t = fs.getNode("TranslationVector").mat()

# 释放FileStorage对象
fs.release()
x_input = float(input("请输入像素 x 坐标: "))
y_input = float(input("请输入像素 y 坐标: "))
point_camera = pixel_to_world([x_input, y_input], K, R, t)
transformed_point = verify_transformation(matrix_loaded, point_camera)
print(f"机械臂坐标系中的坐标: ({transformed_point[0]:.4f}, {transformed_point[1]:.4f})")