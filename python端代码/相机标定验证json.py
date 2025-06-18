import cv2
import numpy as np
import json
import matplotlib.pyplot as plt

def world_to_pixel(world_point, K, R, t):
    """ 将世界坐标 (X, Y) 转换为像素坐标 (u, v)，假设 Z = 0 """
    X, Y = world_point
    world_h = np.array([[X], [Y], [1]], dtype=np.float32)  # 齐次世界坐标

    # 计算单应性矩阵 H = K [r1 r2 t]
    r1 = R[:, 0].reshape(3, 1)
    r2 = R[:, 1].reshape(3, 1)
    H = np.dot(K, np.hstack([r1, r2, t]))  # 计算 H

    # 计算像素坐标
    pixel_h = np.dot(H, world_h)  # 投影到像素坐标
    u, v = pixel_h[:2].flatten() / pixel_h[2, 0]  # 归一化

    return int(round(u)), int(round(v))

def pixel_to_world(pixel, K, R, t):
    """ 将像素坐标转换为世界坐标（假设 Z = 0） """
    u, v = pixel
    K_inv = np.linalg.inv(K)  # 计算 K 的逆

    # 计算归一化相机坐标
    normalized_pixel = np.dot(K_inv, np.array([[u], [v], [1]], dtype=np.float32))  # (3,1)

    # 计算旋转矩阵 R 的前两列
    r1 = R[:, 0].reshape(3, 1)
    r2 = R[:, 1].reshape(3, 1)

    # 计算世界坐标
    H = np.hstack([r1, r2, t])  # 构造 3×3 变换矩阵 H
    H_inv = np.linalg.inv(H)  # 计算 H 的逆

    world_point = np.dot(H_inv, normalized_pixel)  # 变换到世界坐标
    X, Y = world_point[:2].flatten() / world_point[2, 0]  # 归一化

    return X, Y

def read_camera_params_from_json(filename):
    """从JSON文件中读取相机参数"""
    with open(filename, 'r') as f:
        data = json.load(f)
    
    K = np.array(data["CameraMatrix"])
    dist = np.array(data["DistCoeffs"])
    R = np.array(data["Rotation"])
    t = np.array(data["TranslationVector"])
    
    return K, dist, R, t

def draw_coordinate_axes(img, org_pixel, x_pixel, y_pixel, guangxing_pixel):
    """在图像上绘制坐标轴"""
    # 绘制光心
    cv2.circle(img, (int(guangxing_pixel[0]), int(guangxing_pixel[1])), 10, (0, 0, 255), -1)
    
    # 绘制坐标轴
    cv2.arrowedLine(img, org_pixel, x_pixel, (0, 255, 0), 3)
    cv2.arrowedLine(img, org_pixel, y_pixel, (255, 0, 0), 3)
    
    # 添加文字标注
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1.5
    font_color = (255, 255, 255)
    line_type = 3
    
    x_position = (x_pixel[0] + 10, x_pixel[1] + 20)
    y_position = (y_pixel[0] + 10, y_pixel[1] + 20)
    
    cv2.putText(img, "X", x_position, font, font_scale, font_color, line_type)
    cv2.putText(img, "Y", y_position, font, font_scale, font_color, line_type)
    
    return img

# 从JSON文件读取相机参数
try:
    K, dist, R, t = read_camera_params_from_json("camera_params.json")
    print("成功从JSON文件读取相机参数")
    print(f"内参矩阵 K:\n{K}")
except FileNotFoundError:
    print("错误：找不到camera_params.json文件")
    exit()

# 读取图像
img = cv2.imread("biaodingban/1_1.bmp")
if img is None:
    print("错误：无法读取图像文件")
    exit()

# 测试像素点
pixel = (1199, 1200)
X, Y = pixel_to_world(pixel, K, R, t)
print(f"像素点 {pixel} 对应世界坐标: ({X:.3f}, {Y:.3f})")

# 计算坐标轴在像素坐标系中的位置
world_point = (0, 0)  # 原点
x_point = (50, 0)     # X轴方向
y_point = (0, 50)     # Y轴方向

org_pixel = world_to_pixel(world_point, K, R, t)
x_pixel = world_to_pixel(x_point, K, R, t)
y_pixel = world_to_pixel(y_point, K, R, t)

print(f"原点像素坐标: {org_pixel}")
print(f"X轴方向像素坐标: {x_pixel}")
print(f"Y轴方向像素坐标: {y_pixel}")

# 绘制坐标轴
img_with_axes = img.copy()
guangxing_pixel = (K[0, 2], K[1, 2])  # 光心位置
img_with_axes = draw_coordinate_axes(img_with_axes, org_pixel, x_pixel, y_pixel, guangxing_pixel)

# 使用matplotlib显示结果
plt.figure(figsize=(15, 10))

# 原始图像
plt.subplot(1, 2, 1)
img_orig_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
plt.imshow(img_orig_rgb)
plt.title('原始图像')
plt.axis('off')

# 带坐标轴的图像
plt.subplot(1, 2, 2)
img_rgb = cv2.cvtColor(img_with_axes, cv2.COLOR_BGR2RGB)
plt.imshow(img_rgb)
plt.title('世界坐标系映射到像素坐标系')
plt.axis('off')

plt.tight_layout()
plt.show()

print("坐标映射验证完成！")