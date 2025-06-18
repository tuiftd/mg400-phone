import cv2
import numpy as np

def guiyihua(pixel, K):
    """归一化像素坐标"""

    # 计算归一化相机坐标
    normalized_pixel = (pixel - K[:2, 2]) / [K[0, 0], K[1, 1]]
   
    return normalized_pixel[:2] 
def niguiyihua(normalized_pixel, K):
    """反归一化像素坐标"""
    pixel=normalized_pixel*np.array([K[0,0],K[1,1]])+K[:2,2]
    return pixel
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
    R_inv = np.linalg.inv(R)
    r1 = R[:, 0].reshape(3, 1)
    r2 = R[:, 1].reshape(3, 1)

    # 计算世界坐标
    H = np.hstack([r1, r2, t])  # 构造 3×3 变换矩阵 H
    H_inv = np.linalg.inv(H)  # 计算 H 的逆

    world_point = np.dot(H_inv, normalized_pixel)  # 变换到世界坐标
    X, Y = world_point[:2].flatten() / world_point[2, 0]  # 归一化

    return X, Y

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

# 现在你可以使用这些参数进行进一步的处理，例如将像素点转换为世界坐标
img=cv2.imread("biaodingban//1_2.bmp")  # 读取图像
pixel = (1199,1200)  # 示例像素点
X, Y = pixel_to_world(pixel, K, R, t)
print(f"世界坐标: ({X}, {Y})")
# 计算像素坐标  
world_point = (0, 0)
y_point = (0,50)
x_point = (50,0)
org_pixel = world_to_pixel(world_point, K, R, t)
print(f"像素坐标: {org_pixel}")
x_pixel = world_to_pixel(x_point, K, R, t)
y_pixel = world_to_pixel(y_point, K, R, t)
print(f"x方向像素坐标: {x_pixel}")
print(f"y方向像素坐标: {y_pixel}")
print(f"K: {K}")
# cv2.circle(img, pixel, 2, (0, 0, 255), -1)  # 在图像上绘制一个圆
cv2.circle(img,(int(K[0,2]),int(K[1,2])),10, (0, 0, 255), -1)
cv2.arrowedLine(img, org_pixel, x_pixel, (0, 255, 0), 2)  # 在图像上绘制一条箭头
cv2.arrowedLine(img, org_pixel, y_pixel, (0, 255, 0), 2)  # 在图像上绘制一条箭头
font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 1
font_color = (255, 255, 255)  # 白色
line_type = 2
x_position = (x_pixel[0] + 10, x_pixel[1] + 20)
y_position = (y_pixel[0] + 10, y_pixel[1] + 20)
cv2.putText(img, "x", x_position, font, font_scale, font_color, line_type)  # 在图像上绘制文字
cv2.putText(img, "y", y_position, font, font_scale, font_color, line_type)  # 在图像上绘制文字
cv2.imshow("image", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
x_pixel= guiyihua(x_pixel, K)
y_pixel= guiyihua(y_pixel, K)
org_pixel= guiyihua(org_pixel, K)
print(x_pixel)
print(y_pixel)
print(org_pixel)
#畸变校正
img_1 = cv2.imread("biaodingban//1_2.bmp")  # 读取图像
guangxing_pixel=np.array([[int(K[0,2]),int(K[1,2])]])
undistorted_img = cv2.undistort(img_1, K, dist)
x_pixel = cv2.undistortPoints(np.array([x_pixel], dtype=np.float32), K, dist)[0][0]
y_pixel = cv2.undistortPoints(np.array([y_pixel], dtype=np.float32), K, dist)[0][0]
org_pixel = cv2.undistortPoints(np.array([org_pixel], dtype=np.float32), K, dist)[0][0]
guangxing_pixel = cv2.undistortPoints(np.array([guangxing_pixel], dtype=np.float32), K, dist)[0][0]
x_pixel= niguiyihua(x_pixel, K)
y_pixel= niguiyihua(y_pixel, K)
org_pixel= niguiyihua(org_pixel, K)
guangxing_pixel= niguiyihua(guangxing_pixel, K)
print(x_pixel)
print(y_pixel)
print(org_pixel)
print(guangxing_pixel)
cv2.circle(undistorted_img, (int(guangxing_pixel[0]),int(guangxing_pixel[1])), 10, (0, 0, 255), -1)
cv2.arrowedLine(undistorted_img, (int(org_pixel[0]),int(org_pixel[1])), (int(x_pixel[0]),int(x_pixel[1])), (0, 255, 0), 2)  # 在图像上绘制一条箭头
cv2.arrowedLine(undistorted_img, (int(org_pixel[0]),int(org_pixel[1])), (int(y_pixel[0]),int(y_pixel[1])), (0, 255, 0), 2)  # 在图像上绘制一条箭头
font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 1
font_color = (255, 255, 255)  # 白色
line_type = 2
x_position = (int(x_pixel[0]) + 10, int(x_pixel[1]) + 20)
y_position = (int(y_pixel[0]) + 10, int(y_pixel[1]) + 20)
cv2.putText(undistorted_img, "x", x_position, font, font_scale, font_color, line_type)  # 在图像上绘制文字
cv2.putText(undistorted_img, "y", y_position, font, font_scale, font_color, line_type)  # 在图像上绘制文字
# 显示对比图像
comparison_img = cv2.hconcat([img, undistorted_img])
cv2.imshow("Comparison", comparison_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
h,w = img.shape[:2]
new_K ,roi = cv2.getOptimalNewCameraMatrix(K, dist, (w,h), 1, (w,h))
print(new_K)
undistorted_img = cv2.undistort(img_1, K, dist, None, new_K)
cv2.imshow("Comparison", undistorted_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
comparison_img = cv2.hconcat([img_1, undistorted_img])
cv2.imshow("Comparison", comparison_img)
cv2.waitKey(0)
cv2.destroyAllWindows()

