import numpy as np
import cv2
import json

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

def compute_reprojection_error(src_points, dst_points, M):
    """
    计算仿射变换矩阵的重投影误差
    :param src_points: 源点集 (N, 2)
    :param dst_points: 目标点集 (N, 2)
    :param M: 2x3 仿射变换矩阵
    :return: 平均重投影误差 (mean_error), 每个点的误差列表 (errors)
    """
    # 将源点扩展为齐次坐标 (x, y, 1)
    src_homogeneous = np.hstack([src_points, np.ones((src_points.shape[0], 1))])

    # 应用仿射变换矩阵 M，计算变换后的坐标
    transformed_points = src_homogeneous @ M.T  # (N, 2)

    # 计算欧几里得距离（L2 范数）
    errors = np.linalg.norm(transformed_points - dst_points, axis=1)

    # 计算平均误差
    mean_error = np.mean(errors)
    return mean_error, errors

def read_camera_params_from_json(filename):
    """从JSON文件中读取相机参数"""
    with open(filename, 'r') as f:
        data = json.load(f)
    
    K = np.array(data["CameraMatrix"])
    dist = np.array(data["DistCoeffs"])
    R = np.array(data["Rotation"])
    t = np.array(data["TranslationVector"])
    
    return K, dist, R, t

# 从JSON文件读取相机参数
try:
    K, dist, R, t = read_camera_params_from_json("camera_params.json")
    print("成功从JSON文件读取相机参数")
except FileNotFoundError:
    print("错误：找不到camera_params.json文件，请先运行标定板标定程序")
    exit()
except Exception as e:
    print(f"读取JSON文件时出错：{e}")
    exit()

# 原始 点对集,像素
src_pts = np.array([
    [846.107, 1242.420], 
    [1838.762, 1311.622], 
    [1218.233, 1759.726]
], dtype=np.float32)
src_pts_wuli = []
for src_pt in src_pts:
    world_coord = pixel_to_world(src_pt, K, R, t)
    src_pts_wuli.append(world_coord)
src_pts_wuli = np.array(src_pts_wuli, dtype=np.float32)
print("原始点集:\n", src_pts_wuli)
# 目标点集 
dst_pts = np.array([
    [317.93, 39.64], 
    [322.92, -47.82],  
    [362.84, 5.98]
], dtype=np.float32)

# 使用 RANSAC 估计仿射变换矩阵
M, inliers = cv2.estimateAffine2D(src_pts_wuli, dst_pts, method=cv2.RANSAC, ransacReprojThreshold=0.2)

if M is None:
    print("仿射变换矩阵估计失败")
else:
    print("估计的仿射变换矩阵:\n", M)

    # 计算 RANSAC 内点数目
    inlier_count = np.sum(inliers)
    print(f"RANSAC 内点数量: {inlier_count} / {len(src_pts)}")

    # 计算重投影误差
    mean_error, errors = compute_reprojection_error(src_pts_wuli, dst_pts, M)
    print(f"平均重投影误差: {mean_error:.4f}")
    print(f"每个点的误差: {errors}")

    # 输出误差信息
    for i, err in enumerate(errors):
        status = "内点" if inliers[i] else "外点"
        print(f"点 {src_pts_wuli[i]} → {dst_pts[i]} | 误差: {err:.4f} | {status}")

    # 只保存仿射变换矩阵到JSON文件
    affine_data = {
        "AffineMatrix": M.tolist()
    }

    with open("affine_transform.json", 'w') as f:
        json.dump(affine_data, f, indent=4)
    
    print("仿射变换矩阵已保存到 affine_transform.json")