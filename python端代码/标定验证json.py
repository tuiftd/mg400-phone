import numpy as np
import json

def load_camera_params():
    """从JSON文件加载相机参数"""
    try:
        with open("camera_params.json", 'r') as f:
            data = json.load(f)
        
        K = np.array(data["CameraMatrix"])
        dist = np.array(data["DistCoeffs"])
        R = np.array(data["Rotation"])
        t = np.array(data["TranslationVector"])
        
        print("成功从camera_params.json加载相机参数")
        return K, dist, R, t
    except FileNotFoundError:
        print("错误：找不到camera_params.json文件")
        return None, None, None, None
    except Exception as e:
        print(f"加载相机参数时出错：{e}")
        return None, None, None, None

def load_transform_matrix():
    """从JSON文件加载变换矩阵"""
    try:
        with open("affine_transform.json", 'r') as f:
            data = json.load(f)
        
        # 检查是否是2x3仿射矩阵
        if "AffineMatrix" in data:
            affine_matrix = np.array(data["AffineMatrix"])
            # 转换为3x3齐次矩阵
            transformation_matrix = np.vstack([affine_matrix, [0, 0, 1]])
        # 检查是否是3x3齐次矩阵
        elif "HomogeneousMatrix" in data:
            transformation_matrix = np.array(data["HomogeneousMatrix"])
        # 检查是否直接是TransformMatrix
        elif "TransformMatrix" in data:
            transformation_matrix = np.array(data["TransformMatrix"])
        else:
            print("错误：JSON文件中未找到有效的变换矩阵格式")
            return None
        
        print("成功从affine_transform.json加载变换矩阵")
        print(f"变换矩阵:\n{transformation_matrix}")
        return transformation_matrix
        
    except FileNotFoundError:
        print("错误：找不到affine_transform.json文件，使用默认矩阵")
        # 返回默认变换矩阵
        default_matrix = np.array([
            [9.99277962e-01, 3.44535836e-02, 2.91991446e+02],
            [-4.00650989e-02, 1.00289632e+00, -3.02108143e+01],
            [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
        ])
        print(f"使用默认变换矩阵:\n{default_matrix}")
        return default_matrix
    except Exception as e:
        print(f"加载变换矩阵时出错：{e}")
        return None

def save_transform_matrix(matrix, filename="affine_transform.json"):
    """保存变换矩阵到JSON文件"""
    try:
        data = {
            "HomogeneousMatrix": matrix.tolist(),
            "Description": "Camera to Robot coordinate transform matrix",
            "Format": "3x3 homogeneous transformation matrix",
            "Timestamp": str(np.datetime64('now'))
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=4)
        
        print(f"变换矩阵已保存到 {filename}")
        return True
    except Exception as e:
        print(f"保存变换矩阵时出错：{e}")
        return False

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

def transform_pixel_to_robot(pixel_coords, K, R, t, transformation_matrix):
    """
    完整的像素到机械臂坐标转换流程
    
    Args:
        pixel_coords: 像素坐标 [x, y]
        K: 相机内参矩阵
        R: 旋转矩阵
        t: 平移向量
        transformation_matrix: 变换矩阵
    
    Returns:
        机械臂坐标 [X, Y]
    """
    # 步骤1：像素坐标 -> 世界坐标
    world_coords = pixel_to_world(pixel_coords, K, R, t)
    print(f"世界坐标: ({world_coords[0]:.4f}, {world_coords[1]:.4f})")
    
    # 步骤2：世界坐标 -> 机械臂坐标
    robot_coords = verify_transformation(transformation_matrix, world_coords)
    print(f"机械臂坐标: ({robot_coords[0]:.4f}, {robot_coords[1]:.4f})")
    
    return robot_coords

def main():
    """主函数"""
    print("开始加载参数...")
    
    # 加载相机参数
    K, dist, R, t = load_camera_params()
    if K is None:
        print("相机参数加载失败，程序退出")
        return
    
    # 加载变换矩阵
    transformation_matrix = load_transform_matrix()
    if transformation_matrix is None:
        print("变换矩阵加载失败，程序退出")
        return
    
    print("\n参数加载完成！")
    print(f"相机内参矩阵 K:\n{K}")
    print(f"变换矩阵:\n{transformation_matrix}")
    
    # 交互式测试
    while True:
        try:
            print("\n" + "="*50)
            print("像素坐标到机械臂坐标转换")
            print("="*50)
            
            x_input = float(input("请输入像素 x 坐标: "))
            y_input = float(input("请输入像素 y 坐标: "))
            
            # 完整转换流程
            robot_coords = transform_pixel_to_robot(
                [x_input, y_input], K, R, t, transformation_matrix
            )
            
            print(f"\n最终结果:")
            print(f"像素坐标: ({x_input}, {y_input})")
            print(f"机械臂坐标: ({robot_coords[0]:.4f}, {robot_coords[1]:.4f})")
            
            # 询问是否继续
            continue_test = input("\n是否继续测试？(y/n): ").lower()
            if continue_test != 'y':
                break
                
        except ValueError:
            print("错误：请输入有效的数字")
        except KeyboardInterrupt:
            print("\n程序被用户中断")
            break
        except Exception as e:
            print(f"发生错误：{e}")
    
    print("程序结束")

if __name__ == "__main__":
    main()