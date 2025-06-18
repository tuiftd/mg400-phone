def parse_data_new(input_string):
    # 按分号分割为8行
    lines = input_string.strip().split(';')
    
    # 解析第一行：有效性和模版名字
    first_line = lines[0].split(',')
    is_valid = int(first_line[0])  # 1为有效
    phone_names = first_line[1:]  # 模版名字
    
    # 解析第二行：目标名字
    target_names = lines[1].split(',')
    
    # 解析坐标和角度数据
    phone_x_coords = [float(x) for x in lines[2].split(',')]  # 模版x坐标
    target_x_coords = [float(x) for x in lines[3].split(',')]  # 目标x坐标
    phone_y_coords = [float(y) for y in lines[4].split(',')]  # 模版y坐标
    target_y_coords = [float(y) for y in lines[5].split(',')]  # 目标y坐标
    phone_angles = [float(r) for r in lines[6].split(',')]    # 模版角度
    target_angles = [float(r) for r in lines[7].split(',')]   # 目标角度
    
    # 构建结果字典
    phone_dict = {}
    target_dict = {}
    
    if is_valid == 1:  # 只有当数据有效时才处理
        # 构建phone字典
        for i, name in enumerate(phone_names):
            if i < len(phone_x_coords) and i < len(phone_y_coords) and i < len(phone_angles):
                phone_dict[name] = (phone_x_coords[i], phone_y_coords[i], phone_angles[i])
        
        # 构建target字典
        for i, name in enumerate(target_names):
            if i < len(target_x_coords) and i < len(target_y_coords) and i < len(target_angles):
                target_dict[name] = (target_x_coords[i], target_y_coords[i], target_angles[i])
    
    return phone_dict, target_dict
if __name__ == '__main__':
# 测试数据
    input_data = "1,zhiliubianxing,fangao,zhengliubianxing,banyuan,yuanao,tixing;yuanao,banyuan,tixing,fangao,zhengliubianxing,zhiliubianxing;1076.139,1307.966,1280.342,1516.960,1456.269,1706.926;1940.941,1117.448,1641.923,1899.853,1924.425,1712.825;1048.663,828.602,1268.055,1030.210,1468.308,1231.327;274.523,333.366,262.481,543.529,409.027,365.392;138.216,137.275,137.004,135.091,138.417,137.186;101.318,-21.529,168.343,-168.117,140.152,-12.378"

    phone_dict, target_dict = parse_data_new(input_data)
    print("Phone字典:", phone_dict)
    print("Target字典:", target_dict)