def parse_data_to_point_pairs(data):
    # 拆分数据段
    segments = data.strip().split(';')

    # 提取名称和坐标
    names = segments[0].split(',')
    x_coords = list(map(float, segments[1].split(',')))
    y_coords = list(map(float, segments[2].split(',')))

    # 处理圆心部分
    circle_segment = segments[3].split(',')
    circle_name = circle_segment[0]
    circle_x = float(circle_segment[1])
    circle_y = float(circle_segment[2])

    # 创建点对字典
    point_pairs = {}

    # 遍历名称和坐标，生成点对
    for i in range(len(names)):
        if i < len(x_coords) and i < len(y_coords):
            point_pairs[names[i]] = (x_coords[i], y_coords[i])

    # 添加圆心
    point_pairs[circle_name] = (circle_x, circle_y)

    return point_pairs

# 示例使用
data = """
banyuan,fangao,tixing,yuanao,wubianxing,liubianxing;849.430,867.610,843.698,1189.962,1190.397,1185.335;1275.068,1562.707,998.618,988.925,1555.019,1256.124;yuanxin,1023.529,1317.969
"""
point_pairs = parse_data_to_point_pairs(data)
print("点对储存结果如下：")
for name, point in point_pairs.items():
    print(f"{name}: {point}")
