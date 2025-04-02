import math
def calculate_distance(x1, y1, x2, y2):
    # 计算两点之间的距离
    distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return distance



x1 = 388522.91919789044
y1 = 4963434.085515765

x2 = 388548.0952617831
y2 = 4963432.193187664
print(calculate_distance(x1, y1, x2, y2))