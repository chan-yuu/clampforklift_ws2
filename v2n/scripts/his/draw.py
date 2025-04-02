import matplotlib.pyplot as plt

# 数据
points = {
    'p1': (44.8154976, 85.59041701),
    'p2': (44.81551023, 85.59041764),
    'p3': (44.81550991, 85.59040767),
    'p4': (44.81550967, 85.59039761),
    'p5': (44.81550959, 85.59038759),
    'p6': (44.81550911, 85.5903774),
    'p7': (44.81549753, 85.59037659),
    'p8': (44.81549766, 85.59038676),
    'p9': (44.81549751, 85.59039686),
    'p10': (44.8154978, 85.59040699),
    'p11': (44.81552433, 85.59019088),
    'p12': (44.81552374, 85.59018085),
    'p13': (44.81555127, 85.59018155),
    'p14': (44.81555088, 85.59019158),
    'p15warehousr_ws': (44.81550795, 85.59010132),
    'p15warehousr_wn': (44.81560931, 85.5901067),
    'p15warehousr_es': (44.81549503, 85.59041719),
    'p15warehousr_en': (44.81559621, 85.59042543), 
}

# 提取数据
x = [coord[1] for coord in points.values()]
y = [coord[0] for coord in points.values()]
labels = list(points.keys())

# 绘图
plt.figure(figsize=(10, 6))
plt.scatter(x, y)

# 标注
for label, (lat, lon) in zip(labels, points.values()):
    plt.annotate(label, (lon, lat), textcoords="offset points", xytext=(0, 5), ha='center')

plt.title("pic")
plt.xlabel("lon")
plt.ylabel("lat")
plt.grid()
plt.show()
