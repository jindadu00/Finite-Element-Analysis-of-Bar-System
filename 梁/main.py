import os
import numpy as np
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif'] = ['SimHei']  # 用来正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False  # 用来正常显示负号

main = "Finite_Element_Analysis_of_Bar_System.exe"
os.system(main)

with open("output/画图数据.txt", "r") as f:  # 打开文件
    data = f.read().strip()  # 读取文件

data_blocks = data.split('\n')

x0 = np.array([float(i) for i in data_blocks[0].split()])
node = np.array([int(i) for i in data_blocks[1].split()])
dx = np.array([float(i) for i in data_blocks[2].split()])
x = x0+dx

DOF=2

for i in range(len(node)//2):
    node1 = node[i*2]-1
    node2 = node[i*2+1]-1

    X0 = [x0[node1*DOF], x0[node2*DOF]]
    Y0 = [x0[node1*DOF+1], x0[node2*DOF+1]]
    plt.plot(X0, Y0, linestyle='--')


    X = [x[node1*DOF], x[node2*DOF]]
    Y = [x[node1*DOF+1], x[node2*DOF+1]]
    plt.plot(X, Y, linestyle='-')


plt.title('实线表示变形前，虚线表示变性后', fontsize=10)

plt.savefig('picture.jpg')
plt.show()
