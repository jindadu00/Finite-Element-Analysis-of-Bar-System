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

DOF = int(data_blocks[0])
x0 = np.array([float(i) for i in data_blocks[1].split()])
node = np.array([int(i) for i in data_blocks[2].split()])
dx = np.array([float(i) for i in data_blocks[3].split()])
x = x0+dx
tense = np.array([float(i) for i in data_blocks[4].split()])

if DOF == 3:
    fig = plt.figure(1)
    ax = fig.gca(projection='3d')

    for i in range(len(node)//2):
        node1 = node[i*2]-1
        node2 = node[i*2+1]-1

        X0 = [x0[node1*DOF], x0[node2*DOF]]
        Z0 = [x0[node1*DOF+1], x0[node2*DOF+1]]
        Y0 = [x0[node1*DOF+2], x0[node2*DOF+2]]
        f = tense[i]
        if(f >= 0):
            ax.plot(X0, Y0, Z0, color=(
                f/abs(tense).max(), 0, 0), linestyle='--')
        if(f < 0):
            ax.plot(X0, Y0, Z0, color=(
                0, 0, -f/abs(tense).max()), linestyle='--')

        X = [x[node1*DOF], x[node2*DOF]]
        Z = [x[node1*DOF+1], x[node2*DOF+1]]
        Y = [x[node1*DOF+2], x[node2*DOF+2]]
    #     颜色偏红说明是拉应力，红色越鲜艳拉应力越大，颜色越黑拉应力越小
    #     颜色偏蓝说明是压应力，蓝色越鲜艳压应力越大，颜色越黑压应力越小
        if(f >= 0):
            ax.plot(X, Y, Z, color=(f/abs(tense).max(), 0, 0), linestyle='-')
        if(f < 0):
            ax.plot(X, Y, Z, color=(0, 0, -f/abs(tense).max()), linestyle='-')
if DOF == 2:

    for i in range(len(node)//2):
        node1 = node[i*2]-1
        node2 = node[i*2+1]-1

        X0 = [x0[node1*DOF], x0[node2*DOF]]
        Y0 = [x0[node1*DOF+1], x0[node2*DOF+1]]
        f = tense[i]
        if(f >= 0):
            plt.plot(X0, Y0, color=(f/abs(tense).max(), 0, 0), linestyle='--')
        if(f < 0):
            plt.plot(X0, Y0, color=(0, 0, -f/abs(tense).max()), linestyle='--')

        X = [x[node1*DOF], x[node2*DOF]]
        Y = [x[node1*DOF+1], x[node2*DOF+1]]
    #     颜色偏红说明是拉应力，红色越鲜艳拉应力越大，颜色越黑拉应力越小
    #     颜色偏蓝说明是压应力，蓝色越鲜艳压应力越大，颜色越黑压应力越小
        if(f >= 0):
            plt.plot(X, Y, color=(f/abs(tense).max(), 0, 0), linestyle='-')
        if(f < 0):
            plt.plot(X, Y, color=(0, 0, -f/abs(tense).max()), linestyle='-')
plt.title('实线表示变形前，虚线表示变性后\n颜色偏红说明是拉应力，红色越鲜艳拉应力越大，颜色越黑拉应力越小\n颜色偏蓝说明是压应力，蓝色越鲜艳压应力越大，颜色越黑压应力越小', fontsize=10)

plt.savefig('picture.jpg')
plt.show()
