import numpy as np
import buildWeight
import time
import floyd
import random
import sys
#导入数据
source = './1-map-training-1/road.txt'  ## 源文件路径
dest = './1-map-training-1/road1.txt'  ## 去除括号后的文件路径
f = open(dest, "w+")
f.truncate()
with open(source, 'r') as text:
    with open(dest, 'a+') as road_1:  ## 以追加写的方式打开目标文件
            for line in text.readlines():
                road_1.write(line.replace('(', '').replace(')', ''))  ## 去除一行的左右括号
    roadMat = np.loadtxt(dest, dtype=int, skiprows=1, delimiter=',')  ## 读取txt并转换为ndarray

source2 = './1-map-training-1/car.txt'  ## 源文件路径
dest2 = './1-map-training-1/car1.txt'  ## 去除括号后的文件路径
f = open(dest2, "w+")
f.truncate()
with open(source2, 'r') as text:
    with open(dest2, 'a+') as car_1:  ## 以追加写的方式打开目标文件
            for line in text.readlines():
                car_1.write(line.replace('(', '').replace(')', ''))  ## 去除一行的左右括号
    carMat = np.loadtxt(dest2, dtype=int, skiprows=1, delimiter=',')  ## 读取txt并转换为ndarray
maxSpeed=carMat[:,3]
from_car=carMat[:,1]
to_car=carMat[:,2]
planTime=carMat[:,4]
carId=carMat[:,0]
car_amount=len(from_car)
'''
初始化权重矩阵，并运行floyd最短路径寻找程序
'''
start = time.time() # 计算耗时，单位为s
R_all=[]
planTime1=0
from_ = roadMat[:, 4]
to_ = roadMat[:, 5]
roadId = (roadMat[:, 0])
with open('./config/answer.txt', 'a+') as file:
    file=open('./config/answer.txt','w')
    file.truncate()

    weight = buildWeight.buildWeight(roadMat, 1e5)
    # print(weight.shape)
    D, path = floyd.floyd(weight)

    for i in range(0,car_amount):

        R=floyd.router(D,path,from_car[i],to_car[i])

        R1 = []  # 用R1存储道路编号carId
        # 实现节点到道路编号的索引查找
        for m in range(0, len(R) - 1):
            for l in range(0, len(roadId)):
                #if from_.tolist().index(R[m])==to_.tolist().index(R[m+1]) or from_.tolist().index(R[m+1])==to_.tolist().index(R[m]):
                if from_[l] + to_[l] == R[m] + R[m + 1] and from_[l] - R[m] + to_[l] - R[m + 1] == 0:
                    R1.append(roadId[l])
                    break
        if planTime[i]<3:
            planTime1=planTime[i]
        else:
            planTime1=planTime[i]+int(30*random.random())
        R1.insert(0, planTime1)
        R1.insert(0, carId[i])
        #R.insert(0,'(')
        #R.append('/n')

        file.write(str(tuple(R1)) + '\n')

print(car_amount)
print(i)

#np.savetxt("answer.txt",  R_all)
end = time.time()
print(end-start)

# floyd可能找不出所有解