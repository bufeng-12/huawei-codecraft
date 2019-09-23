import numpy as np
import dijkstra
import time
import buildWeight
import dijkstra_net
import floyd
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
file=open('answer2.txt','w')
file.truncate()
run_time=0
with open('./answer2.txt', 'a+') as file:
    for i in range(0,car_amount):
        weight=buildWeight.buildWeight(roadMat,maxSpeed[i])
        #print(weight.shape)

        from_ = roadMat[:, 4]
        to_ = roadMat[:, 5]
        roadId = (roadMat[:, 0])

        R=dijkstra.dijkstra(weight,from_car[i],to_car[i])
        R1=[]   #用R1存储道路编号carId
        #实现节点到道路编号的索引查找
        for m in range(0,len(R)-1):
            for l in range(0,len(roadId)):
                #if from_.tolist().index(R[m])==to_.tolist().index(R[m+1]) or from_.tolist().index(R[m+1])==to_.tolist().index(R[m]):
                if from_[l]+to_[l]== R[m]+R[m+1] and from_[l]-R[m]+to_[l]-R[m+1]==0:
                    R1.append(roadId[l])
                    break

        #print(R)
        R1.insert(0, planTime[i])
        R1.insert(0, carId[i])
        #R.insert(0,'(')
        #R.append('/n')

        file.write(str(tuple(R1)) + '\n')    #转换为tuple,这样输出就会有括号，并换行。
        run_time=run_time+1
        print('运行到',run_time)

#np.savetxt("answer.txt",  R_all)
end = time.time()
print(end-start)
#print(weight)
