import numpy as np
import dijkstra
import time
import buildWeight
import Astar
import math
#导入数据
source = './1-map-training-1/road.txt'  ## 源文件路径
#source = './config/road.txt'  ## 源文件路径
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
初始化权重矩阵，并运行Astar最短路径寻找程序
'''
print(from_car)
start = time.time() # 计算耗时，单位为s
R_all=[]
file=open('answer_astar.txt','w')
file.truncate()
run_time=0
with open('./answer_astar.txt', 'a+') as file:
    for i in range(0,car_amount):
        weight=buildWeight.buildWeight(roadMat,maxSpeed[i])
        #weight = buildWeight.buildWeight(roadMat, 2)
        #print(weight.shape)

        from_ = roadMat[:, 4]
        to_ = roadMat[:, 5]
        roadId = (roadMat[:, 0])
        c=to_car[i]
        print(from_car[i])

        R=Astar.Astar(weight,from_car[i],to_car[i])
        #R = Astar.Astar(weight, 16, 36)
        R1=[]   #用R1存储道路编号carId
        #实现节点到道路编号的索引查找
        time_cost_e=0
        #计算时间是为了让耗时久，预计到达时间晚的车尽早发车。
        for m in range(0,len(R)-1):
            #找出预计耗时，即每条经过的道路的时间权重加起来
            time_cost_e =weight[R[m]-1, R[m+1]-1]+time_cost_e
            for l in range(0,len(roadId)):
                #if from_.tolist().index(R[m])==to_.tolist().index(R[m+1]) or from_.tolist().index(R[m+1])==to_.tolist().index(R[m]):
                if from_[l]+to_[l]== R[m]+R[m+1] and from_[l]-R[m]+to_[l]-R[m+1]==0:
                    R1.append(roadId[l])
                    break
        if time_cost_e+planTime[i]>math.floor(0.8*1000):
            planTime1=planTime[i]
        else:
            planTime1=planTime[i]+np.random.random_integers(1,800)
        #print(R)
        R1.insert(0, planTime1)
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
