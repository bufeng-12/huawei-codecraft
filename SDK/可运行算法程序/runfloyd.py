import numpy as np
import buildWeight
import time
import floyd
#导入数据
source = './config/road.txt'  ## 源文件路径
dest = './config/road1.txt'  ## 去除括号后的文件路径
f = open(dest, "w+")
f.truncate()
with open(source, 'r') as text:
    with open(dest, 'a+') as road_1:  ## 以追加写的方式打开目标文件
            for line in text.readlines():
                road_1.write(line.replace('(', '').replace(')', ''))  ## 去除一行的左右括号
    roadMat = np.loadtxt(dest, dtype=int, skiprows=1, delimiter=',')  ## 读取txt并转换为ndarray

source2 = './config/car.txt'  ## 源文件路径
dest2 = './config/car1.txt'  ## 去除括号后的文件路径
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
file=open('answer.txt','w')
file.truncate()
with open('./answer.txt', 'a+') as file:
    for i in range(0,car_amount):
        weight=buildWeight.buildWeight(roadMat,maxSpeed[i])
        #print(weight.shape)

        D,path=floyd.floyd(weight)
        R=floyd.router(D,path,from_car[i],to_car[i])
        R.insert(0, planTime[i])
        R.insert(0, carId[i])
        #R.insert(0,'(')
        #R.append('/n')

        file.write(str(tuple(R)) + '\n')


#np.savetxt("answer.txt",  R_all)
end = time.time()
print(end-start)
print(weight)
