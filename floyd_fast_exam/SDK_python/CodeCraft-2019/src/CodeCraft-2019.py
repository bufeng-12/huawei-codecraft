# coding:utf-8
import logging
import sys
import random
# import time
import numpy as np
# import buildWeight
# import floyd
import math

import time

# logging.basicConfig(level=logging.DEBUG,
#                    filename='../logs/CodeCraft-2019.log',
#                    format='[%(asctime)s] %(levelname)s [%(funcName)s: %(filename)s, %(lineno)d] %(message)s',
#                    datefmt='%Y-%m-%d %H:%M:%S',
#                    filemode='a')
inf = 1e5
def buildWeight(roadMat, maxSpeed,crossIdList):
    alpha = 1  # 速度匹配权重参数
    roadMat = roadMat.astype(np.int16)
    # print(roadMat.dtype)

    from_ = roadMat[:, 4]
    to_ = roadMat[:, 5]
    isDuplex = roadMat[:, 6]
    roadLength = (roadMat[:, 1])
    roadId = (roadMat[:, 0])
    limitSpeed = (roadMat[:, 2])
    nodenum = len(crossIdList)  # 此处是索引to_里的最大值来找到节点数
    # print(nodenum)
    mat_roadLength = float('inf') * np.ones(shape=(nodenum, nodenum))  # 不通的道路长度无穷大
    mat_maxSpeed = np.zeros(shape=(nodenum, nodenum))
    # maxSpeed=maxSpeed*np.ones(shape=(len(from_),len(from_)))
    # 构造路长与限速矩阵
    # print(len(from_))
    for i in range(0, len(from_)):   #by the roadnumber
        c=crossIdList.index(from_[i])
        mat_roadLength[crossIdList.index(from_[i])][crossIdList.index(to_[i])] = roadLength[i] + abs(maxSpeed - limitSpeed[i])
        mat_roadLength[crossIdList.index(to_[i])][crossIdList.index(from_[i])] = roadLength[i] + abs(maxSpeed - limitSpeed[i])

        mat_maxSpeed[crossIdList.index(from_[i])][crossIdList.index(to_[i])] = min(limitSpeed[i], maxSpeed)
        if isDuplex[i] == 1:
            mat_maxSpeed[crossIdList.index(to_[i])][crossIdList.index(from_[i])] = min(limitSpeed[i], maxSpeed)
    #
    # if maxSpeed < 4:
    #     if roadLength[i] < 15:
    #         mat_roadLength[from_[i] - 1][to_[i] - 1] = roadLength[i]
    #         mat_roadLength[to_[i] - 1][from_[i] - 1] = mat_roadLength[from_[i] - 1][to_[i] - 1]
    #     elif roadLength[i] >= 15 and roadLength[i] < 20:
    #         mat_roadLength[from_[i] - 1][to_[i] - 1] = roadLength[i] + abs(maxSpeed - limitSpeed[i])
    #         mat_roadLength[to_[i] - 1][from_[i] - 1] = mat_roadLength[from_[i] - 1][to_[i] - 1]
    #     else:
    #         mat_roadLength[from_[i] - 1][to_[i] - 1] = roadLength[i] + alpha * abs(maxSpeed - limitSpeed[i])
    #	      mat_roadLength[to_[i] - 1][from_[i] - 1] = mat_roadLength[from_[i] - 1][to_[i] - 1]
    # elif maxSpeed >= 4 and maxSpeed < 6:
    #     if roadLength[i] < 10:
    #         mat_roadLength[from_[i] - 1][to_[i] - 1] = roadLength[i]
    #         mat_roadLength[to_[i] - 1][from_[i] - 1] = mat_roadLength[from_[i] - 1][to_[i] - 1]
    #     else:
    #         mat_roadLength[from_[i] - 1][to_[i] - 1] = roadLength[i] + abs(maxSpeed - limitSpeed[i])
    #         mat_roadLength[to_[i] - 1][from_[i] - 1] = mat_roadLength[from_[i] - 1][to_[i] - 1]
    # else:
    #     mat_roadLength[from_[i] - 1][to_[i] - 1] = roadLength[i]
    #     mat_roadLength[to_[i] - 1][from_[i] - 1] = mat_roadLength[from_[i] - 1][to_[i] - 1]
    # print(mat_roadLength)
    # print(mat_maxSpeed)
    '''
    矩阵的建立方法：length/（速度+1e-4),
     没有连通的（包括自己到自己）length为inf，单向的那么反向限制速度为0
    '''
    timeWeight = (np.true_divide(mat_roadLength, (mat_maxSpeed + 1e-4)))
    return timeWeight


def floyd(weight):
    #print(weight)
    #print("数据已经导入")
    n=(weight.shape)[0]   #导入矩阵的维度
    D=weight
    path=np.zeros(shape=(n,n))
    for i in range(0,n):
        for j in range(0,n):
            if D[i,j]!=inf:
                path[i,j]=j+1  #path不能从0开始，需要从1 开始
    for k in range(0,n):
        for i in range(0,n):
            for j in range(0,n):
                if D[i,k]+D[k,j]<D[i,j]:
                    D[i,j]=D[i,k]+D[k,j]
                    path[i,j]=path[i,k]
    return D,path
def router(D, path, s, t,crossIdList):  #input is crossId
    s=crossIdList.index(s)+1
    t=crossIdList.index(t)+1
    path=path.astype(np.int)
    #print('path的类型')
    #print(path)
    s = int(s)
    t = int(t)
    L=[]
    R=[]
    R.append(s)
    #R=s
    while(1):
        if s==t:
            L.reverse()
            L=[0,L]
            break
        #L.append(D[s-1,t-1])
        #print(m.dtype)
        R.append(path[s-1, t-1])
        s=path[s-1, t-1]
        #print(R)
        #print('s',s)
        if R==[]:
            print('no access to terminal')
    for i in range(0,len(R)):
        R[i]=crossIdList[R[i]-1]
    return R         #L为长度,R为路由


# weight=buildWeight.buildWeight(5)
# print(weight.shape)
# start = time.time()
# D,path=floyd(weight)
# R=router(D,path,1,20)
# end = time.time()
# print(R, (end - start))


def main():
    #	if len(sys.argv) != 5:
    #		logging.info('please input args: car_path, road_path, cross_path, answerPath')
    #		exit(1)
    # #
    # car_path = '../config/car.txt'
    # road_path = '../config/road.txt'
    # cross_path = '../config/cross.txt'
    # answer_path = '../config/answer.txt'
    car_path = sys.argv[1]
    road_path = sys.argv[2]
    cross_path = sys.argv[3]
    answer_path = sys.argv[4]
    #
    #	logging.info("car_path is %s" % (car_path))
    #	logging.info("road_path is %s" % (road_path))
    #	logging.info("cross_path is %s" % (cross_path))
    #	logging.info("answer_path is %s" % (answer_path))

    # to read input file
    source = road_path  ## 源文件路径
    dest = 'config/road1.txt'  ## 去除括号后的文件路径
    f = open(dest, "w+")
    f.truncate()
    with open(source, 'r') as text:
        with open(dest, 'a+') as road_1:  ## 以追加写的方式打开目标文件
            for line in text.readlines():
                road_1.write(line.replace('(', '').replace(')', ''))  ## 去除一行的左右括号
        roadMat = np.loadtxt(dest, dtype=int, skiprows=1, delimiter=',')  ## 读取txt并转换为ndarray

    source2 = car_path  ## 源文件路径
    dest2 = 'config/car1.txt'  ## 去除括号后的文件路径
    f = open(dest2, "w+")
    f.truncate()
    with open(source2, 'r') as text:
        with open(dest2, 'a+') as car_1:  ## 以追加写的方式打开目标文件
            for line in text.readlines():
                car_1.write(line.replace('(', '').replace(')', ''))  ## 去除一行的左右括号
        carMat = np.loadtxt(dest2, dtype=int, skiprows=1, delimiter=',')  ## 读取txt并转换为ndarray
    maxSpeed = carMat[:, 3]
    from_car = carMat[:, 1]
    to_car = carMat[:, 2]
    planTime = carMat[:, 4]
    carId = carMat[:, 0]
    car_amount = len(from_car)

    source3 = cross_path  ## 源文件路径
    dest3 = 'config/cross1.txt'  ## 去除括号后的文件路径
    f = open(dest3, "w+")
    f.truncate()
    with open(source3, 'r') as text:
        with open(dest3, 'a+') as cross1:  ## 以追加写的方式打开目标文件
            for line in text.readlines():
                cross1.write(line.replace('(', '').replace(')', ''))  ## 去除一行的左右括号
        crossMat = np.loadtxt(dest3, dtype=int, skiprows=1, delimiter=',')  ## 读取txt并转换为ndarray
    crossIdList = list(crossMat[:, 0])
    '''
    初始化权重矩阵，并运行Astar最短路径寻找程序
    '''
    start = time.time()  # 计算耗时，单位为s
    R_all = []
    planTime1 = 0
    from_ = roadMat[:, 4]
    to_ = roadMat[:, 5]
    roadId = (roadMat[:, 0])
    arrival = []

    weight = buildWeight(roadMat, 1e5, crossIdList)
    # print(weight.shape)
    D, path = floyd(weight)

    for i in range(0, car_amount):

        #print(i)

        R = router(D, path, from_car[i], to_car[i], crossIdList)  # input is crossId

        R1 = []  # 用R1存储道路编号carId
        time_cost_e = 0
        # 实现节点到道路编号的索引查找
        #weight =buildWeight(roadMat, maxSpeed[i], crossIdList)
        for m in range(0, len(R) - 1):
            time_cost_e = weight[crossIdList.index(R[m]), crossIdList.index(R[m + 1])] + time_cost_e
            for l in range(0, len(roadId)):
                # if from_.tolist().index(R[m])==to_.tolist().index(R[m+1]) or from_.tolist().index(R[m+1])==to_.tolist().index(R[m]):
                if from_[l] + to_[l] == R[m] + R[m + 1] and abs(from_[l]-to_[l])== abs(R[m]-R[m+1]):
                    R1.append(roadId[l])
                    break
        R_all.append(R1)
        arrivalTime = planTime[i] + time_cost_e
        arrival.append(arrivalTime)


    # 对planTime通过发车时间等进行限制
    arrival_max = int(max(arrival))
    for i in range(0, car_amount):
        # if time_cost_e + planTime[i] > math.floor(0.8 * arrival_max) or plantime[i] == 1:
        #     planTime[i] = planTime[i]+ random.randint(1,10000)
        # else:
        #     planTime[i] = planTime[i] + min(5*random.randint(1,10000),9000)
        planTime[i] = planTime[i] + int(4500/car_amount*i)
        #planTime[i] = planTime[i] + min(5 * random.randint(1, 10000), 9000)
        #print(planTime[i])

    # carIsRuning=0
    # countCollect=[]
    # carDepatureList=[]
    # t=0
    # while carIsRuning< len(carId):
    #     t=t+1
    #     for i in range(0,len(carId)):
    #         if planTime[i]=t:
    #           carDepatureList.append(carId[i])
    #     count=0
    #     threshold = (maxcar + (2 * maxcar - car_amount / tmin) / arrival_max * count) if count < arrival_max else car_amount / tmin
    #     while (count<threshold and len(carDepatureList)):
    #         count=count+1
    #         car
    #


    # maxcar = 2 * len(roadId)  #
    # tmin = math.floor(1000)  # 预计首车到达时间取值为预计最晚到达时间两倍
    # planTimeR = planTime
    #
    # for count in range(1, int(tmin)):
    #     kk=[]
    #     threshold = (maxcar + (
    #             2 * maxcar - car_amount / tmin) / arrival_max * count) if count < arrival_max else car_amount / tmin
    #     start_quantity = 0
    #     for i in range(car_amount):
    #         if planTime[i] == count:
    #             kk.append(i)
    #         if len(kk)>threshold:
    #             break
    #
    #             start_quantity = start_quantity + 1
    #             planTimeR[i] = count
    #         # elif planTime[i]<count:
    #         elif planTime[i]<count:
    #             planTimeR[i]=count+1
    #         else:
    #             continue
    #         #      planTime[i] =count
    #         # else:
    #         #     continue
    #
    #         if start_quantity > threshold:
    #             break
    # 存入数据，以标准格式
    file = open(answer_path, 'w')
    file.truncate()
    R1 = []
    with open(answer_path, 'a+') as file:
        for i in range(0, car_amount):
            R1 = R_all[i]
            R1.insert(0, planTime[i])
            R1.insert(0, carId[i])

            file.write(str(tuple(R1)) + '\n')  # 转换为tuple,这样输出就会有括号，并换行。

    # np.savetxt("answer.txt",  R_all)
    end = time.time()
    #print(end - start)
    # floyd可能找不出所有解


if __name__ == "__main__":
    main()
