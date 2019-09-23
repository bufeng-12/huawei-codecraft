# coding:utf-8
import logging
import sys

# import time
import numpy as np
# import buildWeight
# import floyd
import math

# import time

# logging.basicConfig(level=logging.DEBUG,
#                    filename='../logs/CodeCraft-2019.log',
#                    format='[%(asctime)s] %(levelname)s [%(funcName)s: %(filename)s, %(lineno)d] %(message)s',
#                    datefmt='%Y-%m-%d %H:%M:%S',
#                    filemode='a')
inf = 1e5


def buildWeight(roadMat, maxSpeed):
    # source = './config/road.txt'  ## 源文件路径
    # dest = './config/road1.txt'  ## 去除括号后的文件路径
    # f = open(dest, "r+")
    # f.truncate()
    # with open(source, 'r') as text:
    #     with open(dest, 'a+') as road_1:  ## 以追加写的方式打开目标文件
    #         for line in text.readlines():
    #             road_1.write(line.replace('(', '').replace(')', ''))  ## 去除一行的左右括号
    # roadMat = np.loadtxt(dest, dtype=int, skiprows=1, delimiter=',')  ## 读取txt并转换为ndarray
    #
    # # returnMat=(np.loadtxt('./config/road1.txt', skiprows=1,delimiter=',') )
    # print(roadMat)
    roadMat = roadMat.astype(np.int16)
    # print(roadMat.dtype)

    from_ = roadMat[:, 4]
    to_ = roadMat[:, 5]
    isDuplex = roadMat[:, 6]
    roadLength = (roadMat[:, 1])
    roadId = (roadMat[:, 0])
    limitSpeed = (roadMat[:, 2])
    nodenum = max(to_)  # 此处是索引to_里的最大值来找到节点数
    # print(nodenum)
    mat_roadLength = float('inf') * np.ones(shape=(nodenum, nodenum))  # 不通的道路长度无穷大
    mat_maxSpeed = np.zeros(shape=(nodenum, nodenum))
    # maxSpeed=maxSpeed*np.ones(shape=(len(from_),len(from_)))
    # 构造路长与限速矩阵
    # print(len(from_))
    for i in range(0, len(from_)):
        mat_roadLength[from_[i] - 1][to_[i] - 1] = roadLength[i]
        mat_roadLength[to_[i] - 1][from_[i] - 1] = roadLength[i]

        mat_maxSpeed[from_[i] - 1][to_[i] - 1] = min(limitSpeed[i], maxSpeed)
        mat_maxSpeed[to_[i] - 1][from_[i] - 1] = min(limitSpeed[i], maxSpeed)
        if isDuplex[i] == 0:  # 当非双向的时候，limitSpeed=0
            mat_maxSpeed[to_[i] - 1][from_[i] - 1] = 0
    # print(mat_roadLength)
    # print(mat_maxSpeed)
    '''
    矩阵的建立方法：length/（速度+1e-4),
     没有连通的（包括自己到自己）length为inf，单向的那么反向限制速度为0
    '''
    # np.savetxt("mat_roadLength.txt", mat_roadLength)
    # np.savetxt("mat_maxSpeed.txt", mat_maxSpeed)
    # mat_maxSpeed=min(mat_maxSpeed,maxSpeed)
    timeWeight = (np.true_divide(mat_roadLength, (mat_maxSpeed + 1e-4)))
    # np.savetxt("timeWeight.txt",  timeWeight)
    # print(timeWeight.shape)
    return timeWeight
def dijkstra(weight,start,terminal):
    n=(weight.shape)[0]   #导入矩阵的维度
    #print(n)
    #print('n is')
    label=np.empty(n)
    f=np.zeros(n)
    #f=int(f)
    path =[]
    # path=path.astype(np.int)
    label[start-1]=0
    f[start-1]=start
    for i in range(0,n):
        if (i+1) != start:
            label[i]=inf #标记start的label
    s=[]
    s.append(start)
    u=start

    while len(s)<n:
        for i in range(0,n):
            ins=0
            for j in range(0,len(s)):
                if i==s[j]:   #s[j]下标
                    ins=1
            if ins==0:
                v=i
                if label[v]>(label[u-1]+weight[u-1,v]):
                    label[v] = (label[u-1] + weight[u-1, v])
                    f[v]=u#涉及到u=start，u又作为数组下标，应该减一
        v1=0
        k=inf
        # print(s)
        for i in range(0,n):
            ins=0
            for j in range(0,len(s)):
                if i==s[j]:
                    ins=1
            if ins==0:
                v=i
                if k>label[v]:
                    k=label[v]
                    v1=v
        s.append(v1)
        u=v1+1
    mini=label[terminal-1]
    path.append(terminal)
    #path = path.astype(np.int)
    #print('path is')
    #print(path)
    #print('/n')
    #print('s is')
    #print (s)
    i=0
    while path[i] != start:
        path[i]=int(path[i])
        path.append(f[path[i]-1])  #神来之笔，对应f[0]存在，而path=0不存在，所以减1
        i=i+1
    path.pop()
    path.append(start)
    path.reverse()
    return path





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
    #
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
    '''
    初始化权重矩阵，并运行Astar最短路径寻找程序
    '''
    # start = time.time() # 计算耗时，单位为s
    R_all = []
    file = open(answer_path, 'w')


    file.truncate()
    run_time = 0
    with open(answer_path, 'a+') as file:
        for i in range(0, car_amount):
            weight = buildWeight(roadMat, maxSpeed[i])
            # weight = buildWeight.buildWeight(roadMat, 2)
            # print(weight.shape)

            from_ = roadMat[:, 4]
            to_ = roadMat[:, 5]
            roadId = (roadMat[:, 0])
            c = to_car[i]
            # print(from_car[i])

            R = dijkstra(weight, from_car[i], to_car[i])
            # R = Astar.Astar(weight, 16, 36)
            R1 = []  # 用R1存储道路编号carId
            # 实现节点到道路编号的索引查找
            time_cost_e = 0
            # 计算时间是为了让耗时久，预计到达时间晚的车尽早发车。
            for m in range(0, len(R) - 1):
                # 找出预计耗时，即每条经过的道路的时间权重加起来
                time_cost_e = weight[R[m] - 1, R[m + 1] - 1] + time_cost_e
                for l in range(0, len(roadId)):
                    # if from_.tolist().index(R[m])==to_.tolist().index(R[m+1]) or from_.tolist().index(R[m+1])==to_.tolist().index(R[m]):
                    if from_[l] + to_[l] == R[m] + R[m + 1] and from_[l] - R[m] + to_[l] - R[m + 1] == 0:
                        R1.append(roadId[l])
                        break
            if time_cost_e + planTime[i] > math.floor(0.5 * 800):
                planTime1 = planTime[i]
            else:
                planTime1 = planTime[i] + np.random.random_integers(1, 700-time_cost_e)
            # print(R)
            R1.insert(0, planTime1)
            R1.insert(0, carId[i])
            # R.insert(0,'(')
            # R.append('/n')

            file.write(str(tuple(R1)) + '\n')  # 转换为tuple,这样输出就会有括号，并换行。
    # run_time=run_time+1
    # print('运行到',run_time)

    # np.savetxt("answer.txt",  R_all)
    # end = time.time()
    # print(end-start)


if __name__ == "__main__":
    main()
