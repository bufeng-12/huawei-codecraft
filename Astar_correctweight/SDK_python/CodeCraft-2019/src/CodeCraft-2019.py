# coding:utf-8
import logging
import sys

import time
import numpy as np
# import buildWeight
# import floyd
import math


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
    nodenum = max(to_)  # 此处是索引to_里的最大值来找到节点数
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
    # np.savetxt("mat_roadLength.txt", mat_roadLength)
    # np.savetxt("mat_maxSpeed.txt", mat_maxSpeed)
    # mat_maxSpeed=min(mat_maxSpeed,maxSpeed)
    timeWeight = (np.true_divide(mat_roadLength, (mat_maxSpeed + 1e-4)))
    # np.savetxt("timeWeight.txt", timeWeight)
    # print(timeWeight.shape)
    return timeWeight


def buildFWeight(roadMat, maxSpeed, freqWeight,crossIdList):
    alpha = 1  # 速度匹配的权重系数
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
    freqWeight = np.array(freqWeight)
    freqW_var = np.var(freqWeight)
    freqW_max = np.max(freqWeight)

    for i in range(0, len(from_)):
        mat_maxSpeed[crossIdList.index(from_[i])][crossIdList.index(to_[i])] = min(limitSpeed[i], maxSpeed)
        # mat_maxSpeed[to_[i] - 1][from_[i] - 1] = min(limitSpeed[i], maxSpeed)
        if maxSpeed < 4:
            if roadLength[i] < 15:
                mat_roadLength[crossIdList.index(from_[i])][crossIdList.index(to_[i])] = roadLength[i] + min(
                    freqWeight[i] / freqW_max * freqW_var * 5, roadLength[i] / 3)
                mat_roadLength[crossIdList(to_[i])][crossIdList(from_[i])] = mat_roadLength[from_[i] - 1][to_[i] - 1]
            elif roadLength[i] >= 15 and roadLength[i] < 20:
                mat_roadLength[crossIdList.index(from_[i])][crossIdList.index(to_[i])]= roadLength[i] + abs(maxSpeed - limitSpeed[i]) + min(
                    freqWeight[i] / freqW_max * freqW_var * 5, roadLength[i] / 3)
                mat_roadLength[crossIdList(to_[i])][crossIdList(from_[i])] = mat_roadLength[from_[i] - 1][to_[i] - 1]
            else:
                mat_roadLength[crossIdList.index(from_[i])][crossIdList.index(to_[i])] = roadLength[i] + alpha * abs(maxSpeed - limitSpeed[i]) + min(
                    freqWeight[i] / freqW_max * freqW_var * 5, roadLength[i] / 3)
                mat_roadLength[crossIdList(to_[i])][crossIdList(from_[i])] = mat_roadLength[from_[i] - 1][to_[i] - 1]
        elif maxSpeed >= 4 and maxSpeed < 6:
            if roadLength[i] < 10:
                mat_roadLength[crossIdList.index(from_[i])][crossIdList.index(to_[i])] = roadLength[i] + min(
                    freqWeight[i] / freqW_max * freqW_var * 5, roadLength[i] / 3)
                mat_roadLength[crossIdList(to_[i])][crossIdList(from_[i])] = mat_roadLength[from_[i] - 1][to_[i] - 1]
            else:
                mat_roadLength[crossIdList.index(from_[i])][crossIdList.index(to_[i])] = roadLength[i] + abs(maxSpeed - limitSpeed[i]) + min(
                    freqWeight[i] / freqW_max * freqW_var * 5, roadLength[i] / 3)
                mat_roadLength[crossIdList(to_[i])][crossIdList(from_[i])] = mat_roadLength[from_[i] - 1][to_[i] - 1]
        else:
            mat_roadLength[crossIdList.index(from_[i])][crossIdList.index(to_[i])] = roadLength[i] + min(freqWeight[i] / freqW_max * freqW_var * 5,
                                                                           roadLength[i] / 3)
            mat_roadLength[crossIdList(to_[i])][crossIdList(from_[i])] = mat_roadLength[from_[i] - 1][to_[i] - 1]
        if isDuplex[i] == 1:
            mat_maxSpeed[crossIdList.index(to_[i])][crossIdList.index(from_[i])] = min(limitSpeed[i], maxSpeed)

    '''
    矩阵的建立方法：length/（速度+1e-4),
     没有连通的（包括自己到自己）length为inf，单向的那么反向限制速度为0
    '''
    # np.savetxt("mat_roadLength.txt", mat_roadLength)
    # np.savetxt("mat_maxSpeed.txt", mat_maxSpeed)
    # mat_maxSpeed=min(mat_maxSpeed,maxSpeed)
    timeWeight = (np.true_divide(mat_roadLength, (mat_maxSpeed + 1e-4)))
    # np.savetxt("timeWeight.txt", timeWeight)
    # print(timeWeight.shape)
    return timeWeight


def Astar(weight_graph, startPosition, goalPosition,crossIdList):
    startPosition=crossIdList.index(startPosition)
    goalPosition=crossIdList.index(goalPosition)
    inf = 1e5
    weight_graph[:, goalPosition - 1] = weight_graph[:, goalPosition - 1] * 0.5
    # print(goalPosition)
    openList = []
    closeList = []
    openList.append([0, 0, 0])
    openListLength = 0
    closeListLength = 0
    openList[0][0] = startPosition  # 起始点装载到openList中
    openListLength = openListLength + 1  # openList中点个数加1
    wrongFlag = 1  # 最终有无找到正确结果的标志位
    while (1):
        f = openList[0][1] + openList[0][2]  # g表示现节点与起始点间的权重，h表示现节点与终止点间的权重
        nodePosition = 0  # 记录
        for i in range(0, openListLength):
            if f > openList[i][1] + openList[i][2]:  # 若openList中有节点的权重值小于现有权重值
                f = openList[i][1] + openList[i][2]
                nodePosition = i

        # 将f最小的节点放入close list
        closeListLength = closeListLength + 1
        closeList.append(openList[nodePosition])
        openListLength = 0
        # print('\n')
        # print('openList' ,openList)
        # print('closeListLenghth:',closeListLength)
        if closeList[closeListLength - 1][0] == goalPosition:  # 判断现有节点是不是终止节点
            break
        a = weight_graph[(closeList[closeListLength - 1][0]) - 1, :]
        newPosition = [newPosition + 1 for (newPosition, val) in enumerate(a) if val < inf]
        # print('newPosition',newPosition)
        # newPosition = find(weight_graph[closeList[closeListLength].cross,:] != inf);
        flag = 0
        # openList=[]
        for i in range(0, len(newPosition)):
            for j in range(0, len(closeList)):
                if newPosition[i] == closeList[j][0]:
                    flag = 1
                    break
            if flag == 1:
                flag = 0
                continue
            openListLength = openListLength + 1
            cross = newPosition[i]
            d = closeListLength
            g = 0.3 * (closeList[closeListLength - 1][1] + weight_graph[
                (closeList[closeListLength - 1][0]) - 1, newPosition[i] - 1])
            # 我擦勒！！！就是closeList的下标也不能大于length-1啊，啊啊
            # 表示现节点与起始点间的权重
            cross_width=math.sqrt(len(crossIdList))
            d1 = math.floor((goalPosition - 1) / cross_width) + 1
            d2 = (goalPosition - 1) % cross_width + 1
            d3 = math.floor((newPosition[i] - 1) / cross_width) + 1
            d4 = (newPosition[i] - 1) % cross_width + 1
            h = 0.7 * (abs(d1 - d3) + abs(d2 - d4) - 1 / weight_graph[
                newPosition[i] - 1, goalPosition - 1])  # 表示现节点与终止点间的权重
            # 注意newPosition作为矩阵下标时，也得注意减1
            if openListLength <= len(openList):
                openList[openListLength - 1] = [cross, g, h]
            else:
                openList.append([cross, g, h])
            # print('openList:')
            # print(openList)

        if openListLength == 0:
            # print('出错了！')
            for i in range(0, len(closeList) - 1):
                forwardCross = closeList[i][0]
                backwardCross = closeList[i + 1][0]
                weight_graph[forwardCross - 1, backwardCross - 1] = weight_graph[
                                                                        forwardCross - 1, backwardCross - 1] + 0.1 * \
                                                                    weight_graph[forwardCross - 1, backwardCross - 1]
                # 对于到达不了终止点的线路，对线路上的道路权重做惩罚性的增加，以降低之后选择这些道路的可能
            closeList = []
            closeListLength = 0
            openList = []
            openListLength = 0
            openList.append([0, 0, 0])
            openList[0][0] = startPosition  # 起始点装载到openList中

            openListLength = openListLength + 1  # openList中点个数加1
    roadPlan = []
    for i in range(0, len(closeList)):
        roadPlan.append(crossIdList[closeList[i][0]-1])
    return roadPlan


def main():
    #	if len(sys.argv) != 5:
    #		logging.info('please input args: car_path, road_path, cross_path, answerPath')
    #		exit(1)
    #
    # car_path = sys.argv[1]
    # road_path = sys.argv[2]
    # cross_path = sys.argv[3]
    # answer_path = sys.argv[4]
    car_path = '../1-map-exam-1/car.txt'
    road_path = '../1-map-exam-1/road.txt'
    cross_path = '../1-map-exam-1/cross.txt'
    answer_path = '../1-map-exam-1/answer.txt'

    #
    #	logging.info("car_path is %s" % (car_path))
    #	logging.info("road_path is %s" % (road_path))
    #	logging.info("cross_path is %s" % (cross_path))
    #	logging.info("answer_path is %s" % (answer_path))

    # to read input file

    source = road_path  ## 源文件路径
    dest = '../1-map-exam-1/road1.txt'  ## 去除括号后的文件路径
    f = open(dest, "w+")
    f.truncate()
    with open(source, 'r') as text:
        with open(dest, 'a+') as road_1:  ## 以追加写的方式打开目标文件
            for line in text.readlines():
                road_1.write(line.replace('(', '').replace(')', ''))  ## 去除一行的左右括号
        roadMat = np.loadtxt(dest, dtype=int, skiprows=1, delimiter=',')  ## 读取txt并转换为ndarray
    channel = roadMat[:, 3]
    roadLength = roadMat[:, 1]
    source2 = car_path  ## 源文件路径
    dest2 = '../1-map-exam-1/car1.txt'  ## 去除括号后的文件路径
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
    dest3 = '../1-map-exam-1/cross1.txt'  ## 去除括号后的文件路径
    f = open(dest3, "w+")
    f.truncate()
    with open(source3, 'r') as text:
        with open(dest3, 'a+') as cross1:  ## 以追加写的方式打开目标文件
            for line in text.readlines():
                cross1.write(line.replace('(', '').replace(')', ''))  ## 去除一行的左右括号
        crossMat = np.loadtxt(dest3,dtype=int, skiprows=1, delimiter=',')  ## 读取txt并转换为ndarray
    crossIdList = list(crossMat[:,0])
    '''
    初始化权重矩阵，并运行Astar最短路径寻找程序
    '''
    start = time.time() # 计算耗时，单位为s
    R_all = []
    # file=open('answer_astar.txt','w')
    # file.truncate()
    run_time = 0
    roadPlanall = []
    for i in range(0, car_amount):
        weight = buildWeight(roadMat, maxSpeed[i],crossIdList)
        # weight = buildWeight.buildWeight(roadMat, 2)
        # print(weight.shape)

        from_ = roadMat[:, 4]
        to_ = roadMat[:, 5]
        roadId = (roadMat[:, 0])
        # c=to_car[i]
        # print(from_car[i])

        R = Astar(weight, (from_car[i]), (to_car[i]),crossIdList)
        # R = Astar.Astar(weight, 16, 36)
        R1 = []  # 用R1存储道路编号roadId

        # 实现节点到道路编号的索引查找
        # time_cost_e=0
        # 计算时间是为了让耗时久，预计到达时间晚的车尽早发车。
        for m in range(0, len(R) - 1):
            # 找出预计耗时，即每条经过的道路的时间权重加起来
            # time_cost_e =weight[R[m]-1, R[m+1]-1]+time_cost_e
            for l in range(0, len(roadId)):
                # if from_.tolist().index(R[m])==to_.tolist().index(R[m+1]) or from_.tolist().index(R[m+1])==to_.tolist().index(R[m]):
                if from_[l] + to_[l] ==R[m]+ R[m + 1] and from_[l] - R[m]- + to_[l] - R[m + 1]-1 == 0:
                    R1.append(roadId[l])
                    break

        print(i)
        roadPlanall.append((R1))

    ###前面是第一次处理，下述是第二次处理
    frequency = []
    freqWeight = []
    roadPlanall = [j for i in roadPlanall for j in i]
    # roadPlanall= np.array(roadPlanall)
    for i in range(0, len(roadId)):
        # frequency=roadPlanall.count(roadId[i])
        frequency.append(roadPlanall.count(roadId[i]))
        freqWeight.append(frequency[i] / channel[i] / roadLength[i])
    # print(frequency)
    # freqWeight=np.array(freqWeight)
    # freqW_var = np.var(freqWeight)
    # freqW_max=np.max(freqWeight)
    arrival = []

    for i in range(0, car_amount):
        weight = buildFWeight(roadMat, maxSpeed[i], freqWeight,crossIdList)
        from_ = roadMat[:, 4]
        to_ = roadMat[:, 5]
        roadId = (roadMat[:, 0])
        # c = to_car[i]
        # print(from_car[i])

        R = Astar(weight, from_car[i], to_car[i],crossIdList)
        # print('第二次执行到：', i)
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
                if from_[l] + to_[l] == crossIdList[R[m]-1] + crossIdList[R[m + 1]-1] and from_[l] - crossIdList[R[m]-1] + to_[l] - crossIdList[R[m + 1]-1] == 0:
                    R1.append(roadId[l])
                    break
        R_all.append((R1))
        print(i)

        arrival.append(time_cost_e + planTime[i])

    # 对planTime通过发车时间等进行限制
    arrival_max = int(max(arrival))
    for i in range(0, car_amount):
        if time_cost_e + planTime[i] > math.floor(0.8 * arrival_max) or planTime[i]<3:
            planTime[i] = planTime[i]
        else:
            planTime[i] = planTime[i] + np.random.random_integers(1, arrival_max)
    maxcar = 2*len(roadId)   #
    tmin = math.floor(arrival_max * 6)  # 预计首车到达时间取值为预计最晚到达时间两倍
    planTimeR = np.arange(car_amount)
    for count in range(1, int(tmin)):
        threshold = (maxcar + (2 * maxcar - car_amount / tmin) / arrival_max * count) if count < arrival_max else car_amount / tmin
        start_quantity = 0
        for i in range(car_amount):
            if planTime[i] == count:
                start_quantity = start_quantity + 1
                planTimeR[i] = count
            else:
                planTimeR[i] = min(planTime[i],count+1)
            if start_quantity > threshold:
                break
    # 存入数据，以标准格式
    file = open(answer_path, 'w')
    file.truncate()
    R1 = []
    with open(answer_path, 'a+') as file:
        for i in range(0, car_amount):
            R1 = R_all[i]
            R1.insert(0, planTimeR[i])
            R1.insert(0, carId[i])

            file.write(str(tuple(R1)) + '\n')  # 转换为tuple,这样输出就会有括号，并换行。

    end = time.time()
    #print(end - start)


if __name__ == "__main__":
    main()
