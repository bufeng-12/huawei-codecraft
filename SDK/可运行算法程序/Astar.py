import numpy as np
import math
def Astar( weight_graph, startPosition, goalPosition):
    inf=1e5
    weight_graph[:,goalPosition-1]=weight_graph[:,goalPosition-1]*0.5
    #print(goalPosition)
    openList=[]
    closeList=[]
    openList.append([0,0,0])
    openListLength = 0
    closeListLength = 0
    openList[0][0] = startPosition  # 起始点装载到openList中
    openListLength = openListLength + 1  # openList中点个数加1
    wrongFlag = 1 # 最终有无找到正确结果的标志位
    while(1):
        f=openList[0][1]+openList[0][2]    #g表示现节点与起始点间的权重，h表示现节点与终止点间的权重
        nodePosition = 0 #记录
        for i in range(0,openListLength):
            if f > openList[i][1] + openList[i][2]:  #若openList中有节点的权重值小于现有权重值
                f = openList[i][1] + openList[i][2]
                nodePosition = i

        # 将f最小的节点放入close list
        closeListLength = closeListLength + 1
        closeList.append(openList[nodePosition])
        openListLength = 0
        #print('closeList[closeListLength-1][0]', closeList[closeListLength - 1][0])
        #print('goalPosition', goalPosition)
        #print('startPosition', startPosition)
        #print('closeList',closeList)
        c=goalPosition
        #print('\n')
        #print('openList' ,openList)
        #print('closeListLenghth:',closeListLength)
        if closeList[closeListLength-1][0] == goalPosition: # 判断现有节点是不是终止节点
            break
        a= weight_graph[(closeList[closeListLength-1][0])-1,:]
        newPosition = [newPosition+1 for (newPosition, val) in enumerate(a) if val < inf]
        #print('newPosition',newPosition)
        #newPosition = find(weight_graph[closeList[closeListLength].cross,:] != inf);
        flag = 0
        #openList=[]
        for i in range(0,len(newPosition)):
            for j in range(0,len(closeList)):
                if newPosition[i] == closeList[j][0]:
                    flag = 1
                    break
            if flag == 1:
                flag = 0
                continue
            openListLength = openListLength + 1
            cross = newPosition[i]
            d=closeListLength
            g = 0.3 * (closeList[closeListLength-1][1] + weight_graph[(closeList[closeListLength-1][0])-1,newPosition[i]-1])
            # 我擦勒！！！就是closeList的下标也不能大于length-1啊，啊啊
            #表示现节点与起始点间的权重

            d1 = math.floor((goalPosition - 1) / 8) + 1
            d2 = (goalPosition - 1)%8 + 1;
            d3 = math.floor((newPosition[i] - 1) /8) + 1
            d4 = (newPosition[i] - 1)%8 + 1;
            h = 0.7*(abs(d1 - d3) + abs(d2 - d4) - 1 / weight_graph[newPosition[i]-1, goalPosition-1])# 表示现节点与终止点间的权重
            #注意newPosition作为矩阵下标时，也得注意减1
            if openListLength<=len(openList):
                openList[openListLength-1] = [cross,g,h]
            else:
                openList.append([cross,g,h])
            #print('openList:')
            #print(openList)

        if openListLength == 0:
            print('出错了！')
            for i in range(0,len(closeList) - 1):
                forwardCross = closeList[i][0]
                backwardCross = closeList[i + 1][0]
                weight_graph[forwardCross-1, backwardCross-1] = weight_graph[forwardCross-1, backwardCross-1] + 0.1 * weight_graph[forwardCross-1, backwardCross-1]
                # 对于到达不了终止点的线路，对线路上的道路权重做惩罚性的增加，以降低之后选择这些道路的可能
            closeList = []
            closeListLength = 0
            openList = []
            openListLength = 0
            openList.append([0,0,0])
            openList[0][0] = startPosition #起始点装载到openList中

            openListLength = openListLength + 1 # openList中点个数加1
    roadPlan=[]
    for i in range(0,len(closeList)):
        roadPlan.append(closeList[i][0])
    return roadPlan

