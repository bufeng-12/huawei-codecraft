'''
输入车辆自身限速，
从而输出权重矩阵length/(min(maxSpeed,limitSpeed)+1e-4),加上极小值防止分母无意义
初始规划的时候，可以考虑加入匹配系数alpha*abs(maxSpeed-limitSpeed)
'''
# -*- coding: utf-8 -*-
from numpy import *
import numpy as np 
#import pandas as pd
import re

def buildWeight(roadMat,maxSpeed):
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
    #print(roadMat)
    roadMat = roadMat.astype(np.int16)
    #print(roadMat.dtype)

    from_ = roadMat[:,4]
    to_ = roadMat[:,5]
    isDuplex = roadMat[:,6]
    roadLength = (roadMat[:,1])
    roadId = (roadMat[:,0])
    limitSpeed = (roadMat[:,2])
    nodenum = max(to_)   #此处是索引to_里的最大值来找到节点数
    #print(nodenum)
    mat_roadLength=float('inf')*np.ones(shape=(nodenum,nodenum))  #不通的道路长度无穷大
    mat_maxSpeed=np.zeros(shape=(nodenum,nodenum))
   # maxSpeed=maxSpeed*np.ones(shape=(len(from_),len(from_)))
    #构造路长与限速矩阵
    #print(len(from_))
    for i in range(0,len(from_)):
            mat_roadLength[from_[i]-1][to_[i]-1] = roadLength[i]
            mat_roadLength[to_[i]-1][from_[i]-1] = roadLength[i]
            
            mat_maxSpeed[from_[i]-1][to_[i]-1] = min(limitSpeed[i],maxSpeed)
            mat_maxSpeed[to_[i]-1][from_[i]-1] = min(limitSpeed[i],maxSpeed)
            if isDuplex[i]==0:  #当非双向的时候，limitSpeed=0
                mat_maxSpeed[to_[i]-1][from_[i]-1]=0
    # print(mat_roadLength)
    #print(mat_maxSpeed)
    '''
    矩阵的建立方法：length/（速度+1e-4),
     没有连通的（包括自己到自己）length为inf，单向的那么反向限制速度为0
    '''
    #np.savetxt("mat_roadLength.txt", mat_roadLength)
    #np.savetxt("mat_maxSpeed.txt", mat_maxSpeed)
    # mat_maxSpeed=min(mat_maxSpeed,maxSpeed)
    timeWeight=floor(np.true_divide(mat_roadLength,(mat_maxSpeed+1e-4)))
    np.savetxt("timeWeight.txt",  timeWeight)
    #print(timeWeight.shape)
    return timeWeight