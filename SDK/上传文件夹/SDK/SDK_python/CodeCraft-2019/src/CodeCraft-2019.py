#coding:utf-8
import logging
import sys

#import time
import numpy as np
#import buildWeight
#import floyd
import math


#logging.basicConfig(level=logging.DEBUG,
#                    filename='../logs/CodeCraft-2019.log',
#                    format='[%(asctime)s] %(levelname)s [%(funcName)s: %(filename)s, %(lineno)d] %(message)s',
#                    datefmt='%Y-%m-%d %H:%M:%S',
#                    filemode='a')
inf=1e5
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
    timeWeight=(np.true_divide(mat_roadLength,(mat_maxSpeed+1e-4)))
    #np.savetxt("timeWeight.txt",  timeWeight)
    #print(timeWeight.shape)
    return timeWeight
# -*- coding: utf-8 -*-
#floyd.py
#from pylab import *

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
def router(D, path, s, t):
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
	maxSpeed=carMat[:,3]
	from_car=carMat[:,1]
	to_car=carMat[:,2]
	planTime=carMat[:,4]
	carId=carMat[:,0]
	car_amount=len(from_car)
	'''
	初始化权重矩阵，并运行floyd最短路径寻找程序
	'''
#	start = time.time() # 计算耗时，单位为s
	R_all=[]
	planTime1=0
	from_ = roadMat[:, 4]
	to_ = roadMat[:, 5]
	roadId = (roadMat[:, 0])
	with open(answer_path, 'a+') as file:
		file=open(answer_path,'w')
		file.truncate()

		weight = buildWeight(roadMat, 1e5)
		# print(weight.shape)
		D, path = floyd(weight)

		for i in range(0,car_amount):

			R=router(D,path,from_car[i],to_car[i])

			R1 = []  # 用R1存储道路编号carId
			# 实现节点到道路编号的索引查找
			for m in range(0, len(R) - 1):
				for l in range(0, len(roadId)):
					#if from_.tolist().index(R[m])==to_.tolist().index(R[m+1]) or from_.tolist().index(R[m+1])==to_.tolist().index(R[m]):
					if from_[l] + to_[l] == R[m] + R[m + 1] and from_[l] - R[m] + to_[l] - R[m + 1] == 0:
						R1.append(roadId[l])
						break
			if planTime[i]<1:
				planTime1=planTime[i]
			else:
				planTime1=planTime[i]+np.random.randint(1,1000)
			R1.insert(0, planTime1)
			R1.insert(0, carId[i])
			#R.insert(0,'(')
			#R.append('/n')

			file.write(str(tuple(R1)) + '\n')

	#print(car_amount)
	#print(i)

	#np.savetxt("answer.txt",  R_all)
#	end = time.time()
#	print(end-start)

# to write output file


if __name__ == "__main__":
    main()
