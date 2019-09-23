# -*- coding: utf-8 -*-
#floyd.py
#from pylab import *
import numpy as np
import buildWeight
import time
inf=1e5
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

  
