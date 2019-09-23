# -*- coding: utf-8 -*-
import numpy as np
import buildWeight
import time
inf=1e5
'''
dijkstra算法：时间复杂度n^2
'''
import numpy as np
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
