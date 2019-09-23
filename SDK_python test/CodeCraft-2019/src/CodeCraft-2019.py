import logging
import sys
#import time
import numpy as np
import buildWeight
import floyd


#logging.basicConfig(level=logging.DEBUG,
#                    filename='../logs/CodeCraft-2019.log',
#                    format='[%(asctime)s] %(levelname)s [%(funcName)s: %(filename)s, %(lineno)d] %(message)s',
#                    datefmt='%Y-%m-%d %H:%M:%S',
#                    filemode='a')
inf=1e5
def main():
	if len(sys.argv) != 5:
		logging.info('please input args: car_path, road_path, cross_path, answerPath')
		exit(1)

	car_path = sys.argv[1]
	road_path = sys.argv[2]
	cross_path = sys.argv[3]
	answer_path = sys.argv[4]

	logging.info("car_path is %s" % (car_path))
	logging.info("road_path is %s" % (road_path))
	logging.info("cross_path is %s" % (cross_path))
	logging.info("answer_path is %s" % (answer_path))

	# to read input file
	source = road_path  ## 源文件路径
	dest = '../config/road1.txt'  ## 去除括号后的文件路径
	f = open(dest, "w+")
	f.truncate()
	with open(source, 'r') as text:
		with open(dest, 'a+') as road_1:  ## 以追加写的方式打开目标文件
				for line in text.readlines():
					road_1.write(line.replace('(', '').replace(')', ''))  ## 去除一行的左右括号
		roadMat = np.loadtxt(dest, dtype=int, skiprows=1, delimiter=',')  ## 读取txt并转换为ndarray

	source2 = car_path  ## 源文件路径
	dest2 = '../config/car1.txt'  ## 去除括号后的文件路径
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

		weight = buildWeight.buildWeight(roadMat, 1e5)
		# print(weight.shape)
		D, path = floyd.floyd(weight)

		for i in range(0,car_amount):

			R=floyd.router(D,path,from_car[i],to_car[i])

			R1 = []  # 用R1存储道路编号carId
			# 实现节点到道路编号的索引查找
			for m in range(0, len(R) - 1):
				for l in range(0, len(roadId)):
					#if from_.tolist().index(R[m])==to_.tolist().index(R[m+1]) or from_.tolist().index(R[m+1])==to_.tolist().index(R[m]):
					if from_[l] + to_[l] == R[m] + R[m + 1] and from_[l] - R[m] + to_[l] - R[m + 1] == 0:
						R1.append(roadId[l])
						break
			if planTime[i]<3:
				planTime1=planTime[i]
			else:
				planTime1=planTime[i]+np.random.randint(1,500)
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
