import math
import rospy
import mapHelper as mh
import time
import cv2
import os
from timeit import default_timer as timer

def FindSmallest(arr1):
    index = [0][0]
    minn = arr1[0][0]

    for i in range(len(arr1)):
        for j in range(len(arr1[i])):
            if arr1[i][j] < minn:
                minn = arr1[i][j]
                index[0] = i
                index[1] = j
    return index


def FindBiggest(arr1):
    index = [0, 0]
    maxi = arr1[0][0]

    for i in range(len(arr1)):
        for j in range(len(arr1[i])):
            if arr1[i][j] > maxi:
                maxi = arr1[i][j]
                index[0] = i
                index[1] = j
    return index


def distan(x1, y1, x2, y2):  # Compute distance between points
    dist = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    return dist


def createBids(robots, tasks, method,bmap=[],smap=[],currentTimeStump=0,tras=None):  # To do
    bids = []
    if method=='Euc':
        for robot in robots:
            rbids = []
            for task in tasks:
                bid = computeBid(robot, task, method,bmap,smap)
                rbids.append(bid)
            bids.append(rbids)
        return bids
    elif method=='Path':
        trajectories=[]
        for i,robot in enumerate(robots):
            rbids = []
            rtra=[]
            for j,task in enumerate(tasks):
                if tras is None:
                    bid,tra = computeBid(robot, task, method,bmap,smap,currentTimeStump)
                else:
                    bid,tra = computeBid(robot, task, method,bmap,smap,currentTimeStump,tras[i][j])
                rbids.append(bid)
                rtra.append(tra)
            bids.append(rbids)
            trajectories.append(rtra)
        return bids,trajectories

    else:
        trajectories=[]
        for i,robot in enumerate(robots):
            rbids = []
            rtra=[]
            for j,task in enumerate(tasks):
                if tras is None:
                    bid,tra,costArray = computeBid(robot, task, method,bmap,smap,currentTimeStump)
                else:
                    bid,tra,costArray = computeBid(robot, task, method,bmap,smap,currentTimeStump,tras[i][j])
                rbids.append(bid)
                rtra.append(tra)
            bids.append(rbids)
            trajectories.append(rtra)
        return bids,trajectories,costArray

def saveMethod(method,t):
    cwd = os.getcwd()
    dir = cwd + '/'+method+'AllTimes'
    os.system('mkdir '+ dir)
    text=str(t)

    files = os.listdir(dir)
    n = len(files)

    dir=dir+'/a'+str(n)+'.txt'
    f = open(dir, "a")
    f.write(text)
    f.close()


def computeBid(robot, task, method,bmap=[],smap=[],currentTimeStump=0,tra = None):

    if method=='Euc':
        startTime = timer()
        bid =  distan(robot.x, robot.y, task[0], task[1])
        endTime = timer()
        #saveMethod(method,endTime-startTime)
        return bid
    elif method=='Path':
        if tra is None:
            tra = mh.globalPlanner(robot.x, robot.y, 0.5 * math.pi - robot.direction, task[0], task[1], 0, bmap.info, smap)
        startTime = timer()
        bid=0
        for i in range(len(tra)-1):
            bid = bid+distan(tra[i][0],tra[i][1],tra[i+1][0],tra[i+1][1])
        endTime = timer()
        #saveMethod(method,endTime-startTime)
        return bid,tra
    elif method=='Novel':
        if tra is None:
            tra = mh.globalPlanner(robot.x, robot.y, 0.5 * math.pi - robot.direction, task[0], task[1], 0, bmap.info, smap)
        startTime = timer()
        costArray=[]
        bid=0
        path = os.getcwd() +'/predicted_binary_img2'
        r = currentTimeStump%600
        t=currentTimeStump-r
        #r = currentTimeStump%1800
        #t=currentTimeStump-r+334
        name = str(time.ctime(t+28800+3600 ))+".png"
        #print(name)
    #print(name)
    #print(bid)
        img=cv2.imread(path+'/'+name,0)
        for i in range(len(tra)-1):
            punish,pro1,pro2 = probabilityParmameter(tra[i],tra[i+1],img,bmap.info,bid)
            dista=distan(tra[i][0],tra[i][1],tra[i+1][0],tra[i+1][1])
            bid = bid+dista*punish
            costa = [dista,(pro1+pro2)/2]
            costArray.append(costa)
        endTime = timer()
        #saveMethod(method,endTime-startTime)
        return bid,tra,costArray

def probabilityParmameter(point1,point2,img,mapinfo,bid): #TO DO
    #print(currentTimeStump)
    w5=[9.29980536, 0.02222327, 1.00558889]
    #path = os.getcwd() +'/predicted_binary_img2'
    #r = currentTimeStump%600
    #t=currentTimeStump-r
    #r = currentTimeStump%600
    #t=currentTimeStump-r
    #name = str(time.ctime(t+28800+3600 ))+".png"
    #print(name)
    #print(bid)
    #img=cv2.imread(path+'/'+name,0)
    y,x=mh.from_real_point_to_index(point1[0], point1[1], mapinfo)
    x=650-x
    pro1=math.ceil(float(img[x][y])*100/255)
    #print(pro1)
    y,x=mh.from_real_point_to_index(point2[0], point2[1], mapinfo)
    x=650-x
    pro2=math.ceil(float(img[x][y])*100/255)
    #print(pro2)
    #input('222')
    probability = (pro1+pro2)/200
    panish = w5[2]+w5[1] * ((math.exp(probability * w5[0])))
    #if probability>0.19:
    #    print('---------')
    #    print(probability)
     #   print(panish)
    return panish,pro1,pro2

def auction(bids):
    tl = len(bids[0])
    index = 0
    out = [-1] * tl
    while True:
        flag = False
        copyOfVotes = crossOut(bids, out)
        while True:
            i = FindBiggest(copyOfVotes)
            if nonZeroElementsCol(copyOfVotes, i[1]) == 1:
                index = FindBiggestCol(copyOfVotes, i[1])
                out[i[1]] = index
                break
            if nonZeroElementsRow(copyOfVotes, i[0]) == 1:
                index = FindBiggestRow(copyOfVotes, i[0])
                out[index] = i[0]
                break

            copyOfVotes[i[0]][i[1]] = 0

        if (contains(out, -1) == 0):
            break
        # for i in range(len(out)):
        # if out[i] == -1:
        #   bids[index][i] = computeBid(robots[index], tasks[i], method)
    return out


def FindBiggestCol(arr1, col):
    index = 0
    maxd = arr1[index][col]
    for j in range(len(arr1)):
        if (arr1[j][col] > maxd):
            maxd = arr1[j][col]
            index = j

    return index


def FindBiggestRow(arr1, row):
    index = 0
    maxd = arr1[row][0]
    for j in range(len(arr1)):
        if (arr1[row][j] > maxd):
            maxd = arr1[row][j]
            index = j

    return index


def contains(array, v):
    result = 0
    for i in array:
        if i == v:
            result = result + 1
    return result


def nonZeroElementsCol(votes, index):
    result = 0
    for i in range(len(votes)):
        if votes[i][index] != 0:
            result = result + 1
    return result


def nonZeroElementsRow(votes, index):
    result = 0
    for i in range(len(votes)):
        if votes[index][i] != 0:
            result = result + 1
    return result


def crossOut(bids, outd):
    cross = [[bids[x][y] for y in range(len(bids[0]))] for x in range(len(bids))]
    for i in range(len(bids)):
        for j in range(len(bids[i])):
            if (outd[j] != -1):
                cross[i][j] = 0
                cross[outd[j]][i] = 0
    return cross

