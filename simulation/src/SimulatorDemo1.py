#!/usr/bin/env python
import rospy
import math
import cv2
import random
import os
import copy
import sys
from geometry_msgs.msg import PoseArray, Pose, Quaternion, PoseStamped, Point32
from nav_msgs.msg import OccupancyGrid, Path
from mscProject.msg import human
from sensor_msgs.msg import PointCloud

import allocation as al
import mobileRobotics as mr
import mapHelper as mh
import obstacle as obs
import human as hu

lockd = True
humanlock = False

pub = rospy.Publisher('robot_poses', PoseArray, queue_size=100)
pub2 = rospy.Publisher('test', Path, queue_size=100)
pub22 = rospy.Publisher('test2', Path, queue_size=100)
pub23 = rospy.Publisher('test3', Path, queue_size=100)
pub3 = rospy.Publisher('map2', OccupancyGrid, queue_size=100)
robots = []
small_map = 0
small_map2 = 0
big_map = 0
line_map = 0
people = []
den_map = 0
fname = "atc-20121226.txt"#"atc-20121212.txt"#"atc-20121226.txt"
start = 0
timestamp = 0
currentTimeStump = 21600+1356478882.15#1356478861.1
startingTimeStump = 0
step = 0.01
pubHumans = rospy.Publisher('human_poses', PointCloud, queue_size=100)
rpoints = []
connections = []
costArray=[]
pathText=''

def LastNlines(fname, N):
    # assert statement check
    # a condition
    assert N >= 0

    # declaring variable
    # to implement
    # exponential search
    pos = N + 1

    # list to store
    # last N lines
    lines = []

    # opening file using with() method
    # so that file get closed
    # after completing work
    with open(fname) as f:

        # loop which runs
        # until size of list
        # becomes equal to N
        while len(lines) <= N:

            # try block
            try:
                # moving cursor from
                # left side to
                # pos line from end
                f.seek(-pos, 2)

                # exception block
            # to hadle any run
            # time error
            except IOError:
                f.seek(0)
                break

            # finally block
            # to add lines
            # to list after
            # each iteration
            finally:
                lines = list(f)

                # increasing value
            # of variable
            # exponentially
            pos *= 2

    # returning the
    # whole list
    # which stores last
    # N lines
    return lines[-N:]

def publishHumans(people):
    cloud = PointCloud()
    cloud.header.frame_id = "map"
    for person in people:
        p = Point32()
        p.x = person[0]
        p.y = person[1]
        cloud.points.append(p)
    pubHumans.publish(cloud)


def publishRobots():
    global pub
    global robots
    poses = PoseArray()
    poses.header.frame_id = "map"
    ps = []
    for robot in robots:
        p = Pose()
        p.position.x = float(robot.x)
        p.position.y = float(robot.y)
        p.orientation.x = 1
        p.orientation.y = 0.001
        p.orientation.z = 0
        p.orientation.w = 0
        p.orientation = rotateQuaternion(p.orientation, robot.direction)
        ps.append(p)
    poses.poses = ps
    pub.publish(poses)


def pubhumans():
    global pub3
    global people
    poses = PoseArray()
    poses.header.frame_id = "map"
    ps = []
    flag = False
    for person in people:
        flag = True
        if person.moving:
            p = Pose()
            p.position.x = float(person.x)
            p.position.y = float(person.y)
            p.orientation.x = 1
            p.orientation.y = 0.001
            p.orientation.z = 0
            p.orientation.w = 0
            if person.vx != 0 and person.vy != 0:
                p.orientation = rotateQuaternion(p.orientation, math.atan2(person.vy, person.vx))
            ps.append(p)
    poses.poses = ps
    pub3.publish(poses)

    # if flag:
    #    rospy.sleep(111)


def getHeading(q):
    """
    Get the robot heading in radians from a Quaternion representation.

    :Args:
        | q (geometry_msgs.msg.Quaternion): a orientation about the z-axis
    :Return:
        | (double): Equivalent orientation about the z-axis in radians
    """
    yaw = math.atan2(2 * (q.x * q.y + q.w * q.z),
                     q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)
    return yaw


def multiply_quaternions(qa, qb):
    """
    Multiplies two quaternions to give the rotation of qb by qa.

    :Args:
       | qa (geometry_msgs.msg.Quaternion): rotation amount to apply to qb
       | qb (geometry_msgs.msg.Quaternion): to rotate by qa
    :Return:
       | (geometry_msgs.msg.Quaternion): qb rotated by qa.
    """
    combined = Quaternion()

    combined.w = (qa.w * qb.w - qa.x * qb.x - qa.y * qb.y - qa.z * qb.z)
    combined.x = (qa.x * qb.w + qa.w * qb.x + qa.y * qb.z - qa.z * qb.y)
    combined.y = (qa.w * qb.y - qa.x * qb.z + qa.y * qb.w + qa.z * qb.x)
    combined.z = (qa.w * qb.z + qa.x * qb.y - qa.y * qb.x + qa.z * qb.w)
    return combined


def rotateQuaternion(q_orig, yaw):
    """
    Converts a basic rotation about the z-axis (in radians) into the
    Quaternion notation required by ROS transform and pose messages.

    :Args:
       | q_orig (geometry_msgs.msg.Quaternion): to be rotated
       | yaw (double): rotate by this amount in radians
    :Return:
       | (geometry_msgs.msg.Quaternion) q_orig rotated yaw about the z axis
     """
    # Create a temporary Quaternion to represent the change in heading
    q_headingChange = Quaternion()

    p = 0
    y = yaw / 2.0
    r = 0

    sinp = math.sin(p)
    siny = math.sin(y)
    sinr = math.sin(r)
    cosp = math.cos(p)
    cosy = math.cos(y)
    cosr = math.cos(r)

    q_headingChange.x = sinr * cosp * cosy - cosr * sinp * siny
    q_headingChange.y = cosr * sinp * cosy + sinr * cosp * siny
    q_headingChange.z = cosr * cosp * siny - sinr * sinp * cosy
    q_headingChange.w = cosr * cosp * cosy + sinr * sinp * siny

    # ----- Multiply new (heading-only) quaternion by the existing (pitch and bank)
    # ----- quaternion. Order is important! Original orientation is the second
    # ----- argument rotation which will be applied to the quaternion is the first
    # ----- argument.
    return multiply_quaternions(q_headingChange, q_orig)


def readTextFile(dirr):
    file1 = open(dirr, "r")
    text = file1.read().splitlines()
    file1.close()
    tab = []
    for line in text:
        tab.append(line.split())
    return tab

def readPath(method,i):
    cwd = os.getcwd() + '/path/path'+method[0]+str(i)+'.txt'
    
    file1 = open(cwd, "r")
    text = file1.read().split(',')
    file1.close()
    tab = []
    i=0
    point = []
    for line in text:
        point.append(float(line))
        i=i+1
        if i%2==0:
           tab.append(point)
           point=[]
    return tab

def createRobots(dira):
    '''
    tab = readTextFile(dir)
    robots = []
    for i, t in enumerate(tab):
        r = mr.robot(float(t[0]), float(t[1]), float(t[2]), i, [float(t[3]), float(t[4])], 10, 5, 50, 0.1, 3, 1,
                     0.1, 1, 0.1, 0.3, 0.1)
        robots.append(r)
    '''
    i = 0
    robots = []
    #print('a')
    he = big_map.info.height
    wi = big_map.info.width
    #print('a')
    helper = int(dira)
    while len(robots) < helper:
        h = random.randint(0, he - 1)
        w = random.randint(0, wi - 1)
        #print('b')
        #print(len(robots))
        yaw = random.random() * 2 * math.pi - math.pi
        #print(dira)
        #print(len(robots) < 1)
        #input('ewq')
        if small_map2[h][w] == 0:
            #print('c')
            i = i + 1
            x, y = mh.from_index_to_real_point(w, h, big_map.info)
            r = mr.robot(x, y, yaw, i, [x, y], 1, 0.5, 50, 0.1, 3, 1,
                         0.01, 1, 0.1, 0.3, 0.1)
            robots.append(r)

    return robots


def allocateNewTasks(data,method='Novel',tras=None):
    global robots
    global lockd
    global humanlock
    global currentTimeStump
    global startingTimeStump
    global costArray
    humanlock = True
    trajectories = None
    tasks = []
    for pose in data.poses:
        x = pose.position.x
        y = pose.position.y
        t = [x, y]
        tasks.append(t)
    rospy.loginfo("The chosen allocation method is "+method)
    if method == 'Euc':
        bids = al.createBids(robots, tasks, method)
        startingTimeStump=currentTimeStump
        # rospy.loginfo(bids)
        allocations = al.auction(bids)

        # allocations = [0]

        for i, allocation in enumerate(allocations):
            if tras is None:
                #robots[allocation].addTask(tasks[i], small_map, big_map, rpoints, connections)
                readTras = readPath(method,i)
            #robots[allocation].addTask(tasks[i], small_map, big_map, rpoints, connections, trajectories[allocation][i])
                robots[allocation].addTask(tasks[i], small_map, big_map, rpoints, connections, readTras)
            #savePath(trajectories[allocation][i])
                savePath(readTras)
            else:
                readTras = readPath(method,i)
            #robots[allocation].addTask(tasks[i], small_map, big_map, rpoints, connections, trajectories[allocation][i])
                robots[allocation].addTask(tasks[i], small_map, big_map, rpoints, connections, readTras)
            #savePath(trajectories[allocation][i])
                savePath(readTras)
            p = Path()
            p.header.frame_id = "map"
            for t in readTras:#robots[allocation].trajectories[0]:
                pose = PoseStamped()
                pose.pose.position.x = t[0]
                pose.pose.position.y = t[1]
                p.poses.append(pose)
            if i == 0:
                pub2.publish(p)
            elif i == 1:
                pub22.publish(p)
            elif i == 2:
                pub23.publish(p)
    elif method == 'Path':
        bids, trajectories = al.createBids(robots, tasks, method, big_map, small_map, currentTimeStump,tras=tras)
        startingTimeStump=currentTimeStump
        # rospy.loginfo(bids)
        allocations = al.auction(bids)

        # allocations = [0]

        for i, allocation in enumerate(allocations):
            readTras = readPath(method,i)
            #robots[allocation].addTask(tasks[i], small_map, big_map, rpoints, connections, trajectories[allocation][i])
            robots[allocation].addTask(tasks[i], small_map, big_map, rpoints, connections, readTras)
            #savePath(trajectories[allocation][i])
            savePath(readTras)
            p = Path()
            p.header.frame_id = "map"
            for t in readTras:#robots[allocation].trajectories[0]:
                pose = PoseStamped()
                pose.pose.position.x = t[0]
                pose.pose.position.y = t[1]
                p.poses.append(pose)
            if i == 0:
                pub2.publish(p)
            elif i == 1:
                pub22.publish(p)
            elif i == 2:
                pub23.publish(p)
    else:
        bids, trajectories,costArray = al.createBids(robots, tasks, method, big_map, small_map, currentTimeStump,tras=tras)
        startingTimeStump=currentTimeStump
        # rospy.loginfo(bids)
        allocations = al.auction(bids)

        # allocations = [0]

        for i, allocation in enumerate(allocations):
            readTras = readPath(method,i)
            #robots[allocation].addTask(tasks[i], small_map, big_map, rpoints, connections, trajectories[allocation][i])
            robots[allocation].addTask(tasks[i], small_map, big_map, rpoints, connections, readTras)
            #savePath(trajectories[allocation][i])
            savePath(readTras)
            p = Path()
            p.header.frame_id = "map"
            for t in readTras:#robots[allocation].trajectories[0]:
                pose = PoseStamped()
                pose.pose.position.x = t[0]
                pose.pose.position.y = t[1]
                p.poses.append(pose)
            if i == 0:
                pub2.publish(p)
            elif i == 1:
                pub22.publish(p)
            elif i == 2:
                pub23.publish(p)

    # input('das')
    humanlock = False
    # print(humanlock)
    lockd = False
    return trajectories

def savePath(path):
    global pathText
    '''
    cwd = os.getcwd()
    dir = cwd + '/paths'
    os.system('mkdir '+ dir)
    files = os.listdir(dir)
    n = len(files)

    dir=dir+'/r'+str(n)+'.txt'
    f = open(dir, "a")
    '''
    text=''
    for point in path:
       text=text+str(point[0])+','+str(point[1])+','
    #f.write(text[:-1])
    pathText = pathText + '\n'+text
    #f.close()

def addNewHumans(data):
    global people
    person = hu.human(data, len(people))
    people.append(person)

def allStopped(robots):
    for robot in robots:
        if robot.moving == True:
            return False
    return True

def arrayContain(arr,v):
    i=0
    for a in arr:
        if a==v:
            return i
        i=i+1
    return -1

def saveArray(t,r,moment,robots,tasks):
    text = str(t[0])+','+str(t[1])+','+str(t[2])+'\n'+str(moment)+'\n'
    for robot in robots:
        text=text+str(robot.x)+','+str(robot.y)+','
    text = text[:-1]+'\n'
    for task in tasks.poses:
        text=text+str(task.position.x)+','+str(task.position.y)+','
    text = text[:-1]

    #for d,p in zip(distances,prob):
    #    text = text + ","+str(d)+ ","+str(p)
    writeRoad(text,r)

def writeRoad(text,r):
    global pathText
    text = text + pathText
    cwd = os.getcwd()
    dir = cwd + '/robots'+str(r)
    os.system('mkdir '+ dir)
    files = os.listdir(dir)
    n = len(files)

    dir=dir+'/r'+str(n)+'.txt'
    f = open(dir, "a")
    f.write(text)
    f.close()


def runSimulation():
    global small_map
    global big_map
    global robots
    global line_map
    global start
    global timestamp
    global currentTimeStump
    global step
    global end
    global humanlock
    t=0
    while not rospy.is_shutdown():
        with open(fname, 'r') as f:
            people = []

            for i, line in enumerate(f):
                # print(humanlock)
                while humanlock:
                    pass
                if i > start:
                    k = line.split(',')
                    if timestamp == float(k[0]):
                        people.append([float(k[1]), float(k[2])])
                    else:
                        while currentTimeStump < float(k[0]) and not rospy.is_shutdown():
                            if len(sys.argv)<3:
                                rospy.sleep(step)
                            else:
                                rospy.sleep(step/float(sys.argv[2]))
                            currentTimeStump = currentTimeStump + step
                            t = t +step
                            # print(currentTimeStump)
                            if not lockd:
                                mr.move(robots, line_map, small_map, big_map, pub, people, rpoints, connections)
                                if allStopped(robots):
                                    #saveArray(costArray,startingTimeStump,currentTimeStump)
                                    return t
                            publishRobots()
                            if t>3600:
                                return None
                        publishHumans(people)
                        people = []
                        timestamp = float(k[0])
                        #print(timestamp)

        # hu.move(people, line_map, pub,den_map,robots)
        # publishhumans()


def setTime(fname):
    iteration = 0
    global timestamp
    global start
    with open(fname, 'r') as f:
        for line in f:
            k = line.split(',')
            if float(k[0]) != timestamp:

                if float(k[0]) < currentTimeStump:
                    start = iteration
                    timestamp = float(k[0])
                else:
                    end = iteration
                    nextTimestamp = float(k[0])
                    break
            # o=int(k[4])
            iteration = iteration + 1

global robots
global small_map
global small_map2
global big_map
global line_map
global people
global den_map
global start
global timestamp
global currentTimeStump
global step
global end
global rpoints
global connections
global humanlock
global lockd
global pathText

rospy.init_node('simulatior', anonymous=True)

try:  # Try to load map
    big_map = rospy.wait_for_message("/map", OccupancyGrid, 20)
except:  # Send error otherwise
    rospy.logerr("Problem getting a map. Check that you have a map_server"
                 " running: rosrun map_server map_server <mapname> ")
    sys.exit(1)

new_map = mh.create_a_map(big_map.info.width, big_map.info.height, big_map.data)
xmin, ymin, xmax, ymax = mh.small_index(new_map, big_map.info.width, big_map.info.height)

small_map = mh.dilation(new_map, xmin, ymin, xmax, ymax, 0.002, big_map.info.resolution)
small_map2 = mh.dilation(new_map, xmin, ymin, xmax, ymax, 1.5, big_map.info.resolution)
'''
tablica = []
for i in range(big_map.info.height):
    for j in range(big_map.info.width):

        tablica.append(small_map[i][j])
big_map.data=tablica

#line_map=''#obs.createLineMap('xline.txt','yline.txt')
for xline in line_map.xlines:
    rospy.loginfo(xline.x)
for i in range(100):
    rospy.loginfo('qqqqqqqqqqq')
    pub3.publish(big_map)
    rospy.sleep(0.1)
# add subscribers and publishers
'''
rospy.Subscriber('tasks', PoseArray, allocateNewTasks)
rospy.Subscriber('humans', human, addNewHumans)

with open(fname, 'r') as f:
    # print(filename[2:])
    # raw_input("Press Enter to continue...")
    first_line = f.readline()
    k = first_line.split(',')
    # print(first_line )
    # raw_input("Press Enter to continue...")
    t_ini = int(float(k[0]))
    # print(t_ini)
    # raw_input("Press Enter to continue...")
k = LastNlines(fname, 1)[0].split(',')
t_end = int(float(k[0]))
'''
currentTimeStump = random.random() * (t_end - t_ini) + t_ini
robots = createRobots(1)
helper = createRobots(1)
poseHelper = PoseArray()
for h in helper:
    p = Pose()
    p.position.x=h.x
    p.position.y=h.y
    poseHelper.poses.append(p)


allocateNewTasks(poseHelper)
'''
iteration = 0
nextTimestamp = 0
#setTime(fname)
rpoints = 0
connections = 9

print('start sending')
#print(sys.argv[1])

itersMax=1
method = ['Path','Euc','Novel']
timesOfRun = [0,0,0]
#print(2)
for i in range(itersMax):
    tras = None
    pathText=''
    timeHelper =1356505345.46 #random.random() * (t_end -3600- t_ini) + t_ini
    #print(3)
    robotHelper =  createRobots(sys.argv[1])
    helper = createRobots(sys.argv[1])
    
    robotHelper[0].x = -6.59
    robotHelper[0].y = -6.69
    robotHelper[1].x = -34.59
    robotHelper[1].y = -3.599
    robotHelper[2].x = -2.49
    robotHelper[2].y = 6.200
    #print(3)

    helper[0].x = 25.900
    helper[0].y = -16.89
    helper[1].x = 5.2000
    helper[1].y = -0.69
    helper[2].x = -13.09
    helper[2].y = 5.20000
    
    #print(3)
    poseHelper = PoseArray()
    #print(4)
    for h in helper:
        p = Pose()
        p.position.x = h.x
        p.position.y = h.y
        poseHelper.poses.append(p)
        #print(4)
    for j,m in enumerate(method): 
        #input('das')
        currentTimeStump = timeHelper
        print(timeHelper)
        setTime(fname)
        robots = copy.deepcopy(robotHelper)

        lockd = True
        humanlock = False
        trajHelper=allocateNewTasks(poseHelper,m,tras)
        if not trajHelper is None:
            tras = trajHelper
        timesOfRun[j]=runSimulation()
    saveArray(timesOfRun,sys.argv[1],currentTimeStump,robotHelper,poseHelper)

