import math
import mapHelper as mh
import rospy


class Position:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class Pose:
    def __init__(self, x, y):
        self.x = x
        self.y = y  # To DO ?


class Task:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class robot:
    def __init__(self, x, y, direction, id, base, maxv, maxa, kp, kn, negativeBorder, positiveBorder, t, treshV,
                 treshW, treshLast, radius):
        self.x = x
        self.y = y
        self.direction = direction
        self.v = 0
        self.a = 1
        self.id = id
        self.base = base
        self.maxv = maxv
        self.maxa = maxa
        self.tasks = []
        self.trajectories = []
        self.goalDist = 0
        self.nextDist = 0
        self.preDist = 0
        self.currentDist = 0
        self.target = 0
        self.moving = False
        self.rotating = False
        self.kp = kp
        self.kn = kn
        self.negativeBorder = negativeBorder
        self.positiveBorder = positiveBorder
        self.t = t
        self.treshV = treshV
        self.treshW = treshW
        self.treshLast = treshLast
        self.radius = radius
        self.pause = False

    def navigate(self, scan, people):
        for person in people:
            if distan(person[0], person[1], self.x, self.y) < (
                    3 * max(0, math.cos(math.atan2(person[1] - self.y, person[0] - self.x) - self.direction)) + 1):
                self.lv = 0
                self.rv = 0
                return None
        dif_x = self.trajectories[0][self.target][0] - self.x
        dif_y = self.trajectories[0][self.target][1] - self.y
        forcepx, forcepy = positive_force(dif_x, dif_y, self.kp, self.positiveBorder)
        forcepx, forcepy = rotation(forcepx, forcepy,
                                    self.direction)  # Rotate forces to robot coordinate axis from map axis
        if True:  # self.target == (len(self.trajectories[0])-1):
            forcenx = 0
            forceny = 0

        force_robot_x = forcepx + forcenx  # add them
        force_robot_y = forcepy + forceny
        self.lv, self.rv = controller(force_robot_x, force_robot_y, self, 0.5,
                                      1)  # Conroller compute best velocity

    def stepF(self,  sMap, bMap, pub, people, points, graph):
        '''
        v = (self.lv + self.rv) / 2
        w = (self.rv - self.lv) / (2 * self.radius)
        self.x = self.x + self.t * v * math.cos(self.direction)
        self.y = self.y + self.t * v * math.sin(self.direction)
        self.direction = self.direction + self.t * w
        '''
        noticedPerson = False
        for person in people:
            if distan(person[0], person[1], self.x, self.y) < 1:#(
                    #3 * max(0, math.cos(math.atan2(person[1] - self.y, person[0] - self.x) - self.direction)) + 1):
                noticedPerson = True
                break
        #flag = False  # Flag ogarnij
        if noticedPerson:
            states = ['d']
            v2 = self.v - self.t * self.maxa
            if v2 > 0:
                # d = (v1 + v2) * t / 2
                timesOut = [self.t]
            else:
                t1 = self.v / self.maxa
                timesOut = [t1]
            self.pause = True
        else:
            if self.pause:
                self.pause = False
                self.a = self.maxa
            states, timesOut = computeState(self.maxv, self.maxa, self.t, self.v, self.a, self.currentDist,
                                            self.goalDist)
            for i in range(len(timesOut)):
                if timesOut[i] < 0:
                    timesOut[i] = 0
        #print(states)
        #print(timesOut)
        #input('stany/czasy')
        d, self.v, self.a = computeVariables(states, timesOut, self.v, self.maxa, self.maxv)
        #print(d)
        #print(self.v)
        #print(self.a)
        self.currentDist = self.currentDist + d
        #print(self.currentDist)
        #input('dist/v/a/calydist')

        while True:
            if self.currentDist > self.nextDist:
                self.preDist = self.nextDist
                self.target = self.target + 1
                try:
                    self.nextDist = distan(self.trajectories[0][self.target][0], self.trajectories[0][self.target][1],
                                           self.trajectories[0][self.target + 1][0],
                                           self.trajectories[0][self.target + 1][1])+self.nextDist
                except:
                    pass
            else:
                self.direction = math.atan2(
                    self.trajectories[0][self.target + 1][1] - self.trajectories[0][self.target][1],
                    self.trajectories[0][self.target + 1][0] - self.trajectories[0][self.target][0])
                difDist = self.nextDist - self.preDist
                difCurrentDist = self.currentDist - self.preDist
                self.x = self.trajectories[0][self.target][0] + (self.trajectories[0][self.target + 1][0] -
                                                                 self.trajectories[0][self.target][
                                                                     0]) * difCurrentDist / difDist
                self.y = self.trajectories[0][self.target][1] + (self.trajectories[0][self.target + 1][1] -
                                                                 self.trajectories[0][self.target][
                                                                     1]) * difCurrentDist / difDist
                break
        if self.currentDist > self.goalDist - 0.5:
            self.nextTask(sMap, bMap, pub, points, graph)

    def rotate(self):  # Try to rotate a robot to go trhough trajectory

        x1 = self.trajectories[0][0][0]
        y1 = self.trajectories[0][0][1]
        try:
            x2 = self.trajectories[0][1][0]
            y2 = self.trajectories[0][1][1]
            yaw = math.atan2(y2 - y1, x2 - x1)  # compute yaw
            self.lv = -0.2
            self.rv = 0.2
            self.lv, self.rv = controller(x2 - x1, y2 - y1, robot, 0, 0.5)
            self.stepF()
            if is_small_angle(yaw, self.direction, self.treshW):
                self.stopRotating()
        except:
            self.lv = 0
            self.rv = 0
            self.stopRotating()

    def stopRotating(self):
        self.rotating = False
        self.w = 0

    def go(self, robots, lineMap, sMap, bMap, pub, people, points, graph):
        # scan = ''  # lineMap.scanner(self,robots)
        '''if len(scan)>0:
            rospy.loginfo(scan)
            rospy.loginfo('tttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttt')'''
        # self.navigate(scan, people)
        self.stepF(sMap, bMap, pub, people, points, graph)
        '''
        if self.target == (len(self.trajectories[0]) - 1):

            if self.is_smalest_dist():
                self.lv = 0
                self.rv = 0
                self.nextTask(sMap, bMap, pub, points, graph)
        else:
            if self.is_smal_dist():
                self.target = self.target + 1
        '''

    def nextTask(self, sMap, bMap, pub, points, graph):
        if len(self.tasks) == 0 or len(self.tasks) == 1:
            self.tasks = []
            self.moving = False
            self.target = 0
            self.trajectories = []
            self.rotating = 0
        elif len(self.tasks) == 1:
            self.tasks.pop(0)
            self.trajectories.pop(0)
            self.returnToBase(sMap, bMap, pub, points, graph)
        else:
            self.tasks.pop(0)
            self.trajectories.pop(0)
            self.rotating = True
            self.target = 0

    def returnToBase(self, sMap, bMap, pub, points, graph):
        tra = mh.globalPlanner(self.x, self.y, self.base[0], self.base[1], bMap.info, sMap)
        self.rotating = True
        self.trajectories.append(tra)
        self.target = 0

    def addTask(self, task, smap, bmap, points, graph, tra=None):

        startx = 0
        starty = 0
        self.a=self.maxa
        if (len(self.tasks) == 0):
            startx = self.x
            starty = self.y
        else:
            startx = self.trajectories[-1][-1][0]
            starty = self.trajectories[-1][-1][1]
        if tra is None:
            tra = mh.globalPlanner(startx, starty, 0.5 * math.pi - self.direction, task[0], task[1], 0, bmap.info, smap)
        if len(self.tasks) == 0:
            self.trajectories = []
        goal = 0
        for i in range(len(tra) - 1):
            goal = goal + distan(tra[i][0], tra[i][1], tra[i + 1][0], tra[i + 1][1])
        self.trajectories.append(tra)
        self.goalDist = goal

        if len(self.tasks) == 0:
            self.rotating = True
            self.target = 0
            self.nextDist = distan(self.trajectories[0][self.target][0], self.trajectories[0][self.target][1],
                                   self.trajectories[0][self.target + 1][0], self.trajectories[0][self.target + 1][1])
            self.currentDist = 0
        self.tasks.append(task)
        self.moving = True


    def is_smal_dist(self):
        dist = distan(self.x, self.y, self.trajectories[0][self.target][0], self.trajectories[0][self.target][1])
        return self.treshV > dist

    def is_smalest_dist(self):
        dist = distan(self.x, self.y, self.trajectories[0][self.target][0], self.trajectories[0][self.target][1])
        return self.treshLast > dist


def controller(dx, dy, robot, pv, pw):
    theta = math.atan2(dy, dx)
    vtarget = dx * pv  # compute linear velocity
    lv = vtarget - pw * robot.radius * theta
    rv = vtarget + pw * robot.radius * theta
    lv, rv = scale(lv, rv, robot.maxv, robot.maxa, robot.t, robot.lv, robot.rv)
    return lv, rv


def scale(lv, rv, maxv, maxa, t, clv, crv):
    if abs(lv) >= abs(rv) and abs(lv) > maxv:
        rv = rv * maxv / abs(lv)
        lv = maxv * math.copysign(maxv, lv)
    elif abs(lv) < abs(rv) and abs(rv) > maxv:
        lv = lv * maxv / abs(rv)
        rv = maxv * math.copysign(maxv, rv)
    if abs(lv - clv) >= abs(rv - crv):
        if abs(lv - clv) > (maxa * t):
            rv = (rv - crv) * (maxa * t) / abs(lv - clv)
            lv = clv + math.copysign(maxa * t, lv - clv)
    else:
        if abs(rv - crv) > (maxa * t):
            lv = (lv - clv) * (maxa * t) / abs(rv - crv)
            rv = crv + math.copysign(maxa * t, rv - crv)
    return lv, rv


def computeState(mv, ma, step, v, a, distance, goal):
    difference = goal - distance
    slowingTime = mv / ma
    minSlowingRoad = mv * slowingTime / 2

    if a == 0:
        d = v * step
        if difference > (minSlowingRoad + d):
            return ['f'], [step]
        helper = difference - minSlowingRoad
        t1 = (helper / d) * step
        t2 = step - t1
        return ['f', 'd'], [t1, t2]
    if a < 0:
        v2 = v - step * ma
        if v2 > 0:
            # d = (v1 + v2) * t / 2
            return ['d'], [step]
        t1 = v / ma
        return ['d'], [t1]
    if a > 0:
        risingTime = (mv - v) / ma
        minRisingRoad = (mv + v) * risingTime / 2
        if difference > (minRisingRoad + minSlowingRoad):
            v2 = v + step * ma
            if v2 < mv:
                return ['a'], [step]
            t1 = (mv - v) / ma
            t2 = step - t1
            d1 = t1 * (v + mv) / 2
            d2 = t2 * mv
            if difference > (minSlowingRoad + d1 + d2):
                return ['a', 'f'], [t1, t2]
            helper = difference - minSlowingRoad - d1
            t2 = helper / d2
            t3 = step - t2 - t1
            return ['a', 'f', 'd'], [t1, t2, t3]
        preRisingTime = v / ma
        minPreRisingRoad = v * preRisingTime / 2
        helpDist = (difference + minPreRisingRoad) / 2
        v2 = v + step * ma
        d1 = step * (v + v2) / 2
        if (difference - d1) > helpDist:
            return ['a'], [step]
        if helpDist<0:
            helpDist=0
        helpTime = math.sqrt(2 * helpDist / ma)
        helpVel = ma * helpTime
        t1 = (helpVel - v) / ma
        t2 = step - t1
        v2 = v - t2 * ma
        if v2 > 0:
            # d = (v1 + v2) * t / 2
            return ['a', 'd'], [t1, t2]
        t2 = v / ma
        return ['a', 'd'], [t1, t2]


def computeVariables(states, times, v, ma, mv):
    d = 0
    for s, t in zip(states, times):
        if s == 'a':
            v1 = v
            v2 = v + t * ma
            d = d + (v1 + v2) * t / 2
            a = ma
            v = v2
        elif s == 'f':
            d = d + v * t
            a = 0
            v = mv
        elif s == 'd':
            v1 = v
            v2 = v - t * ma
            d = d + (v1 + v2) * t / 2
            a = -ma
            v = v2
    return d, v, a


def rotation(vx, vy, yaw):  # Rotate vector about chosen angle
    v = vx * math.cos(yaw) + vy * math.sin(yaw)
    w = - vx * math.sin(yaw) + vy * math.cos(yaw)
    return v, w


def distan(x1, y1, x2, y2):  # Compute distance between points
    dist = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    return dist


def positive_force(dif_x, dif_y, kp, positive_border):  # Compute postive force
    dist = distan(0, 0, dif_x, dif_y)
    if dist == 0:
        return 0, 0
    if dist > positive_border:
        force = positive_border * kp * 2.5
    else:
        force = dist * kp
    fx = dif_x / dist * force
    fy = dif_y / dist * force
    return fx, fy


def negative_force(robot, scan):  # TO DO
    forcex = 0
    forcey = 0
    for point in scan:
        dist = distan(robot.x, robot.y, point[0], point[1])
        force = robot.kn / dist ** 2 * (1 / robot.negativeBorder - 1 / dist)
        angle = math.atan2(point[1] - robot.y, point[0] - robot.x)
        forcex = forcex + force * math.cos(angle)
        forcey = forcey + force * math.sin(angle)
    return forcex, forcey


def is_small_angle(a1, a2, border):  # check if angle is smal (return True or False)
    diff = a2 - a1
    while (diff > math.pi):
        diff = diff - math.pi * 2
    while (diff < -math.pi):
        diff = diff + math.pi * 2
    if (diff < border) and (diff > -border):
        return True
    else:
        return False


def move(robots, lineMap, smallMap, bigMap, pub, people, points, graph):
    for robot in robots:
        if robot.moving:
            # if robot.rotating:
            #    robot.rotate()
            # else:
            robot.go(robots, lineMap, smallMap, bigMap, pub, people, points, graph)
            ''' try:
                    rospy.loginfo(robot.trajectories[0][robot.target])
                except:
                    rospy.loginfo('koniec trasy')'''

