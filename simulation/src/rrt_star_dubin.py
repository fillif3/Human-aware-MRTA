import copy
import math
import os
import random
import sys
from timeit import default_timer as timer

import matplotlib.pyplot as plt
import numpy as np
import mapHelper as mh

try:
    import dubin_path_planning
    from rrtStar import RRTStar
except ImportError:
    raise

show_animation = False
elapsed_time = []

def saveTimes(eva_time):
    cwd = os.getcwd()
    dir = cwd + '/pathTimes'
    os.system('mkdir '+ dir)
    text=''
    for t in eva_time:
        text=text+str(t)+','


    files = os.listdir(dir)
    n = len(files)

    dir=dir+'/p'+str(n)+'.txt'
    f = open(dir, "a")
    f.write(text)
    f.close()

class RRTStarReedsShepp(RRTStar):
    """
    Class for RRT star planning with Reeds Shepp path
    """

    class Node(RRTStar.Node):
        """
        RRT Node
        """

        def __init__(self, x, y, yaw):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None
            self.cost = 0.0
            self.yaw = yaw
            self.path_yaw = []

    def __init__(self, start, goal, obstacle_list, xlimits,ylimits,
                 max_iter=9200,
                 connect_circle_dist=500.0
                 ):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]
        """
        self.start = self.Node(start[0], start[1], start[2])
        self.end = self.Node(goal[0], goal[1], goal[2])
        self.min_rand_x = []
        self.max_rand_x = []
        self.min_rand_y = []
        self.max_rand_y = []
        state = -1
        goalState = -1
        for i in range(len(xlimits) - 1):
            self.min_rand_x.append(xlimits[i])
            self.max_rand_x.append(xlimits[i + 1])
            self.min_rand_y.append(ylimits[i][0])
            self.max_rand_y.append(ylimits[i][1])
            if start[0] > xlimits[i]:
                state = state + 1
            if goal[0] > xlimits[i]:
                goalState = goalState + 1
        self.state = state
        self.goalState = goalState

        self.forward = goal[0] > start[0]
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.connect_circle_dist = connect_circle_dist

        self.curvature = 0.1
        self.goal_yaw_th = np.deg2rad(3.0)
        self.goal_xy_th = 1

    def planning(self, animation=False, search_until_max_iter=False):
        """
        planning
        animation: flag for animation on or off
        """
        global elapsed_time
        self.node_list = [self.start]
        for i in range(self.max_iter):
            #start = timer()
            #print("Iter:", i, ", number of nodes:", len(self.node_list))
            rnd = self.get_random_node(self.obstacle_list)
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            new_node = self.steer(self.node_list[nearest_ind], rnd)

            if self.check_collision(new_node, self.obstacle_list):
                near_indexes = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, near_indexes)
                if new_node:
                    self.node_list.append(new_node)
                    self.rewire(new_node, near_indexes)
                    self.try_goal_path(new_node)

            if animation and i % 50 == 0:
                #self.plot_start_goal_arrow()
                self.draw_graph(rnd)
            #end = timer()
            #elapsed_time.append(end-start)
            if i>150 and new_node:  # check reaching the goal(not search_until_max_iter) 
                last_index = self.search_best_goal_node()
                if last_index:
                    #saveTimes(elapsed_time)
                    return self.generate_final_course(last_index)

        print("reached max iteration")

        last_index = self.search_best_goal_node()
        if last_index:
            return self.generate_final_course(last_index)
        else:
            print("Cannot find path")

        return None

    def try_goal_path(self, node):

        goal = self.Node(self.end.x, self.end.y, self.end.yaw)

        new_node = self.steer(node, goal)
        if new_node is None:
            return

        if self.check_collision(new_node, self.obstacle_list):
            self.node_list.append(new_node)

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        #for (ox, oy, size) in self.obstacle_list:
         #   plt.plot(ox, oy, "ok", ms=30 * size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([0, 1000, 0, 1000])
        plt.grid(True)
        #self.plot_start_goal_arrow()
        plt.pause(0.01)

    def plot_start_goal_arrow(self):
        dubin_path_planning.plot_arrow(
            self.start.x, self.start.y, self.start.yaw)
        reeds_shepp_path_planning.plot_arrow(
            self.end.x, self.end.y, self.end.yaw)

    def steer(self, from_node, to_node):

        px, py, pyaw, mode, course_lengths = dubin_path_planning.dubins_path_planning(
            from_node.x, from_node.y, from_node.yaw,
            to_node.x, to_node.y, to_node.yaw, self.curvature)


        if len(px) <= 1:  # cannot find a dubins path
            return None

        new_node = copy.deepcopy(from_node)
        #if len(px)>10:

        new_node.x = px[-1]
        new_node.y = py[-1]
        new_node.yaw = pyaw[-1]

        new_node.path_x = px
        new_node.path_y = py
        new_node.path_yaw = pyaw
        new_node.cost += course_lengths
        #new_node.cost += sum([abs(l) for l in course_lengths])
        new_node.parent = from_node

        return new_node

    def calc_new_cost(self, from_node, to_node):

        _, _, _, _, course_lengths = dubin_path_planning.dubins_path_planning(
            from_node.x, from_node.y, from_node.yaw,
            to_node.x, to_node.y, to_node.yaw, self.curvature)
        #if not course_lengths:
        #    return float("inf")


        return from_node.cost + course_lengths
    
    def get_random_node(self, obstacleList):

        while True:
            rnd = self.Node(random.uniform(self.min_rand_x[self.state]-25, self.max_rand_x[self.state]+25),
                            random.uniform(self.min_rand_y[self.state]-25, self.max_rand_y[self.state]+25),
                            random.uniform(-math.pi, math.pi))
            line = [[rnd.y, rnd.x]]
            if mh.safe(obstacleList, line):
                break
        return rnd
    
    def search_best_goal_node(self):

        goal_indexes = []
        for (i, node) in enumerate(self.node_list):
            if self.calc_dist_to_goal(node.x, node.y) <= self.goal_xy_th:
                goal_indexes.append(i)
        #print("goal_indexes:", len(goal_indexes))

        # angle check
        final_goal_indexes = []
        for i in goal_indexes:
            if abs(self.node_list[i].yaw - self.end.yaw) <= self.goal_yaw_th:
                final_goal_indexes.append(i)

        #print("final_goal_indexes:", len(final_goal_indexes))

        if not final_goal_indexes:
            return None

        min_cost = min([self.node_list[i].cost for i in final_goal_indexes])
        #print("min_cost:", min_cost)
        for i in final_goal_indexes:
            if self.node_list[i].cost == min_cost:
                return i

        return None

    def generate_final_course(self, goal_index):
        path = [[self.end.x, self.end.y, self.end.yaw]]
        node = self.node_list[goal_index]
        while node.parent:
            for (ix, iy, iyaw) in zip(reversed(node.path_x), reversed(node.path_y), reversed(node.path_yaw)):
                path.append([ix, iy, iyaw])
            node = node.parent
        path.append([self.start.x, self.start.y, self.start.yaw])
        return path


def startSearching(sx, sy, syaw, gx, gy, gyaw, xlimits,ylimits, obstacleList):
    global elapsed_time
    elapsed_time = []
    #print("start " + __file__)

    # ====Search Path with RRT====

    start = [sx, sy, syaw]
    goal = [gx, gy, gyaw]


    # Set Initial parameters


    rrt_star_reeds_shepp = RRTStarReedsShepp(start=start,
                  goal=goal,
                  xlimits=xlimits,
                  ylimits=ylimits,
                  obstacle_list=obstacleList)
    path = rrt_star_reeds_shepp.planning(animation=show_animation)

    # Draw final path
    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

        # path=list(reversed(path))
        path2 = list(reversed(path))
        #print(path2)

        # Draw final path

        return path2



if __name__ == '__main__':
    main()
