"""
Path planning Sample Code with RRT*
author: Atsushi Sakai(@Atsushi_twi)
"""

import math
import os
import sys
import random
import dubins
import mapHelper as mh


import matplotlib.pyplot as plt

try:
    from rrt import RRT
except ImportError:
    raise

show_animation = True


class RRTStar(RRT):
    """
    Class for RRT Star planning
    """

    class Node(object):
        def __init__(self, x, y, yaw):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None
            self.cost = 0.0
            self.yaw = yaw
            # self.path_yaw = []

    def __init__(self, start, goal, obstacle_list, xlimits,ylimits,
                 expand_dis=2.0,
                 path_resolution=0.10,
                 goal_sample_rate=20,
                 max_iter=13000,
                 connect_circle_dist=50.0
                 ):
        super(RRTStar, self).__init__(start, goal, obstacle_list,
                                      xlimits,ylimits, expand_dis, path_resolution, goal_sample_rate, max_iter)
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]
        """
        self.connect_circle_dist = connect_circle_dist
        self.goal_node = self.Node(goal[0], goal[1], goal[2])

    def get_random_node(self, obstacleList):
        if random.randint(0, 100) > self.goal_sample_rate:
            while True:
                rnd = self.Node(random.uniform(self.min_rand_x[self.state]-25, self.max_rand_x[self.state]+25),
                                random.uniform(self.min_rand_y[self.state]-25, self.max_rand_y[self.state]+25),
                                random.uniform(-math.pi, math.pi))
                line = [[rnd.y, rnd.x]]
                if mh.safe(obstacleList, line):
                    break
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y, self.end.yaw)
        return rnd

    @staticmethod
    def check_collision(node, obstacleList):

        if node is None:
            return False
        dy_list = [x for x in node.path_x]
        dx_list = [y for y in node.path_y]
        line = []
        for (dx, dy) in zip(dx_list, dy_list):
            point = [dx, dy]
            line.append(point)
        # k=mh.safe(obstacleList, line)
        return mh.safe(obstacleList, line)

    def steer(self, from_node, to_node, extend_length=float("inf")):

        d, theta = self.calc_distance_and_angle(from_node, to_node)
        # new_node.path_x = [new_node.x]
        # new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d
        new_node = self.Node(from_node.x + extend_length * math.cos(theta),
                             from_node.y + extend_length * math.sin(theta), to_node.yaw)
        turning_radius = 0.4
        step_size = 0.1
        path = dubins.shortest_path([from_node.x, from_node.y, from_node.yaw], [new_node.x, new_node.y, new_node.yaw],
                                    turning_radius)
        configurations, _ = path.sample_many(step_size)
        for x, y, _ in configurations:
            new_node.path_x.append(x)
            new_node.path_y.append(y)
        '''
        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)

        new_node.parent = from_node
        '''
        return new_node

    def planning(self, animation=True, search_until_max_iter=True):
        """
        rrt star path planning
        animation: flag for animation on or off
        search_until_max_iter: search until max iteration for path improving or not
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            print("Iter:", i, ", number of nodes:", len(self.node_list))
            rnd = self.get_random_node(self.obstacle_list)
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            new_node = self.steer(self.node_list[nearest_ind], rnd, self.expand_dis)

            if self.check_collision(new_node, self.obstacle_list):
                near_inds = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, near_inds)
                if new_node:
                    self.node_list.append(new_node)


                    if self.state!=self.goalState:

                        if self.forward:
                            if new_node.x>self.max_rand_x[self.state]:
                                self.state=self.state+1
                        else:
                            if new_node.x<self.min_rand_x[self.state]:
                                self.state=self.state-1

                    self.rewire(new_node, near_inds)

            if animation and i % 50 == 0:
                if i > 1000:
                    self.draw_graph(rnd)#self.draw_graph(rnd, True)
                else:
                    self.draw_graph(rnd)

            if (i > 500) and new_node:  # check reaching the goal
                last_index = self.search_best_goal_node()
                if last_index:
                    return self.generate_final_course(last_index)

        print("reached max iteration")

        last_index = self.search_best_goal_node()
        if last_index:
            return self.generate_final_course(last_index)

        return None

    def choose_parent(self, new_node, near_inds):
        if not near_inds:
            return None

        # search nearest cost in near_inds
        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)
            if t_node and self.check_collision(t_node, self.obstacle_list):
                costs.append(self.calc_new_cost(near_node, t_node))
            else:
                costs.append(float("inf"))  # the cost of collision node
        min_cost = min(costs)

        if min_cost == float("inf"):
            print("There is no good path.(min_cost is inf)")
            return None

        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.parent = self.node_list[min_ind]
        new_node.cost = min_cost

        return new_node

    def search_best_goal_node(self):
        dist_to_goal_list = [self.calc_dist_to_goal(n.x, n.y) for n in self.node_list]
        goal_inds = [dist_to_goal_list.index(i) for i in dist_to_goal_list if i <= 5 * self.expand_dis]

        safe_goal_inds = []
        for goal_ind in goal_inds:
            t_node = self.steer(self.node_list[goal_ind], self.goal_node)
            if self.check_collision(t_node, self.obstacle_list):
                safe_goal_inds.append(goal_ind)

        if not safe_goal_inds:
            return None

        min_cost = min([self.node_list[i].cost for i in safe_goal_inds])
        for i in safe_goal_inds:
            if self.node_list[i].cost == min_cost:
                return i

        return None

    def find_near_nodes(self, new_node):
        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * math.sqrt((math.log(nnode) / nnode))
        # if expand_dist exists, search vertices in a range no more than expand_dist
        if hasattr(self, 'expand_dis'):
            r = min(r, self.expand_dis)
        dist_list = [(node.x - new_node.x) ** 2 +
                     (node.y - new_node.y) ** 2 for node in self.node_list]
        near_inds = [dist_list.index(i) for i in dist_list if i <= r ** 2]
        return near_inds

    def rewire(self, new_node, near_inds):
        for i in near_inds:
            near_node = self.node_list[i]
            edge_node = self.steer(new_node, near_node)
            if not edge_node:
                continue
            edge_node.cost = self.calc_new_cost(new_node, edge_node)

            no_collision = self.check_collision(edge_node, self.obstacle_list)
            improved_cost = near_node.cost > edge_node.cost

            if no_collision and improved_cost:
                self.node_list[i] = edge_node
                self.propagate_cost_to_leaves(new_node)

    def calc_new_cost(self, from_node, t_node):
        cost = from_node.cost
        for i in range(len(t_node.path_x) - 1):
            cost = cost + math.sqrt(
                ((t_node.path_x[i]) - (t_node.path_x[i + 1])) ** 2 + ((t_node.path_y[i]) - (t_node.path_y[i + 1])) ** 2)
        return cost

    def propagate_cost_to_leaves(self, parent_node):

        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)


def startSearching(sx, sy, syaw, gx, gy, gyaw, xlimits,ylimits, obstacleList):
    print("start " + __file__)

    # ====Search Path with RRT====
    '''obstacleList = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2),
        (8, 10, 1)
    ]  # [x, y, radius]
    # Set Initial parameters
    '''
    row = []
    col = []
    for i in range(len(obstacleList)):
        for j in range(len(obstacleList[0])):

            if obstacleList[i][j] == 0:
                row.append(i)
                col.append(j)

    plt.plot(col, row, 'k*')
    #plt.hold(True)
    rrt = RRTStar(start=[sx, sy, syaw],
                  goal=[gx, gy, gyaw],
                  xlimits=xlimits,
                  ylimits=ylimits,
                  obstacle_list=obstacleList)
    path = rrt.planning(animation=show_animation)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

        # path=list(reversed(path))
        path2 = list(reversed(path))
        print(path2)

        # Draw final path

        return path2


if __name__ == '__main__':
    main()

