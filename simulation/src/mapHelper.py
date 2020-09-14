import numpy as np
import math
import rospy
import rrtStar
import rrt_star_dubin
from geometry_msgs.msg import PoseArray, Pose, Quaternion

def create_a_map(width, height, data):  # change a vector  map into Matrix map
    new_map = []
    row = []
    for i in range(height):
        for j in range(width):
            row.append(data[j + i * width])
        new_map.append(row)
        row = []
    return new_map


def small_index(map, width, height):  # finds small indexes, all points of the map are between them
    xmin = width - 1
    ymin = height - 1
    xmax = 0
    ymax = 0
    for i in range(height):
        for j in range(width):
            data = map[i][j]
            if data == 0:
                xmin = min(i, xmin)
                ymin = min(j, ymin)
                xmax = max(i, xmax)
                ymax = max(j, ymax)

    return xmin, ymin, xmax, ymax


def dilation(mapd, xmin, ymin, xmax, ymax, dist_in_meters, resolution):  # Makes obstacles (and unkown teretory) bigger
    dist_in_cells = int(dist_in_meters / resolution)  # compute dist in cells
    new_map = []

    new_map = list(map(list, mapd))

    for i in range(xmin, (xmax + 1)):
        for j in range(ymin, (ymax + 1)):
            if mapd[i][j] == 0:
                for k in range(i - dist_in_cells, i + dist_in_cells):
                    for l in range(j - dist_in_cells, j + dist_in_cells):
                        if k < xmin or k > (xmax - 1) or l < ymin or l > (
                                ymax - 1):  # if obstacle is close to the border of map
                            new_map[i][j] = 100
                            break
                        if not mapd[k][l] == 0:  # if obstacle is close to this cell
                            new_map[i][j] = 100
                            break
                    else:
                        # Continue if the inner loop wasn't broken.
                        continue
                    # Inner loop was broken, break the outer.
                    break

    return new_map


def random_points_on_map(mapa, xmin, ymin, xmax, ymax, step_in_m,
                         resolution):  # Creates points on the graph, later they are connected nto graph
    step_in_cell = int(step_in_m / resolution)
    points = []
    for i in range(xmin, xmax, step_in_cell):
        for j in range(ymin, ymax, step_in_cell):
            if mapa[i][j] == 0:
                point = [j, i]
                points.append(point)
    return points

def graph():
    return [[30,25],[30,225],[215,25],[215,225],[400,25],[400,225],[585,25],[585,215],[770,25],[770,215],[470,25],[470,215],
        [30,775],[30,575],[215,775],[215,575],[400,775],[400,575],[585,775],[585,585],[770,775],[770,585],[470,775],[470,585],
        [200,530],[200,270],[330,400],[70,400],[200+100,400+100],[200-100,400-100],[200-100,400+100],[200+100,400-100]]


def safe(map, line):  # Check if line is safe (line is safe if every point of the line does not connect with obstacle
    for point in line:
        try:
            if not map[int(point[0])][int(point[1])] == 0:
                return False
        except:
            print('poza mapa')
            return False
    return True


def check_con(list_of_points, mapd):  # Creates graph from list of points and mapd.
    # Check conection between points if there is no obstacle between them, the connection is saved as [[p1 index,p2 index, distance betwenn them]...]
    amount_of_points = len(list_of_points)
    connections = []
    for i in range(amount_of_points):
        for j in range((i + 1), amount_of_points):
            line = create_line(list_of_points[i][1], list_of_points[i][0], list_of_points[j][1],
                               list_of_points[j][0])  # Creates line
            if safe(mapd, line):  # if line is safe
                dist = math.sqrt((list_of_points[i][0] - list_of_points[j][0]) ** 2 + (
                        list_of_points[i][1] - list_of_points[j][1]) ** 2)  # compute distance
                con = [i, j, dist]
                connections.append(con)
    return connections


def create_line(x1, y1, x2, y2):  # Create set of points on the line (between two points [x1,y1] and [x2,y2]
    x1 = float(x1)
    y1 = float(y1)
    x2 = float(x2)
    y2 = float(y2)
    number_of_points = int(math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2))
    line = []
    for i in range(number_of_points):
        if x1 == x2:  # We do not want divide by zero
            x = x1
            y = int(round(y1 + i * ((y2 - y1) / number_of_points)))
            point = [x, y]
            line.append(point)
        else:  # Compute points from equation of a straight line
            a = (y1 - y2) / (x1 - x2);
            b = y1 - a * x1;
            x = x1 + i * ((x2 - x1) / number_of_points)
            y = int(round(a * x + b))
            x = int(round(x))
            point = [x, y]
            line.append(point)

    return line


def dijkstra(list_of_points, list_connects):
    # Args: list_of_points - matrix of points -> [[x1,y1],[x2,y2]... [xn,yn]] where n = amount of points and each point is described by x and y (last point is goal and first is start)
    #  list_connects - list of possible connection between points - each row is one connection and first two coulmns are ordinal number of points and 3r column is the distance

    m = True
    n = True
    amount_of_points_on_road = 1  # it will be increased later
    # iloscpol = len(list_connects)
    amount_points = len(list_of_points)
    maximum = amount_points - 1
    graph = [[0, 0, 0]]
    for i in range(amount_points):  # i=2:iloscpkt
        point_s = [1111, 0,
                   0]  # in Dijkstra algoritm infinity should be used but using 1111 is IMO good enough (we won't have such big numbers in algorithm)
        graph.append(point_s)

    # graph - in this matrix we have information about each point (row per point) in columns we have
    # 1st - shortest possible road found until now
    # 2nd - if this place was already considered - if yes: 1 else: 0
    # 3rd - from which  point the shortest path was found (ordinal number)
    where = 0
    # Actually considered point
    while (n):
        dist = 1111
        for i in range(amount_points):
            if (graph[i][0] <= dist) and (graph[i][1] == 0):
                # We look for point with the lowest distance AND which was not laready considered
                dist = graph[i][0]
                where = i
                # in this for we check which point should be considered
        graph[where][1] = 1  # After finding point, we write that was already considered so we won't choose it again
        if (where == maximum):  # if we consider goal point
            while (m):
                amount_of_points_on_road = amount_of_points_on_road + 1  # We check how many point will road have
                where = graph[where][2]
                if (where == 0):
                    m = False  # After findind shortest road, we leave loop
                    n = False
            where = maximum
        else:  # if it is normal point
            for pol in list_connects:
                if ((pol[0] == where) and (((pol[2]+(pol[2]*pol[2]+1)*0.01 )+ graph[where][0]) <= graph[pol[1]][0])):  # looking for connections with considered point BUT only to the point which has longer path than sum of path to the considered point and distance to the connected point
                    graph[pol[1]][2] = where  # we save considered point in the graph
                    graph[pol[1]][0] = pol[2] + graph[where][0]  # we save distance
                # Same but for the 2nd column of list_connects
                if ((pol[1] == where) and (((pol[2]+(pol[2]*pol[2]+1)*0.01 ) + graph[where][0]) <= graph[pol[0]][0])):
                    graph[pol[0]][2] = where
                    graph[pol[0]][0] = pol[2] + graph[where][0]

    order = [0] * amount_of_points_on_road
    for i in range(amount_of_points_on_road):
        order[amount_of_points_on_road - i - 1] = where
        where = graph[where][2]
    # in loop we look for the order of points
    out = []
    for i in range(amount_of_points_on_road):
        out.append(list_of_points[
                       order[i]])  # We creating new road using save order and list of points (it is input of function)

    return out


def trajectory(road, averge_velocity):
    # Creates smooth trajectory, basing on road and averge_velocity
    traject = []
    angle = math.atan2(road[1][0] - road[0][0], road[1][1] - road[0][1])
    vpx = math.sin(angle)*averge_velocity
    vpy = math.cos(angle)*averge_velocity
    for i in range(len(road) - 1):
        x1 = float(road[i][0])  # current point
        y1 = float(road[i][1])  # current point
        x2 = float(road[i + 1][0])  # Next point
        y2 = float(road[i + 1][1])  # Next point
        if i == (len(road) - 2):  # If we go to the last pair of points, velocity is zero
            vkx = 0
            vky = 0
        else:  # else, compute velocity basing on the next destination
            x3 = float(road[i + 2][0])
            y3 = float(road[i + 2][1])
            angle = math.atan2(x2 - x1, y2 - y1)
            angle2 = math.atan2(x3 - x2, y3 - y2)
            new_angle = (angle2 + angle) / 2
            vkx = averge_velocity * 0.5 * math.sin(new_angle)
            vky = averge_velocity * 0.5 * math.cos(new_angle)

        # In this part we compute best parameters of 3rd degree polyminail to find smooth trajectory
        t = float(math.sqrt(((x1 - x2) ** 2 + (y1 - y2) ** 2)) / float(averge_velocity))
        tmatrix = np.array([[0.0, 0.0, 0.0, 1.0], [t ** 3, t ** 2, t, 1.0], [0.0, 0.0, 1.0, 0.0],
                            [3 * t ** 2, 2 * t, 1.0,
                             0.0]])  # Computed based on equesion of 3rd degree polyminail and its derietive
        tmatrixinverse = np.linalg.inv(tmatrix)
        pmatrix = np.array([[x1, y1], [x2, y2], [vpx, vpy], [vkx, vky]])  # Target matrix
        vpx = vkx  # Next starting velocit iscurrent stoping velocity
        vpy = vky
        parameters = tmatrixinverse.dot(pmatrix)

        for i in range(int(t * 5)):  # Trajectory is discretized
            discrete_time = float(float(i) / 5)
            x = discrete_time ** 3 * float(parameters[0][0]) + discrete_time ** 2 * float(
                parameters[1][0]) + discrete_time * float(parameters[2][0]) + float(parameters[3][0])
            y = discrete_time ** 3 * float(parameters[0][1]) + discrete_time ** 2 * float(
                parameters[1][1]) + discrete_time * float(parameters[2][1]) + float(parameters[3][1])
            traject.append([x, y])
    return traject


def from_index_to_real_point(x, y, mapinfo):  # Change map's index value (for instane 144, 2731) to real value
    real_x = x * mapinfo.resolution + mapinfo.origin.position.x
    real_y = y * mapinfo.resolution + mapinfo.origin.position.y
    return real_x, real_y


def from_real_point_to_index(x, y, mapinfo):  # Change real value (for instane 2.55, 3.15) to index (int) value
    grid_x = int((x - mapinfo.origin.position.x) / mapinfo.resolution)
    grid_y = int((y - mapinfo.origin.position.y) / mapinfo.resolution)
    return grid_x, grid_y

def add_connections(list_of_points, mapd, connections):  # Creates graph from list of points and mapd.

    # Check conection between points if there is no obstacle between them, the connection is saved as [[p1 index,p2 index, distance betwenn them]...]
    amount_of_points = len(list_of_points)
    for i in range(amount_of_points):
        for j in range((i + 1), amount_of_points):
            if i == 0 or j == (amount_of_points - 1):
                line = create_line(list_of_points[i][1], list_of_points[i][0], list_of_points[j][1],
                                   list_of_points[j][0])  # Creates line
                if safe(mapd, line):  # if line is safe
                    dist = math.sqrt((list_of_points[i][0] - list_of_points[j][0]) ** 2 + (
                            list_of_points[i][1] - list_of_points[j][1]) ** 2)  # compute distance
                    con = [i, j, dist]
                    if i == 0:
                        connections.insert(j-1,con)
                    else:
                        connections.append(con)
    return connections

def globalPlanner(startx,starty,startyaw, endx,endy,endyaw, info, small_map):  # When destination received, smooth trajectory is computed
    #random_points = random_points_on_map(small_map, 0, 0, info.height, info.width, 2.5,info.resolution)
    #random_points =graph()                       #               # Create points on the map
    # Change those points to real values
    '''real_random_points = []
    for point in random_points:
        x, y = from_index_to_real_point(point[0], point[1],info)
        real_random_points.append([x, y])
    posea = PoseArray()
    for point in real_random_points:
        help_pose = Pose()
        help_pose.position.x = point[0]
        help_pose.position.y = point[1]
        help_pose.position.z = 0
        help_pose.orientation.x = 1
        help_pose.orientation.y = 1
        help_pose.orientation.z = 1
        help_pose.orientation.w = 1

        posea.poses.append(help_pose)

    posea.header.frame_id = "/map"
    pub.publish(posea)
    new_data=[]
    for i in range(info.width):
        for j in range(info.height):
           new_data.append(small_map[i][j])
    small_map_msg = bmap
    small_map_msg.data = new_data # In this msg, program will send dilatated map
    rospy.sleep(10)
    pub.publish(small_map_msg)'''
    # Add destination point to list of points
    xe, ye = from_real_point_to_index(endx, endy, info)
    #random_points.append([x, y])
    xs, ys = from_real_point_to_index(startx, starty,info)
    #random_points.insert(0, [x, y])

    #con_between_points = check_con(random_points, small_map)  # Create graph with all points
    #con_between_points = add_connections(random_points, small_map,con_between_points)  # Create graph with all points
    #road = dijkstra(random_points, con_between_points)  # Find the shortest path in the graph
    ylimits = [[63,596]]#[[279,337] ,[214,596],[120,395],[63,191]]
    xlimits = [30,942]#[30,190,530,800,942]
    road = rrt_star_dubin.startSearching(xs,ys,startyaw,xe,ye,endyaw,xlimits,ylimits,small_map)
    # Change path into real value
    real_road = []
    for point in road:
        x, y = from_index_to_real_point(point[0], point[1], info)
        real_road.append([x, y])
    return real_road
    #if smooth==False:
     #   return real_road
    # Compute smooth trajectory
    tra = trajectory(real_road, avgvel)
    return tra

