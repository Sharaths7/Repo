import heapq
import scipy.spatial
import numpy as np
import math
import matplotlib.pyplot as plt
import sys
from a_star_heuristic import dp_planning  # , calc_obstacle_map
import reeds_shepp_path as rs
from harvey import move, check_harvey_collision, MAX_STEER, WB, plot_harvey, plot_arrow
from parameters import *
import time
from math import sqrt, cos, sin, tan, pi
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler




import plotly.express as pl
import plotly.graph_objects as go

class Node:

    def __init__(self, xind, yind, yawind, direction,
                 xlist, ylist, yawlist, directions,
                 steer=0.0, pind=None, cost=None):
        self.xind = xind
        self.yind = yind
        self.yawind = yawind
        self.direction = direction
        self.xlist = xlist
        self.ylist = ylist
        self.yawlist = yawlist
        self.directions = directions
        self.steer = steer
        self.pind = pind
        self.cost = cost


class Path:

    def __init__(self, xlist, ylist, yawlist, directionlist, cost):
        self.xlist = xlist
        self.ylist = ylist
        self.yawlist = yawlist
        self.directionlist = directionlist
        self.cost = cost


class KDTree:
    """
    Nearest neighbor search class with KDTree
    """

    def __init__(self, data):
        # store kd-tree
        # used cKDTree instead of KDTree for faster querying
        self.tree = scipy.spatial.cKDTree(data)

    def search(self, inp, k=1):
        """
        Search NN
        inp: input data, single frame or multi frame
        """

        if len(inp.shape) >= 2:  # multi input
            index = []
            dist = []

            for i in inp.T:
                idist, iindex = self.tree.query(i, k=k)
                index.append(iindex)
                dist.append(idist)

            return index, dist

        dist, index = self.tree.query(inp, k=k)
        return index, dist

    def search_in_distance(self, inp, r):
        """
        find points with in a distance r
        """

        index = self.tree.query_ball_point(inp, r)
        return index


class Config:

    def __init__(self, ox, oy, xyreso, yawreso):
        min_x_m = 0
        min_y_m = 0
        max_x_m = 100
        max_y_m = 100

        ox.append(min_x_m)
        oy.append(min_y_m)
        ox.append(max_x_m)
        oy.append(max_y_m)

        self.minx = 0
        self.miny = 0
        self.maxx = round(max_x_m / xyreso)
        self.maxy = round(max_y_m / xyreso)

        self.xw = round(self.maxx - self.minx)
        self.yw = round(self.maxy - self.miny)

        self.minyaw = round(- math.pi / yawreso) - 1
        self.maxyaw = round(math.pi / yawreso)
        self.yaww = round(self.maxyaw - self.minyaw)


def calc_motion_inputs():

    for steer in np.concatenate((np.linspace(-MAX_STEER, MAX_STEER, N_STEER),[0.0])):
        for d in [1, -1]:
            yield [steer, d]


def get_neighbors(current, config, ox, oy, kdtree):

    for steer, d in calc_motion_inputs():
        node = calc_next_node(current, steer, d, config, ox, oy, kdtree)
        if node and verify_index(node, config):
            yield node


def calc_next_node(current, steer, direction, config, ox, oy, kdtree):

    x, y, yaw = current.xlist[-1], current.ylist[-1], current.yawlist[-1]

    arc_l = XY_GRID_RESOLUTION * 1.5
    xlist, ylist, yawlist = [], [], []
    for _ in np.arange(0, arc_l, MOTION_RESOLUTION):
        x, y, yaw = move(x, y, yaw, MOTION_RESOLUTION * direction, steer)
        xlist.append(x)
        ylist.append(y)
        yawlist.append(yaw)
    if not check_harvey_collision(xlist, ylist, yawlist, ox, oy, kdtree):
        return None

    d = direction == 1
    xind = round(x / XY_GRID_RESOLUTION)
    yind = round(y / XY_GRID_RESOLUTION)
    yawind = round(yaw / YAW_GRID_RESOLUTION)

    addedcost = 0.0

    if d != current.direction:
        addedcost += SB_COST

    # steer penalty
    addedcost += STEER_COST * abs(steer)

    # steer change penalty
    addedcost += STEER_CHANGE_COST * abs(current.steer - steer)

    cost = current.cost + addedcost + arc_l

    node = Node(xind, yind, yawind, d, xlist,
                ylist, yawlist, [d],
                pind=calc_index(current, config),
                cost=cost, steer=steer)

    return node


def is_same_grid(n1, n2):
    if n1.xind == n2.xind and n1.yind == n2.yind and n1.yawind == n2.yawind:
        return True
    return False


def analytic_expansion(current, goal, c, ox, oy, kdtree):

    sx = current.xlist[-1]
    sy = current.ylist[-1]
    syaw = current.yawlist[-1]

    gx = goal.xlist[-1]
    gy = goal.ylist[-1]
    gyaw = goal.yawlist[-1]

    max_curvature = math.tan(MAX_STEER) / WB
    paths = rs.calc_paths(sx, sy, syaw, gx, gy, gyaw,
                          max_curvature, step_size=MOTION_RESOLUTION)

    if not paths:
        return None

    best_path, best = None, None

    for path in paths:
        # plt.cla()
        # plt.plot(ox, oy, ".k")
        # plt.plot(path.x, path.y, "-b", label="Evaluation")
        # plt.grid(True)
        # plt.axis("equal")
        # # plot_harvey(path.x, path.y, path.yaw)
        # plt.pause(0.01)
        # plt.scatter(path.x, path.y)
        if check_harvey_collision(path.x, path.y, path.yaw, ox, oy, kdtree):
            cost = calc_rs_path_cost(path)
            if not best or best > cost:
                best = cost
                best_path = path


    return best_path


def update_node_with_analystic_expantion(current, goal,
                                         c, ox, oy, kdtree):
    apath = analytic_expansion(current, goal, c, ox, oy, kdtree)

    if apath:
        # plt.plot(apath.x, apath.y)
        fx = apath.x[1:]
        fy = apath.y[1:]
        fyaw = apath.yaw[1:]

        fcost = current.cost + calc_rs_path_cost(apath)
        fpind = calc_index(current, c)

        fd = []
        for d in apath.directions[1:]:
            fd.append(d >= 0)

        fsteer = 0.0
        fpath = Node(current.xind, current.yind, current.yawind,
                     current.direction, fx, fy, fyaw, fd,
                     cost=fcost, pind=fpind, steer=fsteer)
        return True, fpath

    return False, None


def calc_rs_path_cost(rspath):

    cost = 0.0
    for l in rspath.lengths:
        if l >= 0:  # forward
            cost += l
        else:  # back
            cost += abs(l) * BACK_COST

    # swich back penalty
    for i in range(len(rspath.lengths) - 1):
        if rspath.lengths[i] * rspath.lengths[i + 1] < 0.0:  # switch back
            cost += SB_COST

    # steer penalyty
    for ctype in rspath.ctypes:
        if ctype != "S":  # curve
            cost += STEER_COST * abs(MAX_STEER)

    # ==steer change penalty
    # calc steer profile
    nctypes = len(rspath.ctypes)
    ulist = [0.0] * nctypes
    for i in range(nctypes):
        if rspath.ctypes[i] == "R":
            ulist[i] = - MAX_STEER
        elif rspath.ctypes[i] == "L":
            ulist[i] = MAX_STEER

    for i in range(len(rspath.ctypes) - 1):
        cost += STEER_CHANGE_COST * abs(ulist[i + 1] - ulist[i])

    return cost


def hybrid_a_star_planning(start, goal, ox, oy, xyreso, yawreso):
    """
    start
    goal
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    xyreso: grid resolution [m]
    yawreso: yaw angle resolution [rad]
    """

    start[2], goal[2] = rs.pi_2_pi(start[2]), rs.pi_2_pi(goal[2])
    tox, toy = ox[:], oy[:]

    obkdtree = KDTree(np.vstack((tox, toy)).T)

    config = Config(tox, toy, xyreso, yawreso)

    nstart = Node(round(start[0] / xyreso), round(start[1] / xyreso), round(start[2] / yawreso),
                  True, [start[0]], [start[1]], [start[2]], [True], cost=0)
    ngoal = Node(round(goal[0] / xyreso), round(goal[1] / xyreso), round(goal[2] / yawreso),
                 True, [goal[0]], [goal[1]], [goal[2]], [True])

    openList, closedList = {}, {}

    _, _, h_dp = dp_planning(nstart.xlist[-1], nstart.ylist[-1],
                             ngoal.xlist[-1], ngoal.ylist[-1], ox, oy, xyreso, VR)

    pq = []
    openList[calc_index(nstart, config)] = nstart
    heapq.heappush(pq, (calc_cost(nstart, h_dp, ngoal, config),
                        calc_index(nstart, config)))

    while True:
        if not openList:
            print("Error: Cannot find path, No open set")
            return [], [], []

        cost, c_id = heapq.heappop(pq)
        if c_id in openList:
            current = openList.pop(c_id)
            closedList[c_id] = current
        else:
            continue

        # if show_animation:  # pragma: no cover
        #     plt.plot(current.xlist[-1], current.ylist[-1], "xc")
        #     # for stopping simulation with the esc key.
        #     plt.gcf().canvas.mpl_connect('key_release_event',
        #             lambda event: [exit(0) if event.key == 'escape' else None])
        #     if len(closedList.keys()) % 10 == 0:
        #         plt.pause(0.1)

        isupdated, fpath = update_node_with_analystic_expantion(
            current, ngoal, config, ox, oy, obkdtree)

        if isupdated:
            break

        for neighbor in get_neighbors(current, config, ox, oy, obkdtree):
            neighbor_index = calc_index(neighbor, config)
            if neighbor_index in closedList:
                continue
            if neighbor not in openList \
                    or openList[neighbor_index].cost > neighbor.cost:
                heapq.heappush(
                    pq, (calc_cost(neighbor, h_dp, ngoal, config),
                         neighbor_index))
                openList[neighbor_index] = neighbor

    path = get_final_path(closedList, fpath, nstart, config)
    return path


def calc_cost(n, h_dp, goal, c):
    ind = (n.yind - c.miny) * c.xw + (n.xind - c.minx)
    if ind not in h_dp:
        return n.cost + 999999999  # collision cost
    return n.cost + H_COST * h_dp[ind].cost


def get_final_path(closed, ngoal, nstart, config):
    rx, ry, ryaw = list(reversed(ngoal.xlist)), list(
        reversed(ngoal.ylist)), list(reversed(ngoal.yawlist))
    direction = list(reversed(ngoal.directions))
    nid = ngoal.pind
    finalcost = ngoal.cost

    while nid:
        n = closed[nid]
        rx.extend(list(reversed(n.xlist)))
        ry.extend(list(reversed(n.ylist)))
        ryaw.extend(list(reversed(n.yawlist)))
        direction.extend(list(reversed(n.directions)))

        nid = n.pind

    rx = list(reversed(rx))
    ry = list(reversed(ry))
    ryaw = list(reversed(ryaw))
    direction = list(reversed(direction))

    # adjust first direction
    direction[0] = direction[1]

    path = Path(rx, ry, ryaw, direction, finalcost)

    return path


def verify_index(node, c):
    xind, yind = node.xind, node.yind
    if xind >= c.minx and xind <= c.maxx and yind >= c.miny \
            and yind <= c.maxy:
        return True

    return False


def calc_index(node, c):
    ind = (node.yawind - c.minyaw) * c.xw * c.yw + \
        (node.yind - c.miny) * c.xw + (node.xind - c.minx)

    if ind <= 0:
        print("Error(calc_index):", ind)

    return ind
def unit_testing(ox,oy):
    startList = [[10.0, 20.0, np.deg2rad(90.0)],[30.0, 40.0, np.deg2rad(90.0)]]
    goalList = [[50.0, 50.0, np.deg2rad(90.0)], [50.0, 15.0, np.deg2rad(-90.0)]]
    plt.plot(ox, oy, ".k")
    for start in startList:
        for goal in goalList:
            print('Start Location:  ',start)
            print('Goal location:    ',goal)
            # print(start)
            # print('!!!!',goal)
            rs.plot_arrow(start[0], start[1], start[2], fc='g')
            rs.plot_arrow(goal[0], goal[1], goal[2])
            path = hybrid_a_star_planning(
            start, goal, ox, oy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)

            x = path.xlist
            y = path.ylist
            yaw = path.yawlist
            plot_trajectory_and_motion(ox, oy,x, y, yaw)

def unit_testing_local_planner(ox,oy):
    startList = [[30.0, 20.0, np.deg2rad(90.0)]]
    goalList = [[30.0, 35.0, np.deg2rad(90.0)], [35.0, 35.0, np.deg2rad(90.0)],[25.0, 35.0, np.deg2rad(90.0)],[40.0, 35.0, np.deg2rad(90.0)],[20.0, 35.0, np.deg2rad(90.0)],[45.0, 35.0, np.deg2rad(90.0)],[15.0, 35.0, np.deg2rad(90.0)],[50.0, 35.0, np.deg2rad(90.0)],[10.0, 35.0, np.deg2rad(90.0)]]
    plt.plot(ox, oy, ".k")
    for start in startList:
        for goal in goalList:
            print("Start Location:  ",start)
            print('Goal location:    ',goal)

            # print(start)
            # print('!!!!',goal)
            rs.plot_arrow(start[0], start[1], start[2], fc='g')
            rs.plot_arrow(goal[0], goal[1], goal[2])
            path = hybrid_a_star_planning(
            start, goal, ox, oy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)

            x = path.xlist
            y = path.ylist
            yaw = path.yawlist
            plot_trajectory_and_motion(ox, oy,x, y, yaw)

def unit_testing_parking(ox,oy):
    startList = [[10.0, 30.0, np.deg2rad(0)]]
    goalList = [[10.0, 50.0, np.deg2rad(0)]]
    plt.plot(ox, oy, ".k")
    for start in startList:
        for goal in goalList:
            print("Start Location:  ",start)
            print('Goal location:    ',goal)

            # print(start)
            # print('!!!!',goal)
            rs.plot_arrow(start[0], start[1], start[2], fc='g')
            rs.plot_arrow(goal[0], goal[1], goal[2])
            path = hybrid_a_star_planning(
            start, goal, ox, oy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)

            x = path.xlist
            y = path.ylist
            yaw = path.yawlist
            plot_trajectory_and_motion(ox, oy,x, y, yaw)

def create_world():
    x, y = [], []
    for i in range(60):
        x.append(i)
        y.append(0.0)
    for i in range(60):
        x.append(60.0)
        y.append(i)
    for i in range(61):
        x.append(i)
        y.append(60.0)
    for i in range(61):
        x.append(0.0)
        y.append(i)
    for i in range(40):
        x.append(20.0)
        y.append(i)
    for i in range(40):
        x.append(40.0)
        y.append(60.0 - i)
    return x, y

def create_world_local_planner():
    x, y = [], []
    for i in range(60):
        x.append(i)
        y.append(0.0)
    for i in range(60):
        x.append(60.0)
        y.append(i)
    for i in range(61):
        x.append(i)
        y.append(60.0)
    for i in range(61):
        x.append(0.0)
        y.append(i)
    for i in range(40):
        x.append(i)
        y.append(45)
    return x, y

def plot_arrow(x, y, yaw, arrow_length=1.0,
               origin_point_plot_style="xr",
               head_width=0.1, fc="r", ec="k", **kwargs):
    if not isinstance(x, float):
        for (i_x, i_y, i_yaw) in zip(x, y, yaw):
            plot_arrow(i_x, i_y, i_yaw, head_width=head_width,
                       fc=fc, ec=ec, **kwargs)
    else:
        plt.arrow(x, y,
                  arrow_length * math.cos(yaw),
                  arrow_length * math.sin(yaw),
                  head_width=head_width,
                  fc=fc, ec=ec,
                  **kwargs)
        if origin_point_plot_style is not None:
            plt.plot(x, y, origin_point_plot_style)



def plot_trajectory_and_motion(ox, oy,x, y, yaw):
    for ix, iy, iyaw in zip(x, y, yaw):
        plt.cla()
        plt.plot(ox, oy, ".k")
        plt.plot(x, y, "-r", label="Hybrid A* path")
        plt.grid(True)
        plt.axis("equal")
        plot_harvey(ix, iy, iyaw)
        # c, s = cos(yaw), sin(yaw)

        # arrow_x, arrow_y, arrow_yaw = c * 1.5 + ix, s * 1.5 + iy, iyaw

        # plot_arrow(arrow_x, arrow_y, arrow_yaw)
        plt.pause(0.00000001)        
    plt.scatter(x,y)
    plt.show()
    

class Error(Exception):
    pass

class unknownerror(Error):
    pass

def main():
    
    ox, oy = [], []
    start = [10,20,90]
    goal = [10, 100,-90]
    goal[2] = np.deg2rad(goal[2])
    print('Goal location:    ',goal)
    start_time = time.time()
    path = hybrid_a_star_planning(
        start, goal, ox, oy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)
    x = list(path.xlist)
    y = list(path.ylist)
    yaw = list(path.yawlist)
    print("--- %s seconds ---" % (time.time() - start_time))
    end = list(zip(x,y,yaw))
    print(end)
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=ox, y=oy,
                    mode='markers',
                    name='Boundary',
                    fill='tozeroy'
                    ))
                   

    fig.add_trace(go.Scatter(x=x, y=y,
                    mode='markers',
                    name='Waypoints',
                    
                    ))
            
    fig.show()
    plot_trajectory_and_motion(ox, oy,x, y, yaw)
    print(__file__ + " done!!")


if __name__ == '__main__':

    print("Start Hybrid A* planning")
    start = True
    while  start:
        start = main()
