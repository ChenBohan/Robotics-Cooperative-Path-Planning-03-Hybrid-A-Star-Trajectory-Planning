import numpy as np
import vehicle
import planner
import vehicle_lib
import rs_path
import grid_a_star
from scipy import spatial
import math
import Queue
from sklearn import neighbors
import scipy.spatial
import matplotlib.pyplot as plt

XY_GRID_RESOLUTION = 2.0  # [m]
YAW_GRID_RESOLUTION = np.deg2rad(15.0)  # [rad]
GOAL_TYAW_TH = np.deg2rad(5.0)  # [rad]
MOTION_RESOLUTION = 0.1  # [m] path interporate resolution
N_STEER = 20.0  # number of steer command
EXTEND_AREA = 5.0  # [m] map extend length
SKIP_COLLISION_CHECK = 4  # skip number for collision check

SB_COST = 100.0  # switch back penalty cost
BACK_COST = 5.0  # backward penalty cost
STEER_CHANGE_COST = 5.0  # steer angle change penalty cost
STEER_COST = 1.0  # steer angle change penalty cost
JACKKNIF_COST = 200.0  # Jackknif cost
H_COST = 5.0  # Heuristic cost

WB = vehicle.WB  # [m] Wheel base
MAX_STEER = vehicle.MAX_STEER  # [rad] maximum steering angle

class Node(object):

    # xind::Int64  # x index
    # yind::Int64  # y index
    # yawind::Int64  # yaw index
    # direction::Bool  # moving direction forword:true, backword:false
    # x::Array
    # {Float64}  # x position [m]
    # y::Array
    # {Float64}  # y position [m]
    # yaw::Array
    # {Float64}  # yaw angle [rad]
    # yaw1::Array
    # {Float64}  # trailer yaw angle [rad]
    # directions::Array
    # {Bool}  # directions of each points forward: true, backward:false
    # steer::Float64  # steer input
    # cost::Float64  # cost
    # pind::Int64  # parent index

    def __init__(self, xind, yind, yawind, direction, x, y, yaw, directions, steer, cost, pind):
        self.xind = xind
        self.yind = yind
        self.yawind = yawind
        self.direction = direction
        self.x = x
        self.y = y
        self.yaw = yaw
        self.directions = directions
        self.steer = steer
        self.cost = cost
        self.pind = pind

class Config(object):

    def __init__(self, minx, miny, minyaw, maxx, maxy, maxyaw, xw, yw, yaww, xyreso, yawreso):
        self.minx = minx
        self.miny = miny
        self.minyaw = minyaw
        self.maxx = maxx
        self.maxy = maxy
        self.maxyaw = maxyaw
        self.xw = xw
        self.yw = yw
        self.yaww = yaww
        self.xyreso = xyreso
        self.yawreso = yawreso

    def prn_obj(obj):
        print '\n'.join(['%s:%s' % item for item in obj.__dict__.items()])

class Path(object):

    # x::Array{Float64} # x position [m]
    # y::Array{Float64} # y position [m]
    # yaw::Array{Float64} # yaw angle [rad]
    # yaw1::Array{Float64} # trailer angle [rad]
    # direction::Array{Bool} # direction forward: true, back false
    # cost::Float64 # cost

    def __init__(self, x, y, yaw, direction, cost):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.direction = direction
        self.cost = cost

def calc_hybrid_astar_path(sx , sy , syaw , gx , gy , gyaw ,  ox , oy , xyreso , yawreso):

    # sx: start x position[m]
    # sy: start y position[m]
    # gx: goal x position[m]
    # gx: goal x position[m]
    # ox: x position list of Obstacles[m]
    # oy: y position list of Obstacles[m]
    # xyreso: grid resolution[m]
    # yawreso: yaw angle resolution[rad]

    syaw0 = rs_path.pi_2_pi(syaw)
    gyaw0 = rs_path.pi_2_pi(gyaw)
    global tox,toy
    ox, oy = ox[:], oy[:]
    tox, toy = ox[:], oy[:]
    kdtree = KDTree(np.vstack((tox, toy)).T)

    # a = np.array((1,2,3))
    # b = np.array((2,3,4))
    # c = np.hstack((a,b))
    # tree = spatial.KDTree((a,b),10)
    # print tree.date

    c = calc_config(ox, oy, xyreso, yawreso)
    #c.prn_obj()
    nstart = Node(round(sx / xyreso), round(sy / xyreso), round(syaw0 / yawreso), True, [sx], [sy],[syaw0], [True], 0.0, 0.0, -1)
    ngoal = Node(round(gx/xyreso), round(gy/xyreso), round(gyaw0/yawreso), True, [gx],[gy],[gyaw0],[True],0.0,0.0, -1)
    h_dp = calc_holonomic_with_obstacle_heuristic(ngoal, ox, oy, xyreso)
    openset, closedset = {},{}
    fnode = None
    openset[calc_index(nstart, c)] = nstart
    pq = Queue.PriorityQueue()
    pq.put(calc_index(nstart, c),  calc_cost(nstart, h_dp, ngoal, c))
    u, d = calc_motion_inputs()
    nmotion = len(u)

    if vehicle_lib.check_collision(ox, oy, [sx], [sy], [syaw0], kdtree) == False:
        print '1111111'
        return []
    if vehicle_lib.check_collision(ox, oy, [gx], [gy], [gyaw0], kdtree) == False:
        print '2222222'
        return []



    times = 0
    while 1:
        # if times >100:
        #     return []
        if len(openset) == 0:
            print "Error: Cannot find path, No open set"
            return []

        c_id = min(
            openset, key=lambda o: openset[o].cost + calc_cost(nstart, h_dp, ngoal, c))


        #c_id = pq.get()
        current = openset[c_id]

        # move current node from open to closed
        del openset[c_id]
        closedset[c_id] = current

        # visualize_x = []
        # visualize_y = []
        # for v in closedset.values():
        #     visualize_x.append(v.x[-1])
        #     visualize_y.append(v.y[-1])
        # print visualize_x,visualize_y
        # plt.plot(tox, toy, ".k")
        # plt.plot(visualize_x,visualize_y,'.k')
        # plt.pause(0.1)

        isupdated, fpath = update_node_with_analystic_expantion(current, ngoal, c, ox, oy, kdtree, True)
        if isupdated:  # found
            fnode = fpath
            break


        for i in range(nmotion):
            node = calc_next_node(current, c_id, u[i], d[i], c)

            if verify_index(node, c, ox, oy, kdtree) == False:
                continue

            node_ind = calc_index(node, c)
            #print 'expend node id:', node_ind
            # If it is already in the closed set, skip it
            if closedset.has_key(node_ind):
                continue

            if openset.has_key(node_ind) == False:
                openset[node_ind] = node
                pq.put(node_ind, calc_cost(nstart, h_dp, ngoal, c))
            else:
                if openset[node_ind].cost > node.cost:
                    # If so, update the node to have a new parent
                    openset[node_ind] = node
            times = times + 1
    print "final expand node:", len(openset) + len(closedset)
    path = get_final_path(closedset, fnode, nstart, c)
    return path

def calc_config(ox, oy, xyreso, yawreso):

    min_x_m = min(ox) - EXTEND_AREA
    min_y_m = min(oy) - EXTEND_AREA
    max_x_m = max(ox) + EXTEND_AREA
    max_y_m = max(oy) + EXTEND_AREA

    ox.append(min_x_m)
    oy.append(min_y_m)
    ox.append(max_x_m)
    oy.append(max_y_m)

    minx = round(min_x_m/xyreso)
    miny = round(min_y_m/xyreso)
    maxx = round(max_x_m/xyreso)
    maxy = round(max_y_m/xyreso)

    xw = round(maxx - minx)
    yw = round(maxy - miny)

    minyaw = round(- math.pi/yawreso) - 1
    maxyaw = round(math.pi/yawreso)
    yaww = round(maxyaw - minyaw)

    # minyawt = minyaw
    # maxyawt = maxyaw
    # yawtw = yaww

    config = Config(minx, miny, minyaw, maxx, maxy, maxyaw, xw, yw, yaww, xyreso, yawreso)
    return config

def calc_holonomic_with_obstacle_heuristic(Node, ox , oy , xyreso):
    h_dp = grid_a_star.calc_dist_policy(Node.x[-1], Node.y[-1], ox, oy, xyreso, 1.0)
    return h_dp

def calc_index(node, c):
    ind = (node.yawind - c.minyaw)*c.xw*c.yw+(node.yind - c.miny)*c.xw + (node.xind - c.minx)

    # 4D grid
    yaw1ind = round(node.yaw1[-1]/c.yawreso)
    ind += (yaw1ind - c.minyawt) *c.xw*c.yw*c.yaww

    if ind <= 0:
        print "Error(calc_index):", ind
    return ind

def calc_cost(n, h_dp, ngoal, c):

   return (n.cost + 3*H_COST*h_dp[int(n.xind - c.minx)][int(n.yind - c.miny)])

def calc_motion_inputs():

    up = []
    for i in range(int(N_STEER)-1,-1,-1):
        x = MAX_STEER - i*(MAX_STEER/N_STEER)
        up.append(x)
    #print up
    u = [0.0] + [i for i in up] + [-i for i in up]
    #print u
    d = [1.0 for i in range(len(u))] + [-1.0 for i in range(len(u))]
    #print d
    u = u + u
    #print u
    return u, d

def update_node_with_analystic_expantion(current, ngoal, c, ox, oy, kdtree, draw = False):

    apath = analystic_expantion(current, ngoal, ox, oy, kdtree, draw)
    if apath != None:
        fx = apath.x[1:]
        fy = apath.y[1:]
        fyaw =  apath.yaw[1:]
        fcost = current.cost + calc_rs_path_cost(apath)
        fpind = calc_index(current, c)

        fd = []
        for d in apath.directions[1:]:
            if d >= 0:
                fd.append(True)
            else:
                fd.append(False)
        fsteer = 0.0
        fpath = Node(current.xind, current.yind, current.yawind, current.direction, fx, fy, fyaw, fd, fsteer, fcost, fpind)
        return True, fpath
    return False, None #no update

def analystic_expantion(n, ngoal, ox, oy, kdtree, draw):

    sx = n.x[-1]
    sy = n.y[-1]
    syaw = n.yaw[-1]

    max_curvature = math.tan(MAX_STEER)/WB
    paths = rs_path.calc_paths(sx,sy,syaw,ngoal.x[-1], ngoal.y[-1], ngoal.yaw[-1],
                                   max_curvature, step_size=MOTION_RESOLUTION)

    if len(paths) == 0:
        return None

    #pathqueue = PriorityQueue{rs_path.Path}
    #pathqueue = Queue.PriorityQueue()
    pathset = {}
    path_id = 0
    for path in paths:        #yaw1 = vehicle_lib.calc_trailer_yaw_from_xyyaw(path.x, path.y, path.yaw, n.yaw1[-1], steps)
        #pathqueue.put(path, calc_rs_path_cost(path))
        pathset[path_id] = path
        path_id = path_id + 1

    ind = 0
    for i in range(len(pathset)):
        #path = pathqueue.get()
        p_id = min(
            pathset, key=lambda o: calc_rs_path_cost(pathset[o]))
        path = pathset[p_id]
        # if draw == True:
        #     plt.grid(True)
        #     plt.axis("equal")
        #     plt.plot(path.x, path.y, linewidth = '0.3', color= 'red')
        #     plt.pause(0.01)
        if vehicle_lib.check_collision(ox, oy, path.x, path.y, path.yaw, kdtree):
            #plt.plot(path.x, path.y, "-^b")
            return path # path is ok

    return None


def calc_rs_path_cost(rspath):

    cost = 0.0
    for l in rspath.lengths:
        if l >= 0:  # forward
            cost += l
        else:  # back
            cost += abs(l) * BACK_COST

    # swich back penalty
    for i in range(len(rspath.lengths)-1):
        if rspath.lengths[i] * rspath.lengths[i + 1] < 0.0:  # switch back
            cost += SB_COST

    # steer penalyty
    for ctype in rspath.ctypes:
        if ctype != "S" : # curve
            cost += STEER_COST * abs(MAX_STEER)

    # ==steer change penalty
    # calc steer profile
    nctypes = len(rspath.ctypes)
    ulist = [0.0 for i in range(nctypes)]
    for i in range(nctypes):
        if rspath.ctypes[i] == "R":
            ulist[i] = - MAX_STEER
        elif rspath.ctypes[i] == "L":
            ulist[i] = MAX_STEER

    for i in range(len(rspath.ctypes)-1):
        cost += STEER_CHANGE_COST * abs(ulist[i + 1] - ulist[i])

    return cost

def get_final_path(closed, ngoal, nstart, c):

    rx, ry, ryaw = ngoal.x[::-1], ngoal.y[::-1], ngoal.yaw[::-1]
    direction = ngoal.directions[::-1]
    nid = ngoal.pind
    finalcost = ngoal.cost

    while 1:
        n = closed[nid]
        rx.extend(n.x[::-1])
        ry.extend(n.y[::-1])
        ryaw.extend(n.yaw[::-1])
        direction.extend(n.directions[::-1])
        nid = n.pind
        if is_same_grid(n, nstart):
            break

    rx = rx[::-1]
    ry = ry[::-1]
    ryaw = ryaw[::-1]
    direction = direction[::-1]

    # adjuct first direction
    direction[0] = direction[1]

    path = Path(rx, ry, ryaw, direction, finalcost)

    return path

def is_same_grid(node1, node2):

    if node1.xind != node2.xind:
        return False

    if node1.yind != node2.yind:
        return False

    if node1.yawind != node2.yawind:
        return False


    return True


def calc_next_node(current, c_id, u, d, c):

    arc_l = XY_GRID_RESOLUTION * 1.5

    nlist = math.ceil(arc_l / MOTION_RESOLUTION) + 1

    xlist, ylist, yawlist = [], [], []

    xlist_0 = current.x[-1] + d * MOTION_RESOLUTION * math.cos(current.yaw[-1])
    ylist_0 = current.y[-1] + d * MOTION_RESOLUTION * math.sin(current.yaw[-1])
    yawlist_0 = rs_path.pi_2_pi(current.yaw[-1] + d * MOTION_RESOLUTION / WB * math.tan(u))
    xlist.append(xlist_0)
    ylist.append(ylist_0)
    yawlist.append(yawlist_0)

    for i in range(1,int(nlist)):
        xlist_i = xlist[i-1] + d * MOTION_RESOLUTION * math.cos(yawlist[i-1])
        ylist_i = ylist[i-1] + d * MOTION_RESOLUTION * math.sin(yawlist[i-1])
        yawlist_i = rs_path.pi_2_pi(yawlist[i-1] + d * MOTION_RESOLUTION / WB * math.tan(u))
        xlist.append(xlist_i)
        ylist.append(ylist_i)
        yawlist.append(yawlist_i)

    xind = round(xlist[-1] / c.xyreso)
    yind = round(ylist[-1] / c.xyreso)
    yawind = round(yawlist[-1] / c.yawreso)

    addedcost = 0.0
    if d > 0:
        direction = True
        addedcost += abs(arc_l)
    else:
        direction = False
        addedcost += abs(arc_l) * BACK_COST

    # swich back penalty
    if direction != current.direction:  # switch back penalty
        addedcost += SB_COST

    # steer penalyty
    addedcost += STEER_COST * abs(u)

    # steer change penalty
    addedcost += STEER_CHANGE_COST * abs(current.steer - u)

    cost = current.cost + addedcost

    directions = [direction for i in range(len(xlist))]
    node = Node(xind, yind, yawind, direction, xlist, ylist, yawlist, directions, u, cost, c_id)

    return node


def calc_index(node, c):
    ind = (node.yawind - c.minyaw)*c.xw*c.yw+(node.yind - c.miny)*c.xw + (node.xind - c.minx)

    # 4D grid
    #yaw1ind = round(node.yaw1[-1]/c.yawreso)
    #ind += (yaw1ind - c.minyawt) *c.xw*c.yw*c.yaww

    if ind <= 0:
        print "Error(calc_index):", ind

    return ind


def verify_index(node, c, ox, oy, kdtree):

    # overflow map
    if (node.xind - c.minx) >= c.xw:
        return False
    elif (node.xind - c.minx) <= 0:
        return False
    if (node.yind - c.miny) >= c.yw:
        return False
    elif (node.yind - c.miny) <= 0:
        return False

    # check collisiton
    #steps = MOTION_RESOLUTION*node.directions
    #yaw1 = vehicle_lib.calc_trailer_yaw_from_xyyaw(node.x, node.y, node.yaw, inityaw1, steps)
    #ind = 1:SKIP_COLLISION_CHECK:length(node.x)
    #if !trailerlib.check_trailer_collision(ox, oy, node.x[ind], node.y[ind], node.yaw[ind], yaw1[ind], kdtree = kdtree):
        #return False

    if vehicle_lib.check_collision(ox, oy, node.x, node.y, node.yaw, kdtree) == False:
        return False
    return True #index is ok"



class KDTree:
    """
    Nearest neighbor search class with KDTree
    """

    def __init__(self, data):
        # store kd-tree
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
        else:
            dist, index = self.tree.query(inp, k=k)
            return index, dist

    def search_in_distance(self, inp, r):
        """
        find points with in a distance r
        """
        index = self.tree.query_ball_point(inp, r)
        return index