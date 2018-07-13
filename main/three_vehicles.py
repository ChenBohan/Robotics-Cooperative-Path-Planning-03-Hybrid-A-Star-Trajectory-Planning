import numpy as np
import matplotlib.pyplot as plt
import vehicle
import planner
from scipy import interpolate
import vehicle_lib
import math
import scipy.spatial


SX = -20.0  # [m]
SY = 7.0  # [m]
SYAW = np.deg2rad(0.0)

# goal state
GX = 20.0  # [m]
GY = 7.0  # [m]
GYAW = np.deg2rad(0.0)

OTHER_SX = 0.0  # [m]
OTHER_SY = 20.0  # [m]
OTHER_SYAW = np.deg2rad(-90.0)

OTHER_GX = -20  # [m]
OTHER_GY = 7.0  # [m]
OTHER_GYAW = np.deg2rad(180.0)

OTHER_OTHER_SX = 20  # [m]
OTHER_OTHER_SY = 7  # [m]
OTHER_OTHER_SYAW = np.deg2rad(-180.0)

OTHER_OTHER_GX = 0.0  # [m]
OTHER_OTHER_GY = 20.0  # [m]
OTHER_OTHER_GYAW = np.deg2rad(90.0)

# set obstacles
global ox, oy
ox = []
oy = []
for i in range(-25, 25):
    ox.append(i)
    oy.append(4.0)
for i in range(-25, -3):
    ox.append(i)
    oy.append(9.0)
for i in range(9, 25):
    ox.append(-3.0)
    oy.append(i)
for i in range(9, 26):
    ox.append(3.0)
    oy.append(i)
for i in range(3, 25):
    ox.append(i)
    oy.append(9.0)
for i in range(-3, 3):
    ox.append(i)
    oy.append(25.0)
for i in range(4, 9):
    ox.append(-25)
    oy.append(i)
for i in range(4, 9):
    ox.append(25)
    oy.append(i)

plt.grid(True)
plt.axis("equal")
plt.plot(ox,oy,'.k')
vehicle.plot_trailer(SX, SY, SYAW, 0)
vehicle.plot_trailer(OTHER_SX, OTHER_SY, OTHER_SYAW, 0)
vehicle.plot_trailer(OTHER_OTHER_SX, OTHER_OTHER_SY, OTHER_OTHER_SYAW, 0)
plt.pause(0.01)

class States(object):

    def __init__(self, sx, sy, syaw, mx, my, myaw, ox, oy, gx = None, gy = None, gyaw = None):
        self.sx = sx
        self.sy = sy
        self.syaw = syaw
        self.mx = mx
        self.my = my
        self.myaw = myaw
        self.gx = gx
        self.gy = gy
        self.gyaw = gyaw
        if gx == None and gy == None and gyaw == None:
            self.estimated_cost = self.estimate_total_2_rs_cost(sx, sy, syaw, mx, my, myaw, ox, oy)
        else:
            self.estimated_cost = self.estimate_total_3_rs_cost(sx, sy, syaw, mx, my, myaw, gx, gy, gyaw, ox, oy)

    def estimate_rs_cost(self, sx, sy, syaw, gx, gy, gyaw, ox, oy):

        tox,toy = ox[:], oy[:]
        oox,ooy = ox[:], oy[:]
        kdtree = planner.KDTree(np.vstack((tox, toy)).T)
        nstart = planner.Node(round(sx / planner.XY_GRID_RESOLUTION), round(sy / planner.XY_GRID_RESOLUTION), round(syaw / planner.YAW_GRID_RESOLUTION), True, [sx], [sy], [syaw], [True],
                      0.0, 0.0, -1)
        ngoal = planner.Node(round(gx / planner.XY_GRID_RESOLUTION), round(gy / planner.XY_GRID_RESOLUTION), round(gyaw / planner.YAW_GRID_RESOLUTION), True, [gx], [gy], [gyaw], [True],
                     0.0, 0.0, -1)
        c = planner.calc_config(oox, ooy, planner.XY_GRID_RESOLUTION, planner.YAW_GRID_RESOLUTION)
        isupdated, fpath = planner.update_node_with_analystic_expantion(nstart, ngoal, c, oox, ooy, kdtree)
        if isupdated:
            cost = fpath.cost
            return cost
        else:
            return 10000

    def estimate_total_3_rs_cost(self, sx, sy, syaw, mx, my, myaw, gx, gy, gyaw, ox, oy):

        cost1 = self.estimate_rs_cost(sx, sy, syaw, mx, my, myaw, ox, oy)
        vehicle_x, vehicle_y = planner.vehicle.plot_trailer(mx, my, myaw, 0.0, True)
        oox_add, ooy_add = add_obstacle(vehicle_x, vehicle_y, ox, oy)
        cost2 = self.estimate_rs_cost(OTHER_SX, OTHER_SY, OTHER_SYAW, OTHER_GX, OTHER_GY, OTHER_GYAW, oox_add, ooy_add)
        cost3 = self.estimate_rs_cost(mx, my, myaw, gx, gy, gyaw, ox, oy)
        return cost1 + cost2 + cost3

    def estimate_total_2_rs_cost(self, sx, sy, syaw, mx, my, myaw, ox, oy):

        cost1 = self.estimate_rs_cost(sx, sy, syaw, mx, my, myaw, ox, oy)
        vehicle_x, vehicle_y = planner.vehicle.plot_trailer(mx, my, myaw, 0.0, True)
        oox_add, ooy_add = add_obstacle(vehicle_x, vehicle_y, ox, oy)
        cost2 = self.estimate_rs_cost(OTHER_SX, OTHER_SY, OTHER_SYAW, OTHER_GX, OTHER_GY, OTHER_GYAW, oox_add, ooy_add)
        return cost1 + cost2

def main(x,y,yaw):
    sx = SX  # [m]
    sy = SY  # [m]
    syaw0 = SYAW

    # goal state
    gx = GX  # [m]
    gy = GY  # [m]
    gyaw0 = GYAW

    other_sx = OTHER_SX  # [m]
    other_sy = OTHER_SY  # [m]
    other_syaw0 = OTHER_SYAW

    other_gx = OTHER_GX  # [m]
    other_gy = OTHER_GY  # [m]
    other_gyaw0 = OTHER_GYAW

    mx = x  # [m]
    my = y  # [m]
    myaw0 = yaw

    # set obstacles
    global ox,oy

    oox = ox[:]
    ooy = oy[:]

    tox, toy = ox[:], oy[:]
    kdtree = planner.KDTree(np.vstack((tox, toy)).T)

    if vehicle_lib.check_collision(oox, ooy, [mx], [my], [myaw0], kdtree) == False:
        return


    # path generation
    path1 = planner.calc_hybrid_astar_path(sx, sy, syaw0, mx, my, myaw0, oox, ooy,
                                            planner.XY_GRID_RESOLUTION,
                                            planner.YAW_GRID_RESOLUTION)
    if path1 == []:
        print 'Cannot find path.(1)'
        return

    vehicle_x,vehicle_y = planner.vehicle.plot_trailer(mx, my, myaw0, 0.0, True)

    oox_add, ooy_add = add_obstacle(vehicle_x, vehicle_y, oox, ooy)

    path2 = planner.calc_hybrid_astar_path(other_sx, other_sy, other_syaw0, other_gx, other_gy, other_gyaw0, oox_add,
                                          ooy_add,
                                          planner.XY_GRID_RESOLUTION,
                                          planner.YAW_GRID_RESOLUTION)

    if path2 == []:
        print 'The middle state is invalid(2)'
        return

    path3 = planner.calc_hybrid_astar_path(mx, my, myaw0, gx, gy, gyaw0, oox, ooy,
                                          planner.XY_GRID_RESOLUTION,
                                          planner.YAW_GRID_RESOLUTION)

    if path3 == []:
        print 'Cannot find path.(3)'
        return

    vehicle.plot_trailer(other_sx, other_sy, other_syaw0, 0)
    show_animation(path1, ox, oy, other_sx, other_sy, other_syaw0)
    show_animation(path2, ox, oy, mx, my, myaw0)
    show_animation(path3, ox, oy, other_gx, other_gy, other_gyaw0)
    plt.pause(0.1)


def main_middle(sx, sy, syaw, mx, my, myaw, other_sx, other_sy, other_syaw, other_gx, other_gy, other_gyaw, other_other_x = None, other_other_y = None, other_other_yaw = None):


    # set obstacles
    global ox,oy

    oox = ox[:]
    ooy = oy[:]

    tox, toy = ox[:], oy[:]
    kdtree = planner.KDTree(np.vstack((tox, toy)).T)

    if vehicle_lib.check_collision(oox, ooy, [mx], [my], [myaw], kdtree) == False:
        return None, None, None

    vehicle_x, vehicle_y = planner.vehicle.plot_trailer(mx, my, myaw, 0.0, True)

    oox_add, ooy_add = add_obstacle(vehicle_x, vehicle_y, oox, ooy)

    path2 = planner.calc_hybrid_astar_path(other_sx, other_sy, other_syaw, other_gx, other_gy, other_gyaw, oox_add,
                                           ooy_add,
                                           planner.XY_GRID_RESOLUTION,
                                           planner.YAW_GRID_RESOLUTION)

    if path2 == []:
        print 'The middle state is invalid(2)'
        return None, None, None


    # path generation
    path1 = planner.calc_hybrid_astar_path(sx, sy, syaw, mx, my, myaw, oox, ooy,
                                            planner.XY_GRID_RESOLUTION,
                                            planner.YAW_GRID_RESOLUTION)
    if path1 == []:
        print 'Cannot find path.(1)'
        return None, None, None

    vehicle.plot_trailer(other_sx, other_sy, other_syaw, 0)
    show_animation(path1, ox, oy, other_sx, other_sy, other_syaw, other_other_x, other_other_y, other_other_yaw)
    show_animation(path2, ox, oy, mx, my, myaw, other_other_x, other_other_y, other_other_yaw)
    plt.pause(0.01)

    return mx, my, myaw

def add_obstacle(x, y, oox, ooy):
    new_obstale_x, new_obstale_y = oox[:], ooy[:]
    for i in range(len(x)-1):
        temp_x = [x[i], x[i + 1]]
        temp_y = [y[i], y[i + 1]]
        if y[i] == y[i + 1]:
            xnew = np.linspace(x[i],x[i+1],10)
            f = interpolate.interp1d(temp_x, temp_y, kind="slinear")
            ynew = f(xnew)
            xnew, ynew = list(xnew), list(ynew)
            for i in range(len(xnew)):
                new_obstale_x.append(xnew[i])
                new_obstale_y.append(ynew[i])
        else:
            ynew = np.linspace(y[i], y[i + 1], 20)
            f = interpolate.interp1d(temp_y, temp_x, kind="slinear")
            xnew = f(ynew)
            xnew, ynew = list(xnew), list(ynew)
            for i in range(len(xnew)):
                new_obstale_x.append(xnew[i])
                new_obstale_y.append(ynew[i])
    # plt.plot(new_obstale_x, new_obstale_y, ".k")
    # plt.show()
    return new_obstale_x, new_obstale_y

def show_animation(path, oox, ooy, other_x=None, other_y=None, other_yaw0=None, other_other_x = None, other_other_y = None, other_other_yaw = None):
    x = path.x
    y = path.y
    yaw = path.yaw
    direction = path.direction
    steer = 0.0
    for ii in range(0,len(x),20):
        plt.cla()
        plt.plot(oox, ooy, ".k")
        plt.plot(x, y, "-r", label="Hybrid A* path")

        if ii < len(x)-1:
            k = (yaw[ii+1] - yaw[ii])/planner.MOTION_RESOLUTION
            if direction[ii] == False:
                k *= -1
            steer = math.atan2(vehicle_lib.WB*k, 1.0)
        else:
            steer = 0.0

        if other_x != None and other_y != None and other_yaw0 != None:
            vehicle.plot_trailer(other_x, other_y, other_yaw0, 0)
        if other_other_x != None and other_other_y != None and other_other_yaw != None:
            vehicle.plot_trailer(other_other_x, other_other_y, other_other_yaw, 0)
        vehicle.plot_trailer(x[ii], y[ii], yaw[ii], steer)
        plt.grid(True)
        plt.axis("equal")
        plt.pause(0.01)
    print yaw[ii]

yield_x_1, yield_y_1, yield_yaw_1  = [],[],[]
for x in range(0,5,1):
    for y in range(5,8,1):
        for yaw in range(-45,45,15):
            yield_x_1.append(x)
            yield_y_1.append(y)
            yield_yaw_1.append(yaw)

yield_x_2, yield_y_2, yield_yaw_2  = [],[],[]
for x in range(-5,0,1):
    for y in range(5,8,1):
        for yaw in range(-45,45,15):
            yield_x_2.append(x)
            yield_y_2.append(y)
            yield_yaw_2.append(yaw)

yield_x_3, yield_y_3, yield_yaw_3  = [],[],[]
for x in range(-8,8,2):
    for y in range(4,9,1):
        for yaw in range(-180,180,15):
            yield_x_3.append(x)
            yield_y_3.append(y)
            yield_yaw_3.append(yaw)

states_set_1 = {}
states_set_2 = {}
states_set_3 = {}
s_id = 0
print len(yield_x_1)
for mx,my,myaw in zip(yield_x_1, yield_y_1, yield_yaw_1):
    oox, ooy = ox[:],oy[:]
    states = States(SX, SY, SYAW, mx, my, np.deg2rad(myaw), ox, oy)
    states_set_1[s_id] = states
    s_id = s_id + 1



while 1:

    global final_mx_1, final_my_1, final_myaw_1

    for i in range(len(states_set_1)):
        s_id = min(
            states_set_1, key=lambda o: states_set_1[o].estimated_cost)
        states = states_set_1[s_id]
        del states_set_1[s_id]
        print 'mx:', states.mx, 'my:', states.my, 'myaw:', states.myaw, 'cost:', states.estimated_cost
        final_mx_1, final_my_1, final_myaw_1= main_middle(SX, SY,SYAW, states.mx, states.my, states.myaw, OTHER_SX, OTHER_SY, OTHER_SYAW, OTHER_GX, OTHER_GY, OTHER_GYAW, OTHER_OTHER_SX, OTHER_OTHER_SY, OTHER_OTHER_SYAW)
        if final_mx_1 != None and final_my_1 != None and final_myaw_1 !=None :
            break

    states_set_2 = {}
    s_id = 0
    print len(yield_x_2)
    for mx, my, myaw in zip(yield_x_2, yield_y_2, yield_yaw_2):
        oox, ooy = ox[:], oy[:]
        states = States(final_mx_1, final_my_1, final_myaw_1, mx, my, np.deg2rad(myaw), ox, oy)
        states_set_2[s_id] = states
        s_id = s_id + 1

    for i in range(len(states_set_2)):
        s_id = min(
            states_set_2, key=lambda o: states_set_2[o].estimated_cost)
        states = states_set_2[s_id]
        del states_set_2[s_id]
        print 'mx:', states.mx, 'my:', states.my, 'myaw:', states.myaw, 'cost:', states.estimated_cost
        final_mx_2, final_my_2,  final_myaw_2 = main_middle(final_mx_1, final_my_1, final_myaw_1, states.mx, states.my, states.myaw, OTHER_OTHER_SX, OTHER_OTHER_SY, OTHER_OTHER_SYAW, OTHER_OTHER_GX, OTHER_OTHER_GY, OTHER_OTHER_GYAW, OTHER_GX, OTHER_GY, OTHER_GYAW)
        if final_mx_2 != None and final_my_2 != None and final_myaw_2 != None:
            break

    oox = ox[:]
    ooy = oy[:]

    path3 = planner.calc_hybrid_astar_path(final_mx_2, final_my_2, final_myaw_2, GX, GY, GYAW, oox, ooy,
                                           planner.XY_GRID_RESOLUTION,
                                           planner.YAW_GRID_RESOLUTION)

    show_animation(path3, ox, oy, OTHER_OTHER_GX, OTHER_OTHER_GY, OTHER_OTHER_GYAW, OTHER_GX, OTHER_GY, OTHER_GYAW)





