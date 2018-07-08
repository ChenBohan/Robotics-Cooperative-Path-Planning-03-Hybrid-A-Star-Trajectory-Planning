import numpy as np
import matplotlib.pyplot as plt
import vehicle
import planner
from scipy import interpolate
import vehicle_lib
import math
import scipy.spatial

final_mx = 0
final_my = 0

SX = -4.0  # [m]
SY = -6.0  # [m]
SYAW = np.deg2rad(90.0)

# goal state
GX = 14.0  # [m]
GY = 6.0  # [m]
GYAW = np.deg2rad(0)

OTHER_SX = 14.0  # [m]
OTHER_SY = 6.0  # [m]
OTHER_SYAW = np.deg2rad(180.0)

OTHER_GX = -4.0  # [m]
OTHER_GY = -6.0  # [m]
OTHER_GYAW = np.deg2rad(-90.0)

# set obstacles
ox = []
oy = []

for i in range(-8, 25):
    ox.append(i)
    oy.append(9.0)
for i in range(0, 25):
    ox.append(i)
    oy.append(3)
for i in range(-10, 10):
    ox.append(-8.0)
    oy.append(i)
for i in range(-10, 4):
    ox.append(0)
    oy.append(i)
for i in range(-8, 0):
    ox.append(i)
    oy.append(-10)
for i in range(3, 9):
    ox.append(24)
    oy.append(i)

plt.grid(True)
plt.axis("equal")
plt.plot(ox,oy,'.k')
vehicle.plot_trailer(SX, SY, SYAW, 0)
vehicle.plot_trailer(OTHER_SX, OTHER_SY, OTHER_SYAW, 0)
plt.show()

class States(object):

    def __init__(self, sx, sy, syaw, mx, my, myaw, gx, gy, gyaw, ox, oy):
        self.sx = sx
        self.sy = sy
        self.syaw = syaw
        self.mx = mx
        self.my = my
        self.myaw = myaw
        self.gx = gx
        self.gy = gy
        self.gyaw = gyaw
        self.estimated_cost = self.estimate_total_rs_cost(sx, sy, syaw, mx, my, myaw, gx, gy, gyaw, ox, oy)

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

    def estimate_total_rs_cost(self, sx, sy, syaw, mx, my, myaw, gx, gy, gyaw, ox, oy):

        cost1 = self.estimate_rs_cost(sx, sy, syaw, mx, my, myaw, ox, oy)
        vehicle_x, vehicle_y = planner.vehicle.plot_trailer(mx, my, myaw, 0.0, True)
        oox_add, ooy_add = add_obstacle(vehicle_x, vehicle_y, ox, oy)
        cost2 = self.estimate_rs_cost(OTHER_SX, OTHER_SY, OTHER_SYAW, OTHER_GX, OTHER_GY, OTHER_GYAW, oox_add, ooy_add)
        cost3 = self.estimate_rs_cost(mx, my, myaw, gx, gy, gyaw, ox, oy)
        return cost1 + cost2 + cost3


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



    # global total_cost
    # global final_path1
    # global final_path2
    # global final_path3,final_mx,final_my,final_myaw0
    #
    # if path1.cost + path2.cost + path3.cost < total_cost:
    #     final_path1 = path1
    #     final_path2 = path2
    #     final_path3 = path3
    #     final_mx = mx
    #     final_my = my
    #     final_myaw0 = myaw0

    show_animation(path1, ox, oy, other_sx, other_sy, other_syaw0)
    show_animation(path2, ox, oy, mx, my, myaw0)
    show_animation(path3, ox, oy, other_gx, other_gy, other_gyaw0)
    plt.show()


# def show_animation(path, oox, ooy, sx, sy, syaw0, gx, gy, gyaw0):
#     plt.plot(path.x, path.y, ".k")
#     plt.plot(oox, ooy, ".k")
#     planner.vehicle.plot_trailer(sx, sy, syaw0, 0.0)

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

def show_animation(path, oox, ooy, other_x=None, other_y=None, other_yaw0=None):
    x = path.x
    y = path.y
    yaw = path.yaw
    direction = path.direction
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
        vehicle.plot_trailer(x[ii], y[ii], yaw[ii], steer)
        plt.grid(True)
        plt.axis("equal")
        plt.pause(0.01)
    print yaw[ii]

yield_x, yield_y, yield_yaw  = [],[],[]
for x in range(-8,0,1):
    for y in range(0,8,1):
        for yaw in range(0,90,15):
            yield_x.append(x)
            yield_y.append(y)
            yield_yaw.append(yaw)

states_set = {}
s_id = 0
print len(yield_x)
for mx,my,myaw in zip(yield_x, yield_y, yield_yaw):
    oox, ooy = ox[:],oy[:]
    states = States(SX, SY, SYAW, mx, my, np.deg2rad(myaw), GX, GY, GYAW, ox, oy)
    states_set[s_id] = states
    s_id = s_id + 1


for i in range(len(states_set)):
    s_id = min(
        states_set, key=lambda o: states_set[o].estimated_cost)
    states = states_set[s_id]
    del states_set[s_id]
    print 'mx:',states.mx, 'my:',states.my, 'myaw:',states.myaw, 'cost:', states.estimated_cost
    main(states.mx, states.my, states.myaw)







