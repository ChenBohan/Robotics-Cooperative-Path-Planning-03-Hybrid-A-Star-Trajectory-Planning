import math
import matplotlib.pyplot as plt

# Vehicle parameter


# for collision check
B = 1.0 # distance from rear to vehicle back end
C = 3.7 # distance from rear to vehicle front end
I = 2.5 # width of vehicle
WBUBBLE_DIST = (B+C)/2.0-B #[m] distance from rear and the center of whole bubble
WBUBBLE_R = (B+C)/2.0 #[m] whole bubble radius
VRX = [C, C, -B, -B, C ]
VRY = [-I/2.0, I/2.0, I/2.0, -I/2.0, -I/2.0]
WB = 2.7  #[m] wheel base: rear to front steer


# def check_collision(ox, oy, x, y, yaw):
#
#     for (ix, iy, iyaw) in zip(x, y, yaw):
#         # Whole bubble check
#         if rect_check(ix, iy, iyaw, ox, oy, VRX, VRY) == False:
#             # println("collision")
#             return False #collision
#         # println(ids)
#     return True


def rect_check(ix, iy, iyaw, ox, oy, vrx, vry):

    c = math.cos(-iyaw)
    s = math.sin(-iyaw)

    for (iox, ioy) in zip(ox, oy):
        tx = iox - ix
        ty = ioy - iy
        lx = (c*tx - s*ty)
        ly = (s*tx + c*ty)

        sumangle = 0.0
        for i in range(len(vrx)-1):
            x1 = vrx[i] - lx
            y1 = vry[i] - ly
            x2 = vrx[i+1] - lx
            y2 = vry[i+1] - ly
            d1 = math.hypot(x1,y1)
            d2 = math.hypot(x2,y2)
            theta1 = math.atan2(y1,x1)
            tty = (-math.sin(theta1)*x2 + math.cos(theta1)*y2)
            tmp = (x1*x2+y1*y2)/(d1*d2)

            if tmp >= 1.0:
                tmp = 1.0
            elif tmp <= 0.0:
                tmp = 0.0

            if tty >= 0.0:
                sumangle += math.acos(tmp)
            else:
                sumangle -= math.acos(tmp)

        if sumangle >= math.pi:
            return False #collision
    return True #OK


def check_collision(ox, oy, x, y, yaw, kdtree):


    for (ix, iy, iyaw) in zip(x, y, yaw):

        cx = ix + WBUBBLE_DIST* math.cos(iyaw)
        cy = iy + WBUBBLE_DIST* math.sin(iyaw)

        # Whole bubble check
        ids = kdtree.search_in_distance([cx,cy], WBUBBLE_R)
        # println(length(ids))
        #print len(ids)
        if len(ids) == 0:
            continue
        vrx = [C, C, -B, -B, C]
        vry = [-I / 2.0, I / 2.0, I / 2.0, -I / 2.0, -I / 2.0]
        temp_ox, temp_oy = [],[]
        for i in ids:
            temp_ox.append(ox[i])
            temp_oy.append(oy[i])

        if rect_check(ix, iy, iyaw, temp_ox, temp_oy, vrx, vry) == False:
            return False #collision
        # println(ids)

    # plt.plot(x, y, ".k")
    # plt.plot(ox, oy, ".k")
    # plt.show()

    return True  # OK

# def inrange(tree, points, radius, sortres=false):
#     # check_input(tree, points)
#     # check_radius(radius)
#
#     idxs = [Vector{Int}() for _ in 1:length(points)]
#
#     for i in 1:length(points)
#         inrange_point!(tree, points[i], radius, sortres, idxs[i])
#     end
#     return idxs
# end

# def check_input(tree, points):
#     if length(V1) != length(point)
#         throw(ArgumentError(
#             "dimension of input points:$(length(point)) and tree data:$(length(V1)) must agree"))
#     end
# end


