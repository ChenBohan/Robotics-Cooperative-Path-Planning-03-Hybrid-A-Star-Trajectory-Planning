import numpy as np
import matplotlib.pyplot as plt
import math
# Vehicle parameter
W = 1.8 #[m] width of vehicle
LF = 3.7 #[m] distance from rear to vehicle front end of vehicle
LB = 1.0 #[m] distance from rear to vehicle back end of vehicle
TR = 0.5 # Tyre radius [m] for plot
TW = 1.0 # Tyre width [m] for plot
MAX_STEER = 0.6 #[rad] maximum steering angle
WB = 2.7  #[m] wheel base: rear to front steer

def plot_trailer(x, y, yaw, steer, retrun_trailer = False):
    truckcolor = "-k"

    LENGTH = LB+LF

    truckOutLine = np.array([[-LB, (LENGTH - LB), (LENGTH - LB), (-LB), (-LB)],
                             [W / 2, W / 2, - W / 2, - W / 2, W / 2]])


    rr_wheel = np.array([[TR, - TR, - TR, TR, TR],
                         [-W / 12.0 + TW, - W / 12.0 + TW, W / 12.0 + TW, W / 12.0 + TW, - W / 12.0 + TW]])

    rl_wheel = np.array([[TR, - TR, - TR, TR, TR],
                         [-W / 12.0 - TW, - W / 12.0 - TW, W / 12.0 - TW, W / 12.0 - TW, - W / 12.0 - TW]])

    fr_wheel = np.array([[TR, - TR, - TR, TR, TR],
                         [- W / 12.0 + TW, - W / 12.0 + TW, W / 12.0 + TW, W / 12.0 + TW, - W / 12.0 + TW]])

    fl_wheel = np.array([[TR, - TR, - TR, TR, TR],
                         [-W / 12.0 - TW, - W / 12.0 - TW, W / 12.0 - TW, W / 12.0 - TW, - W / 12.0 - TW]])

    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                    [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])


    fr_wheel = np.dot(fr_wheel.T, Rot2).T
    fl_wheel = np.dot(fl_wheel.T, Rot2).T
    fr_wheel[0,:] += WB
    fl_wheel[0,:] += WB
    fr_wheel = np.dot(fr_wheel.T, Rot1).T
    fl_wheel = np.dot(fl_wheel.T, Rot1).T

    truckOutLine = np.dot(truckOutLine.T, Rot1)

    rr_wheel = np.dot(rr_wheel.T, Rot1).T
    rl_wheel = np.dot(rl_wheel.T, Rot1).T

    truckOutLine = truckOutLine.T
    truckOutLine[0,:] += x
    truckOutLine[1,:] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    if retrun_trailer == False:
        plt.plot(x, y, "*")
        plt.plot(fr_wheel[0, :], fr_wheel[1, :], truckcolor)
        plt.plot(rr_wheel[0, :], rr_wheel[1, :], truckcolor)
        plt.plot(fl_wheel[0, :], fl_wheel[1, :], truckcolor)
        plt.plot(rl_wheel[0, :], rl_wheel[1, :], truckcolor)
        plt.plot(truckOutLine[0, :], truckOutLine[1, :], truckcolor)
    else:
        return truckOutLine[0, :], truckOutLine[1, :]