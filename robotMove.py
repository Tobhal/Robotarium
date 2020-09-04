import rps.robotarium as robotarium
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *

import numpy as np
import time



# initialization
vel = np.array

N = 1
initial_conditions = np.array(np.mat('0.05;0.05;0'))

r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=initial_conditions,sim_in_real_time=True)

unicycle_pose_controller = create_hybrid_unicycle_pose_controller()
uni_barrier_cert = create_unicycle_barrier_certificate()

corse = [[],[]]

arrX = []
arrY = []

# Functions to help the code look better
def setVelocity(linear: float, angular: float):
    vel = np.array([linear, angular])
    vel.shape = (2,1)

    r.set_velocities(np.arange(N), vel)

def plotCorse(angle, length, offset, steps, roadWidth):
    global corse
    
    xVec = [0]
    yVec = [0]

    xVec.append(length)
    yVec.append(np.sin(angle))

    circle = np.arange(0, np.pi, np.pi/steps)

    for i in circle:
        xVec.append(offset + (angle * np.sin(i)))
        yVec.append(angle * np.cos(i))

    xVec.append(length)
    yVec.append(-(np.sin(angle)))

    xVec.append(0)  # Can remove?
    yVec.append(0)  # Can remove?

    xVec.append(-length)
    yVec.append(np.sin(angle))

    circle = np.arange(0, np.pi, np.pi/steps)

    for i in circle:
        xVec.append(-(offset + (angle * np.sin(i))))
        yVec.append((angle * np.cos(i)))

    xVec.append(-length)
    yVec.append(-(np.sin(angle)))

    xVec.append(0)
    yVec.append(0)

    corse[0] = xVec
    corse[1] = yVec

    # Draw the corse
    r.axes.plot(xVec, yVec, color='red', lineWidth = roadWidth)

def closesPoint(t):
    global corse
    global arrX
    global arrY

    x = t[0]
    y = t[1]

    mini = 0
    minDist = 9999999

    for i in range(len(corse[0])):
        #print(corse[0][i]," - ", corse[1][i])
        dist = np.exp2(x - corse[0][i]) + np.exp2(y - corse[1][i])
        
        if dist < minDist:
            minDist = dist
            mini = i

    #r.axes.clear()

    r.axes.plot(arrX, arrY, color = 'white', lineWidth = 2)

    arrX = []
    arrY = []

    arrX.append(x - 0.001)
    arrY.append(y - 0.001)
    arrX.append(corse[0][mini] - 0.001)
    arrY.append(corse[1][mini] - 0.001)

    r.axes.plot(arrX, arrY, color = 'blue', lineWidth = 1)

    return mini


    
    

plotCorse(0.4, 0.4, 1, 20, 1)

x = r.get_poses()

r.step()

setVelocity(0, 0)

countMax = 200

count = 0

# Test printing

def debugPrint(t):
    print("-----")

    #print(corse)

    print("-----")

while ((count < countMax)):
    x = r.get_poses()

    setVelocity(0.3,0)

    if count == 0:
        debugPrint(x)

    print(closesPoint(x))

    r.step()

    count += 1

r.call_at_scripts_end()


