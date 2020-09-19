import rps.robotarium as robotarium
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *

import math
import numpy as np
import time

# initialization
vel = np.array

N = 1
initial_conditions = np.array(np.mat('0.5;-0.7;0'))

r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=initial_conditions,sim_in_real_time=True)

unicycle_pose_controller = create_hybrid_unicycle_pose_controller()
uni_barrier_cert = create_unicycle_barrier_certificate()

goal = 0

corse = [[],[]]

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

    xVec.append(offset)
    yVec.append(-(np.sin(angle)))

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

    xVec.append(-offset)
    yVec.append(-(np.sin(angle)))

    xVec.append(-length)
    yVec.append(-(np.sin(angle)))

    xVec.append(0)
    yVec.append(0)

    corse[0] = xVec
    corse[1] = yVec

    # Draw the corse
    r.axes.plot(xVec, yVec, color='red', lineWidth = roadWidth)

def calcAngle(rob, x2, y2):
    a = rob[0]
    b = rob[1]
    P = rob[2]

    c = x2
    d = y2

    theta = math.atan2( (d-b),(c-a) )

    fhi = P - theta

    turn = 3 * np.sin(fhi) 
    return -1*turn

def calcSpeed(rob, x2, y2):
    x1 = rob[0]
    y1 = rob[1]
    p = rob[2]

    length = math.sqrt( math.pow(x2-x1, 2) + math.pow(y2-y1, 2) )

    speed = 5 * length if length > 0.2 else 0.2

    return speed

def atGoal(rob, xG, yG):
    global goal

    return math.sqrt(math.pow(xG - rob[0], 2) + math.pow(yG - rob[1], 2))

#Robot driving
numPointsOneSide = 11
plotCorse(0.2, 0.2, 1, numPointsOneSide, 1)

count = 0

while count < 1:
    x = r.get_poses()

    if atGoal(x, corse[0][goal], corse[1][goal]) < 0.1:
        goal += 1
        if goal >= len(corse[0]):
            goal = 0
            count = count + 1

        print("Next goal", goal, ": X", corse[0][goal],"Y", corse[1][goal])
        print("Angle", calcAngle(x, corse[0][goal], corse[1][goal]))
        print("This Val:", x[0], x[1], x[2])
        print()
        print()

    setVelocity(calcSpeed(x, corse[0][goal], corse[1][goal]), calcAngle(x, corse[0][goal], corse[1][goal]))

    r.step()

r.call_at_scripts_end()