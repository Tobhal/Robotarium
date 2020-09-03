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
initial_conditions = np.array(np.mat('1.00;1.00;0'))

r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=initial_conditions,sim_in_real_time=True)

unicycle_pose_controller = create_hybrid_unicycle_pose_controller()
uni_barrier_cert = create_unicycle_barrier_certificate()



# Functions to help the code look better
def setVelocity(linear: float, angular: float):
    vel = np.array([linear, angular])
    vel.shape = (2,1)

    r.set_velocities(np.arange(N), vel)

def plotCorse():
    numPoints = 25
    parameterVec = np.linspace( 0.0 , 2 * np.pi , num = numPoints)

    xVec = np.zeros(numPoints, dtype = np.float64)
    yVec = np.zeros(numPoints, dtype = np.float64)
    for i  in range(numPoints):
        theta = float( parameterVec[ i ] )
        radius = np.sqrt( np.absolute( np.cos (   theta) ) )
        xVec[i] = radius *  np.cos ( theta )
        yVec[i] = radius *  np.sin ( theta )
    
    r.axes.plot(xVec,yVec,color='red',linewidth = 5)

def plotCorseTwo():
    radius = 1
    xVec = np.zeros(2, dtype = np.float64)
    yVec = np.zeros(2, dtype = np.float64)

    xVec[0] = (radius + 0.3) * np.cos(0)
    yVec[0] = radius * np.sin(0)

    xVec[1] = ((radius + 0.3) * np.cos(0.03))
    yVec[1] = radius * np.sin(0.03)

    

    r.axes.plot(xVec, yVec, color='red', lineWidth = 1)


def plotCorseThree():
    steps = 20

    angle = 0.3
    offset = 0.5

    xVec = [0]
    yVec = [0]

    xVec.append(offset)
    yVec.append(np.sin(angle))

    #xVec.append(offset)
    #yVec.append(-radius)

    circle = np.arange(0, np.pi, 0.1)

    for i in circle:
        xVec.append(offset + (angle * np.sin(i)))
        yVec.append(angle * np.cos(i))

    circle = np.arange(0, np.pi, 0.1)

    for i in circle:
        xVec.append(offset + (angle * np.sin(i)))
        yVec.append(angle * np.cos(i))



    # Draw the corse
    r.axes.plot(xVec, yVec, color='red', lineWidth = 1)

        
plotCorseTwo()
#plotCorseThree()

x = r.get_poses()

r.step()

setVelocity(0, 0)

countMax = 200

count = 0


while ((count < countMax)):
    x = r.get_poses()

    setVelocity(0, 0)

    r.step()

    count += 1

r.call_at_scripts_end()


