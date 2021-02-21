from Robot import *
from Path import *
from ShowPath import *
from math import *

MAX_DIST = 1
# MAX_DIST = 0.5
i = 0

""" 
This controller programm guides a robot in a simulated environment, such that it follows a pre-specified path. 
The path is given as a sequence of coordinates, and the program uses the robot's sensors (a laser scanner and a position sensor) 
in order to reproduce the given path.

The goal is to let the robot traverse over the given path and reach the goal, the target point,  as quickly as possible.

The program interacts with the Kompai robot in MRDS by using a defined functions for reading the robot's sensors and 
controlling the robot's actuators.

The path that the robot traverses and orientation of the robot is loaded as a list of coordinates. 
"""
def lookAhead(distance):
    
    global i
    coord = path[i]
    while distanceToPoint(path[i]) < distance and i < len(path):
         coord = path[i]
         i = i + 1
         if(i == len(path)):
             return coord

    return coord

# Calculates the distance from the robot to a given coordinate
# on the path.
# Uses Pythagoras to calculate the distance. 
def distanceToPoint(coord):
    pos = robot.getPosition()
    x = coord['X'] - pos['X']
    y = coord['Y'] - pos['Y']

    return sqrt((x**2)+(y**2))

# Calculates the angle from the robots x-axis to the point.
# First calculates the angle to the coordinate from the origo
# on the "world coordinate system".
# The robots angle/direction is given in the Robot.py.
# Subtracting the angle of the world coordinate system by
# the direction of the robots gives the robots angle to the
# point. 
def angleToPoint(lookAhead_coord):
    pos = robot.getPosition()
    x = lookAhead_coord['X'] - pos['X']
    y = lookAhead_coord['Y'] - pos['Y']

    angle = atan2(y,x)

    return (angle - robot.getHeading())
    

# Moves to robot from the current location to the given coordinates.
def goTo():

    lookAhead_coord = lookAhead(MAX_DIST)
    point = [lookAhead_coord['X'], lookAhead_coord['Y']]
    
    # rotate robot (radians/s)
    angle = angleToPoint(lookAhead_coord)

    turnRate = 0.5
    
    # Set the direction to turn depending on the angle.
    if((angle < 0 and angle > -pi) or angle > pi):
        turnRate = -turnRate

    # Turn the robot to face the look ahead point.
    robot.setMotion(0, turnRate)
    while(angle > 0.1 or angle < -0.1):
        # print("Turning...")
        angle = angleToPoint(lookAhead_coord)
        sp.update(robot.getPosition(), point)

    robot.setMotion(0,0)

    # move robot to point (m/s)
    #speed = 0.5
    speed = 1
    dist = distanceToPoint(lookAhead_coord)
    prevDist = dist

    # Move the robot towards the look ahead point.
    # The robot moves forward until it reaches a 
    # distance less than 0.1 units. 
    # If the distance of the previous iteration 
    # is less than the current distance it means
    # we are moving away from the point; stop 
    # the robot. 
    robot.setMotion(speed, 0)
    while(dist > 0.1 and prevDist >= dist):
        # print("Moving...")
        prevDist = dist
        dist = distanceToPoint(lookAhead_coord)
        sp.update(robot.getPosition(), point)
        
        if(dist < 0.3):
            robot.setMotion(0.2,0)

    robot.setMotion(0,0)

   
# Load a reference path 
#---------------Change Later----------
p = Path("exam2020.json")
path = p.getPath()
sp = ShowPath(path)

robot = Robot()
# print("Path loaded.")

# start time counter
start_time = time.time()  

while i < len(path):
    goTo()

# compute the elapsed time
elapsed_time = time.time() - start_time

mins, secs = divmod(elapsed_time, 60)
print('The program takes', mins, 'minutes and', secs,  'seconds to reach the target point.')