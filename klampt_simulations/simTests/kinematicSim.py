#!/usr/bin/python
##Author: Saurav Agarwal
##E-mail: sagarw10@uncc.edu
## The file demonstrates:
##   1. Adding rooms and walls to the environment (refer to buildWorld.py as well)
##   2. Setting up a robot
##   3. Perform collision checking
##   4. Modify the robot configurations and visualize
##   5. Adding text objects and modifying them

import sys
from klampt import *
from klampt import vis
from klampt.robotsim import setRandomSeed
from klampt.vis.glcommon import GLWidgetPlugin
from klampt import RobotPoser
from klampt.model import ik,coordinates
from klampt.math import so3
import klampt.model.collide as collide
import time
import numpy as np
import math
import buildWorld as bW
sys.path.append("./kinematics/")
from sphero6DoF import sphero6DoF
from kobuki import kobuki
from turtlebot import turtlebot
from RRTPlanner import find_path
from decimal import Decimal

if __name__ == "__main__":

    # Hardcoded world file for now...
    sys.argv.append('simpleWorld.xml')

    if len(sys.argv)<=1:
        print "USAGE: kinematicSim.py [world_file]"
        exit()

    ## Creates a world and loads all the items on the command line
    world = WorldModel()
    for fn in sys.argv[1:]:
        res = world.readFile(fn)
        if not res:
            raise RuntimeError("Unable to load model "+fn)

    coordinates.setWorldModel(world)

    ## Get walls
    ## Two rooms separated by a wall with a window
    #bW.getDoubleRoomWindow(world, 8, 8, 1.2)

    ## Two rooms separated by a wall with a door
    bW.getDoubleRoomDoor(world, 8, 8, 1)

    ## Add the world to the visualizer
    vis.add("world",world)

    vp = vis.getViewport()
    vp.w,vp.h = 1200,800
    vis.setViewport(vp)

    ## Create robot object. Change the class to the desired robot.
    ## Also, make sure the robot class corresponds to the robot in simpleWorld.xml file
    #robot = kobuki(world.robot(0), vis)
    #robot.setAltitude(0.01)

    robot = turtlebot(world.robot(0), "turtle", vis)
    robot.setAltitude(0.02)

    #vis.autoFitCamera()
    vis.addText("textCol", "No collision")
    vis.setAttribute("textCol","size",24)
    collisionFlag = False
    collisionChecker = collide.WorldCollider(world)

    ## On-screen text display
    vis.addText("textConfig","Robot configuration: ")
    vis.setAttribute("textConfig","size",24)
    vis.addText("textbottom","WCS: X-axis Red, Y-axis Green, Z-axis Blue",(20,-30))




    # PLANNER CALLED HERE
    robot.setConfig([3, 3, math.pi])
    q_init = robot.getConfig()
    q_goal = [3, -3, math.pi / 2]
    print("Finding path from q_init to q_goal...")
    startTime = time.time()
    path = find_path(world, robot, q_init, q_goal, 1000)
    planTime = time.time() - startTime
    print("Plan time: %d seconds" % planTime)

    robot.setConfig(q_init)

    start_xy = q_init[:1]
    start_angle = q_init[2]

    curr_path_item = 0
    target_xy = path[0][:2]
    target_angle = path[0][2]

    angle_diff = math.atan2(math.sin(target_angle - start_angle), math.cos(target_angle - start_angle))

    # Only setting to suppress warnings
    dist_moved = 0
    target_mag = 0

    # Determines how quickly the robot will move with respect to time. This is arbitrary.
    angle_step = 1
    xy_step = .3

    is_rotating = True  # is translating when False
    rads_rotated = 0

    reached_goal = False

    print "Starting visualization window#..."

    ## Run the visualizer, which runs in a separate thread
    vis.setWindowTitle("Visualization for kinematic simulation")

    # print(next(collisionFlag))
    vis.show()
    simTime = 500
    startTime = time.time()
    oldTime = startTime

    while vis.shown() and not reached_goal:
        vis.lock()

        deltaT = time.time() - oldTime
        oldTime = time.time()
        # robot.velControlKin(vel, omega, deltaT)

        curr_config = robot.getConfig()
        curr_xy = curr_config[:2]
        curr_angle = math.degrees(curr_config[2])

        #My logic for determining motion based on path
        #If arrived at destination, switch to next configuration.
        if is_rotating:
            # Check to see how far away from angle destination
            if rads_rotated < abs(angle_diff):
                # Rotate towards target angle.

                # Apply movement.
                robot.velControlKin(0, np.sign(angle_diff) * angle_step, deltaT)

                rads_rotated = rads_rotated + angle_step * deltaT

            else:
                # Finished with rotation. Snap rotation to target in case of overshooting.
                robot.setConfig([curr_config[0], curr_config[1], target_angle])
                curr_path_item = curr_path_item + 1
                is_rotating = False
                # Get ready for translation on next vis update.
                target_xy = path[curr_path_item - 1][:2]
                x_diff = target_xy[0] - curr_xy[0]
                y_diff = target_xy[1] - curr_xy[1]
                dist_moved = 0
                target_mag = math.sqrt(x_diff ** 2 + y_diff ** 2)
                if curr_path_item < len(path):
                    # Prepare for next rotation after the translation.
                    rads_rotated = 0
                    start_angle = target_angle
                    target_angle = path[curr_path_item][2]
                    #angle_diff = target_angle - start_angle
                    #angle_diff = abs((angle_diff + 180) % 360) - 180
                    angle_diff = math.atan2(math.sin(target_angle - start_angle), math.cos(target_angle - start_angle))

        else:
            # Check how far away from x,y destination
            if dist_moved < target_mag:
                # Translate towards target xy

                # Apply movement
                robot.velControlKin(xy_step, 0, deltaT)

                # Check how far the robot has translated
                dist_moved = dist_moved + xy_step * deltaT
                curr_config = robot.getConfig()

            else:
                # Finished with translation. Snap in case of overshooting.
                robot.setConfig([target_xy[0], target_xy[1], curr_config[2]])
                is_rotating = True

                if curr_path_item == len(path):
                    reached_goal = True

        q = robot.getConfig()
        q2f = [ '{0:.2f}'.format(elem) for elem in q]
        strng = "Robot configuration: " + str(q2f)
        vis.addText("textConfig", strng)

        ## Checking collision
        collisionFlag = False
  
        for iR in range(world.numRobots()):
            collRT2 = collisionChecker.robotObjectCollisions(world.robot(iR))
            for i,j in collRT2:
                collisionFlag = True
                strng = world.robot(iR).getName() + " collides with " + j.getName()
                # print(strng)
                vis.addText("textCol", strng)
                vis.setColor("textCol", 0.8500, 0.3250, 0.0980)


        if not collisionFlag:
            vis.addText("textCol", "No collision")
            vis.setColor("textCol", 0.4660, 0.6740, 0.1880)

        vis.unlock()
        time.sleep(0.01)
    vis.clearText()

    print "Ending klampt.vis visualization."
    vis.kill()
