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
    
    #robot = sphero6DoF(world.robot(0), "sphero", vis)

    ## Display the world coordinate system
    vis.add("WCS", [so3.identity(),[0,0,0]])
    vis.setAttribute("WCS", "size", 24)


    #print "Visualization items:"
    #vis.listItems(indent=2)

    #vis.autoFitCamera()
    vis.addText("textCol", "No collision")
    vis.setAttribute("textCol","size",24)
    collisionFlag = False
    collisionChecker = collide.WorldCollider(world)

    ## On-screen text display
    vis.addText("textConfig","Robot configuration: ")
    vis.setAttribute("textConfig","size",24)
    vis.addText("textbottom","WCS: X-axis Red, Y-axis Green, Z-axis Blue",(20,-30))

    print "Starting visualization window#..."

    ## Run the visualizer, which runs in a separate thread
    vis.setWindowTitle("Visualization for kinematic simulation")

    #print(next(collisionFlag))
    vis.show()
    simTime = 500
    startTime = time.time()
    oldTime = startTime


    # PLANNER CALLED HERE
    robot.setConfig([3, 3, 180])
    q_init = robot.getConfig()
    q_goal = [3, -3, 90]
    print("Finding path from q_init to q_goal...")
    path = find_path(world, robot, q_init, q_goal)
    curr_path_item = 0
    is_rotating = True  # is translating when False
    target_xy = path[curr_path_item][:1]
    target_angle = path[curr_path_item][2]

    while vis.shown() and (time.time() - startTime < simTime):
        vis.lock()
        ## You may modify the world here.
        ## Specifying change in configuration of the robot

        
        ## Turtlebot  2DoF Non-holonomic
        ## The controls are in terms of forward velocity (along x-axis) and angular velocity (about z-axis)
        ## The state of the robot is described as (x, y, alpha)
        ## The kinematics for converting the control inputs to the state vector is given in the function turtlebot.controlKin

        # Forward velocity
        vel = 0.5*math.cos(time.time())
        # Angular velocity
        omega = math.sin(time.time())
        deltaT = time.time() - oldTime
        oldTime = time.time()
        robot.velControlKin(vel, omega, deltaT)

        curr_config = robot.getConfig()
        curr_xy = curr_config[:1]
        curr_angle = curr_config[2]

        # My logic for determining motion based on path
        # If arrived at destination, switch to next configuration.
        if is_rotating:
            # Check to see how far away from angle destination
            if curr_angle is not target_angle:
                # TODO: rotate towards target angle
                # TODO: Then, if has passed target, set equal to target. Set flag to translate if not at final goal config.
                pass
        else:
            # Check how far away from x,y destination
            if curr_xy is not target_xy:
                # TODO: translate towards target xy
                # TODO: Then, if has passed target, set equal to target. Set flag to rotate if not at final goal config.
                    # TODO: If not at goal config, increment curr_path_item and set appropriate xy and t goals
                pass
            pass

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
                print(strng)
                vis.addText("textCol", strng)
                vis.setColor("textCol", 0.8500, 0.3250, 0.0980)


        if not collisionFlag:
            vis.addText("textCol", "No collision")
            vis.setColor("textCol", 0.4660, 0.6740, 0.1880)

        vis.unlock()
        #changes to the visualization must be done outside the lock
        time.sleep(0.01)
    vis.clearText()

    print "Ending klampt.vis visualization."
    vis.kill()