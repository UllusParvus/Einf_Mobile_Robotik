from math import *
import numpy as np
import emptyWorld, officeWorld
import Robot
from HTWG_Robot_Simulator_AIN_V1 import Kinematics as Kin
from library.transformations_lib import *
from HTWG_Robot_Simulator_AIN_V1 import testWorld

def a1_1():
    myWorld = emptyWorld.buildWorld()
    myRobot = Robot.Robot()
    myWorld.setRobot(myRobot, [10, 10, pi / 2])

    motions = myRobot.curveDrive(0.5, 4, np.deg2rad(0));

    for motion in motions:
        myRobot.move(motion)

    # Simulation schliessen:
    myWorld.close()

def a1_2():
    myWorld = emptyWorld.buildWorld()
    myRobot = Robot.Robot()
    myWorld.setRobot(myRobot, [10, 5.5, pi / 2])

    motions = myRobot.straightDrive(10, 10);

    for motion in motions:
        myRobot.move(motion)

    # Simulation schliessen:
    myWorld.close()


def a2_a():
    p1 = np.array([3, 3])
    p2 = np.array([77, 77])
    myWorld = emptyWorld.buildWorld(80, 80)
    myRobot = Robot.Robot()
    myWorld.setRobot(myRobot, [3, 7, pi / 4])

    polyline = [p1, p2]
    myWorld.drawPolyline(polyline)

    # controller_mode = 'p' or 'pd'
    myRobot.followLine(p1, p2, 'pd', 1, tolerance=0.4)
    myWorld.close()


def a2_b():
    point = [17, 22]
    myWorld = emptyWorld.buildWorld(30,30)
    myWorld.addBox(point[0],point[1])

    myRobot = Robot.Robot()
    myWorld.setRobot(myRobot, [3, 5, np.deg2rad(-110)])
    myRobot.gotoGlobal(0.4, np.array(point), 0.3)

    myWorld.close()

def a2_c():
    list_points = [[5,5], [10,5], [10,10]]
    myWorld = emptyWorld.buildWorld(30,30)
    for point in list_points:
        myWorld.addBox(point[0],point[1])
    myRobot = Robot.Robot()
    myWorld.setRobot(myRobot, [1, 1, np.deg2rad(-110)])
    myRobot.followPolyline(0.5, list_points, 0.5)

def a3_a():
    myWorld = emptyWorld.buildWorld(30,30)
    point = np.array([[10], [-10]])
    x = np.append(point, [[0], [1]])
    print(x)
    t_OR = np.dot(trans((1,1,0)), rot2trans(rotz(np.deg2rad(90))))
    x_in_global = np.dot(t_OR, x)
    print(x_in_global)
    myWorld.addBox(x_in_global[0], x_in_global[1])
    myRobot = Robot.Robot()
    myWorld.setRobot(myRobot, [1, 1, np.deg2rad(90)])

    myRobot.gotoLocal(0.5, point, 0.1)

def a3_b():
    myWorld = testWorld.buildWorld()
    myRobot = Robot.Robot()
    myWorld.setRobot(myRobot, [2,4.5, np.deg2rad(0)])

    myRobot.followWalls(0.1, myWorld.getTrueRobotPose())


def main():
    #a1_1()
    #a1_2()
    #a2_a()
    #a2_b()
    #a2_c()
    #a3_a()
    a3_b()

if __name__ == '__main__':
    main()