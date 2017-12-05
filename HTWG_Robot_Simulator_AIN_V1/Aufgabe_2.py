from math import *
import numpy as np
import emptyWorld
import Robot
from HTWG_Robot_Simulator_AIN_V1 import Kinematics as Kin

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
    myRobot.followLine(p1, p2, 'pd', 0.5, tolerance=0.4)
    myWorld.close()


def a2_b():
    point = [17, 22]
    myWorld = emptyWorld.buildWorld(30,30)
    myWorld.addBox(point[0],point[1])

    myRobot = Robot.Robot()
    myWorld.setRobot(myRobot, [3, 5, np.deg2rad(-110)])
    myRobot.gotoGlobal(0.4, np.array(point), 0.3)

    myWorld.close()

def main():
    #a1_1()
    #a1_2()
    a2_a()
    #a2_b()

if __name__ == '__main__':
    main()