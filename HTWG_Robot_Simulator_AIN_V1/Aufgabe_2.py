from math import *
import numpy as np
import emptyWorld
import Robot
from HTWG_Robot_Simulator_AIN_V1 import Kinematics as Kin

def a1_1():
    myWorld = emptyWorld.buildWorld()
    myRobot = Robot.Robot()
    myWorld.setRobot(myRobot, [10, 10, pi / 2])

    motions = myRobot.curveDrive(0.5, 4, np.deg2rad(180));

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
    p2 = np.array([15, 15])
    print(str(p2 - p1))
    myWorld = emptyWorld.buildWorld()
    myRobot = Robot.Robot()
    myWorld.setRobot(myRobot, [p1[0], p1[1], pi / 4])

    polyline = [p1, p2]
    myWorld.drawPolyline(polyline)

    myRobot.followLine(p1, p2)
    myWorld.close()

def main():
    #a1_1()
    #a1_2()
    a2_a()

if __name__ == '__main__':
    main()