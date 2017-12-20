import simpleWorld, Robot, math as m, numpy as np
from HTWG_Robot_Simulator_AIN_V1.ParticleFilterPoseEstimator import *


def main():
    myWorld = simpleWorld.buildWorld()
    myRobot = Robot.Robot()
    myWorld.setRobot(myRobot, [4, 4, m.pi/2])

    myGrid = myWorld.getDistanceGrid()
    print("distance grid generated")
    myGrid.drawGrid()

    estimator = ParticleFilterPoseEstimator()
    estimator.initialize()

    motions = myRobot.curveDrive(1.0, 3, -m.pi);
    print(motions)

    for motion in motions:
        myRobot.move(motion)

    myWorld.close()


if __name__ == '__main__':
    main()