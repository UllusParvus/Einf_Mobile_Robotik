import simpleWorld, Robot, math as m, numpy as np
from HTWG_Robot_Simulator_AIN_V1.ParticleFilterPoseEstimator import *
from HTWG_Robot_Simulator_AIN_V1.PlotUtilities import *


def main():
    myWorld = simpleWorld.buildWorld()
    myRobot = Robot.Robot()
    robotStartPose = [4, 4, m.pi/2]
    myWorld.setRobot(myRobot, robotStartPose)

    dist_grid = myWorld.getDistanceGrid()

    fromPose = [robotStartPose[0]-1.5, robotStartPose[1]-1.5, robotStartPose[2] - m.pi/6]
    toPose = [robotStartPose[0]+1.5, robotStartPose[1]+1.5, robotStartPose[2] + m.pi/6]
    estimator = ParticleFilterPoseEstimator()
    estimator.setRobot(myRobot)
    estimator.initialize(fromPose, toPose, n=200)

    motions = myRobot.curveDrive(1.0, 4, -m.pi)
    counter = 1

    for motion in motions:
        myRobot.move(motion)
        estimator.integrateMovement(motion)

        if counter % 5 == 0:
            estimator.integrateMeasurement(myRobot.sense(), myRobot.getSensorDirections(), dist_grid)

        counter += 1
        myWorld.drawPoints(estimator._particles, color='green')

    real_pose = myRobot._world.getTrueRobotPose()
    estimated_pose = estimator.getPose()
    print('Echte Roboter-Position und -Ausrichung -> ', real_pose[0], real_pose[1], np.rad2deg(real_pose[2]))
    print('Vom Partikel-Filter geschätzte Position und Ausrichtung -> ', estimated_pose[0], estimated_pose[1], np.rad2deg(2*pi + estimated_pose[2]))
    print('')
    print('Kovarianz-Matrix')
    print(estimator.getCovariance())

    myWorld.close()


if __name__ == '__main__':
    main()