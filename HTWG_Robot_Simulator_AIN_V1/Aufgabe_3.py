import simpleWorld, Robot, math as m, numpy as np
from HTWG_Robot_Simulator_AIN_V1.ParticleFilterPoseEstimator import *
from HTWG_Robot_Simulator_AIN_V1.PlotUtilities import *


def main():
    myWorld = simpleWorld.buildWorld()
    myRobot = Robot.Robot()
    robotStartPose = [4, 4, m.pi/2]
    myWorld.setRobot(myRobot, robotStartPose)

    dist_grid = myWorld.getDistanceGrid()

    fromPose = [robotStartPose[0]-1, robotStartPose[1]-1, robotStartPose[2] - m.pi/8]
    toPose = [robotStartPose[0]+1, robotStartPose[1]+1, robotStartPose[2] + m.pi/8]
    estimator = ParticleFilterPoseEstimator()
    estimator.setRobot(myRobot)
    estimator.initialize(fromPose, toPose, n=200)

    motions = myRobot.curveDrive(1.0, 3, -m.pi)
    counter = 1

    for motion in motions:
        myRobot.move(motion)
        estimator.integrateMovement(motion)

        if counter % 5 == 0:
            #plotPoseParticles(estimator._particles)
            #plotShow()
            estimator.integrateMeasurement(myRobot.sense(), myRobot.getSensorDirections(), dist_grid)
            #plotPoseParticles(estimator._particles)
            #plotShow()

        # Only every 5 times

        counter += 1
    real_pose = myRobot._world.getTrueRobotPose()
    estimated_pose = estimator.getPose()
    print('Echte Roboter-Position und -Ausrichung -> ', real_pose[0], real_pose[1], np.rad2deg(real_pose[2]))
    print('Vom Partikel-Filter geschätzte Position und Ausrichtung -> ', estimated_pose[0], estimated_pose[1], np.rad2deg(2*pi + estimated_pose[2]))
    print('')
    print('Kovarianz-Matrix')
    print(estimator.getCovariance())

    plotPoseParticles(estimator._particles)
    plotShow()

    myWorld.close()


if __name__ == '__main__':
    main()