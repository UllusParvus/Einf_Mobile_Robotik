import simpleWorld, Robot, math as m, numpy as np
from HTWG_Robot_Simulator_AIN_V1.ParticleFilterPoseEstimator import *
from HTWG_Robot_Simulator_AIN_V1.PlotUtilities import *


def main():
    myWorld = simpleWorld.buildWorld()
    myRobot = Robot.Robot()
    robotStartPose = [4, 4, m.pi/2]
    myWorld.setRobot(myRobot, robotStartPose)

    dist_grid = myWorld.getDistanceGrid()
    #print("distance grid generated")
    #dist_grid.drawGrid()

    fromPose = [robotStartPose[0]-1, robotStartPose[1]-1, robotStartPose[2] - m.pi/8]
    toPose = [robotStartPose[0]+1, robotStartPose[1]+1, robotStartPose[2] + m.pi/8]
    estimator = ParticleFilterPoseEstimator()
    estimator.setRobot(myRobot)
    estimator.initialize(fromPose, toPose, n=200)

    #plotPoseParticles(estimator._particles)
    #plotShow()

    motions = myRobot.curveDrive(1.0, 3, -m.pi)
    counter = 1

    for motion in motions:
        myRobot.move(motion)
        estimator.integrateMovement(motion)

        if counter % 5 == 0:
            plotPoseParticles(estimator._particles)
            plotShow()
            estimator.integrateMeasurement(myRobot.sense(), myRobot.getSensorDirections(), dist_grid)

            mean = estimator.getPose()
            print("mean x: ", mean[0])
            print("mean y: ", mean[1])
            print("mean theta: ", np.rad2deg(mean[2]))
            cov = estimator.getCovariance()
            #print("cov x: ", cov[0])
            print(cov)
            plotPoseParticles(estimator._particles)
            plotShow()

        # Only every 5 times

        counter += 1




    myWorld.close()


if __name__ == '__main__':
    main()