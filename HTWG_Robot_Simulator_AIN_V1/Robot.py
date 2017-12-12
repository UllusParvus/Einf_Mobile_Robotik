# class Robot.
#
# This class define methods to move a robot and to sense the world.
# The robot does not know its pose!
#
# O. Bittel
# AIN V1.0; 26.09.2017


from math import *
import numpy as np
import random
from HTWG_Robot_Simulator_AIN_V1.OdometryPoseEstimator import *
from library.transformations_lib import *
from HTWG_Robot_Simulator_AIN_V1 import SensorUtilities
from HTWG_Robot_Simulator_AIN_V1.graphics import *
from HTWG_Robot_Simulator_AIN_V1 import World


class Robot:
    # --------
    # init: creates robot.
    #
    def __init__(self):
        self._size = 0.4  # robot diameter
        self._T = 0.1  # time step
        self._world = None  # robot's world; is set by setWorld()

        # Motion noise parameter:
        self._k_d = 0.02 * 0.02  # velocity noise parameter = 0.02m*0.02m / 1m
        self._k_theta = (5.0 * 5.0/360.0) * (pi/180.0)  # turning rate noise parameter = 2deg*2deg/360deg * (1rad/1deg)
        self._k_drift = (2.0 * 2.0)/1.0 * (pi/180.0) ** 2  # drift noise parameter = 2deg*2deg / 1m
        self._maxSpeed = 1 # maximum speed
        self._maxOmega = pi # maximum rotational speed

        # SigmaMotion
        self._SigmaMotion = np.zeros((2,2))

        # Controller parameter
        self._K_p = 0.04
        self._K_d = 1

        # Controller parameter for exerice 3
        self._K_p_1 = 0.08
        self._K_p_2 = 0.04
        self._K_p_3 = 0.02

        self.error_distance = [0, 0]

        # Sensor parameter (x-axis is in forward direction):
        self._numberOfSensors = 28
        dTheta = 270.0 / (self._numberOfSensors-1)
        self._sensorDirections = [(-135.0 + dTheta * i) * (pi/180.0) for i in range(self._numberOfSensors)]
        self._maxSenseValue = 5.0  # Maximum sensor value for each sensor beam
        self._sensorNoise = 0.01  # standard deviation of distance measurement for 1m


    def getTimeStep(self):
        return self._T

    def setTimeStep(self, T):
        self._T = T

    # --------
    # returns the diameter of the robot
    #
    def getSize(self):
        return self._size

    # --------
    # returns the direction of the sensors
    #
    def getSensorDirections(self):
        return self._sensorDirections

    # --------
    # returns the maximal possible sensor value
    #
    def getMaxSenseValue(self):
        return self._maxSenseValue

    # --------
    # returns a list of motion vectors for angular motion
    #
    def curveDrive(self, v, r, delta_theta):
        if delta_theta == 0:
            return [[0, 0.0]]
        elif delta_theta < 0:
            omega = - v / r
        elif delta_theta > 0:
            omega = v / r

        tau = abs(delta_theta / omega)
        n = int(tau/self.getTimeStep())

        return [[v, omega] for i in range(n)]

    # --------
    # returns a list of motion vectors for linear motion
    #
    def straightDrive(self, v, l):
        t = l / v
        n = int(t/self.getTimeStep())

        return [[v, 0.0] for i in range(n)]

    def followLine(self, start_point, end_point, controller_mode, v, tolerance=0.0):
        if controller_mode != 'p' and controller_mode != 'pd':
            raise RuntimeError('No valid controller mode!')

        x, y, theta = self._world.getTrueRobotPose()
        current_position = np.array([x,y])
        distance_to_goal = np.sqrt((current_position[0]-end_point[0])**2 + (current_position[1]-end_point[1])**2)
        while distance_to_goal > tolerance:
            self.calculate_error(start_point, end_point)
            if controller_mode == 'p':
                self.move([v, self.p_controller()])
            elif controller_mode == 'pd':
                self.move([v, self.pd_controller()])
            x, y, theta = self._world.getTrueRobotPose()
            current_position = np.array([x, y])
            distance_to_goal = np.sqrt((current_position[0] - end_point[0]) ** 2 + (current_position[1] - end_point[1]) ** 2)
            print(distance_to_goal)
        print('GOAL REACHED WITHIN TOLERANCE!')

    def followLineLocal(self, estimator, start_point, end_point, controller_mode, v, tolerance=0.0):
        if controller_mode != 'p' and controller_mode != 'pd':
            raise RuntimeError('No valid controller mode!')

        x, y, theta = estimator.getPose()
        current_position = np.array([x,y])
        distance_to_goal = np.sqrt((current_position[0]-end_point[0])**2 + (current_position[1]-end_point[1])**2)
        while distance_to_goal > tolerance:
            self.calculate_error_local(estimator, start_point, end_point)
            if controller_mode == 'p':
                movement = [v, self.p_controller()]
                self.move(movement)
                estimator.integrateMovement(movement,0)
            elif controller_mode == 'pd':
                movement = [v, self.pd_controller()]
                self.move(movement)
                estimator.integrateMovement(movement,0)

            x, y, theta = estimator.getPose()
            current_position = np.array([x, y])
            distance_to_goal = np.sqrt((current_position[0] - end_point[0]) ** 2 + (current_position[1] - end_point[1]) ** 2)
            print(distance_to_goal)
        print('GOAL REACHED WITHIN TOLERANCE!')

    def calculate_error(self, start_point, end_point):
        self.error_distance[1] = self.error_distance[0]
        x, y, theta = self._world.getTrueRobotPose()
        current_position = np.array([x, y])

        # Regelabweichung e -> kuerzeste Distanz zwischen aktueller Position und Gerade, der gefolgt wird
        self.error_distance[0] = np.linalg.norm(np.cross(end_point - start_point, start_point - current_position)) / np.linalg.norm(end_point - start_point)
        sign = (current_position[0] - start_point[0]) * (end_point[1] - start_point[1]) - \
               (current_position[1] - start_point[1]) * (end_point[0] - start_point[0])

        if sign < 0:
            self.error_distance[0] *= -1

    def calculate_error_local(self, estimator, start_point, end_point):
        self.error_distance[1] = self.error_distance[0]
        x, y, theta = estimator.getPose()
        current_position = np.array([x, y])

        # Regelabweichung e -> kuerzeste Distanz zwischen aktueller Position und Gerade, der gefolgt wird
        self.error_distance[0] = np.linalg.norm(np.cross(end_point - start_point, start_point - current_position)) / np.linalg.norm(end_point - start_point)
        sign = (current_position[0] - start_point[0]) * (end_point[1] - start_point[1]) - \
               (current_position[1] - start_point[1]) * (end_point[0] - start_point[0])

        if sign < 0:
            self.error_distance[0] *= -1

    def p_controller(self):
        return self._K_p * self.error_distance[0]

    def pd_controller(self):
        d_e = (self.error_distance[0] - self.error_distance[1])/self.getTimeStep()
        return self._K_d * d_e + self.p_controller()

    def gotoGlobal(self, v, p, tol):
        x, y, theta = self._world.getTrueRobotPose()
        theta_to_goal = atan2(p[1]-y, p[0]-x)
        delta_theta = theta - theta_to_goal
        motions = self.curveDrive(0.05, 0.1, -delta_theta)
        for motion in motions:
            self.move(motion)
        self.followLine(np.array([x, y]), p, 'pd', v, tol)

    def followPolyline(self, v, poly, tol=0.0):
        for point in poly:
            self.gotoGlobal(v, point, tol)

    def gotoLocal(self, v, p, tol):
        """
        Moves robot towards a point in the local coordinate system.
        :param v: speed
        :param p: point in local CS
        :param tol: tolerance
        """
        estimator = OdometryPoseEstimator()
        estimator.setInitialPose((1,1,pi/2))
        # Transform p to global coordinate system
        x = np.append(p, [[0], [1]])
        t_OR = np.dot(trans((1, 1, 0)), rot2trans(rotz(np.deg2rad(90))))
        x_in_global = np.dot(t_OR, x)
        x_in_global = np.array([[x_in_global[0]], [x_in_global[1]]])
        x, y, theta = estimator.getPose()
        theta_to_goal = atan2(p[1], p[0])
        motions = self.curveDrive(0.2, 0.1, theta_to_goal)
        for motion in motions:
            self.move(motion)
            estimator.integrateMovement(motion, 0)
        x, y, theta = estimator.getPose()
        self.followLineLocal(estimator, np.array([x, y]), x_in_global, 'pd', v, tol)


    def followWalls(self, v_start, currentPose):
        desired_distance_to_wall = self.getSize()/2 + 0.4
        while True:
            dists = self.sense()
            directions = self.getSensorDirections()
            min_dist = min(x for x in dists if x is not None)
            min_index = dists.index(min_dist)
            min_dist_angle = directions[min_index]
            lines_l = SensorUtilities.extractLinesFromSensorData(dists, directions)
            lines_g = SensorUtilities.transform(lines_l, self._world.getTrueRobotPose())
            self._world.drawPolylines(lines_g)

            omega_distance_to_wall = self._K_p_1 * (min_dist - desired_distance_to_wall)

            if min_dist_angle > 0:
                diff_orientation_to_wall = min_dist_angle - pi / 2
            elif min_dist_angle < 0:
                diff_orientation_to_wall = min_dist_angle + pi / 2
            else:
                diff_orientation_to_wall = 0.0

            omega_orientation_to_wall = self._K_p_2 * diff_orientation_to_wall
            omega = omega_distance_to_wall + omega_orientation_to_wall

            if dists[len(dists)//2] is not None and dists[len(dists)//2 - 1] is not None:
                frontal_distance = dists[len(dists) // 2] + dists[len(dists) // 2 - 1] / 2
                v = frontal_distance * self._K_p_3
                if frontal_distance < 2 * min_dist:
                    sum_left = sum(dists[:14])
                    sum_right = sum(dists[14:])

            else:
                v = v_start



            self.move([v, omega])


    # --------
    # move the robot for the next time step T by the
    # command motion = [v,omega].
    # Returns False if robot is stalled.
    #
    def move(self, motion):
        v = motion[0]
        omega = motion[1]

        # translational and rotational speed is limited:
        if omega > self._maxOmega:
            omega = self._maxOmega
        if omega < -self._maxOmega:
            omega = -self._maxOmega
        if v > self._maxSpeed:
            v = self._maxSpeed
        if v < -self._maxSpeed:
            v = -self._maxSpeed

        #print("motion ", v, omega*180/pi)

        # Add noise to v:
        sigma_v_2 = (self._k_d / self._T) * abs(v)
        v_noisy = v + random.gauss(0.0, sqrt(sigma_v_2))

        # Add noise to omega:
        sigma_omega_tr_2 = (self._k_theta / self._T) * abs(omega)  # turning rate noise
        sigma_omega_drift_2 = (self._k_drift / self._T) * abs(v)  # drift noise
        omega_noisy = omega + random.gauss(0.0, sqrt(sigma_omega_tr_2))
        omega_noisy += random.gauss(0.0, sqrt(sigma_omega_drift_2))

        # Set SigmaMotion:
        self._SigmaMotion[0,0] = sigma_v_2
        self._SigmaMotion[1,1] = sigma_omega_tr_2 + sigma_omega_drift_2

        # Move robot in the world (with noise):
        d_noisy = v_noisy * self._T
        dTheta_noisy = omega_noisy * self._T
        return self._world.moveRobot(d_noisy, dTheta_noisy, self._T)

    # --------
    # returns SigmaMotion from the last move step
    #
    def getSigmaMotion(self):
        return self._SigmaMotion

    # --------
    # sense and returns distance measurements for each sensor beam.
    # If a sensor beams senses no obstacle distance value is set to None.
    #
    def sense(self):
        sensorDistNoisy = []
        sensorDist = self._world.sense()
        for d in sensorDist:
            if d is not None:
                # print "d: ", d
                sigma2 = self._sensorNoise**2 * d
                d += random.gauss(0.0, sqrt(sigma2))
            sensorDistNoisy.append(d)
        return sensorDistNoisy

    # --------
    # Sense boxes.
    # Return [distances, angles] for all sensed boxes.
    # Return None, if no boxes are visible.
    #
    def senseBoxes(self):
        distAngles = self._world.senseBox()
        if distAngles is None or distAngles[0] == []:
            return None
        else:
            return distAngles

    # --------
    # set world
    #
    def setWorld(self, world):
        self._world = world




