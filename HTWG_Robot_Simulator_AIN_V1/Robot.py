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
        self._K_p = 1.5
        self._K_d = None

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
        omega = v / r
        tau = delta_theta / omega
        n = int(tau/self.getTimeStep())

        return [[v, omega] for i in range(n)]

    # --------
    # returns a list of motion vectors for linear motion
    #
    def straightDrive(self, v, l):
        t = l / v
        n = int(t/self.getTimeStep())

        return [[v, 0.0] for i in range(n)]

    def followLine(self, start_point, end_point):
        l = np.sqrt((end_point[0] - start_point[0]) ** 2 + (end_point[1] - start_point[1]) ** 2)
        motions = self.straightDrive(1, l)
        for motion in motions:
            self.move(motion)
            error_distance, omega, velocity, delta_theta = self.p_controller(start_point, end_point)
            motions = self.curveDrive(velocity, error_distance, delta_theta)

    def p_controller(self, start_point, end_point):
        x, y, theta = self._world.getTrueRobotPose()
        current_position = np.array([x, y])
        # Regelabweichung e -> kuerzeste Distanz zwischen aktueller Position und Gerade, der gefolgt wird
        error_distance = np.linalg.norm(np.cross(end_point - start_point, start_point - current_position)) / np.linalg.norm(end_point - start_point)
        #print('Distanz -> ' + str(error_distance))
        omega = self._K_p * error_distance
        velocity = self._K_p * np.sqrt((current_position[0] - end_point[0]) ** 2 + (current_position[1] - end_point[1]) ** 2)
        delta_theta = theta - atan2(end_point[1] - current_position[1], end_point[0] - current_position[0])
        # links oder rechts von der Linie?
        sign = (current_position[0] - start_point[0])*(end_point[1] - start_point[1]) - (current_position[1] - start_point[1])*(end_point[0] - start_point[0])
        if sign < 0:
            print('Links')
        elif sign > 0:
            print('Rechts')
            omega *= -1
            velocity *= -1
            delta_theta *= -1
        else:
            print('Linie!')
            omega = 0.0

        return [error_distance, omega, velocity, delta_theta]

    def pd_controller(self):
        pass

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




