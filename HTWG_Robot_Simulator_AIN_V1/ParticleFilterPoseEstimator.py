import numpy as np, random
from library.transformations_lib import *
from bisect import bisect_left
import math as m
from HTWG_Robot_Simulator_AIN_V1.PlotUtilities import *

class ParticleFilterPoseEstimator():
    '''
    Estimates the current robot position with a particle filter (Monte-Carlo)
    '''

    def __init__(self):
        self._particles = None
        self._robot = None

    def setRobot(self, robot):
        self._robot = robot

    def initialize(self, poseFrom, poseTo, n=200):
        self._particles = np.empty((n, 4))
        for i in range(n):
            self._particles[i, 0] = np.random.uniform(poseFrom[0], poseTo[0])
            self._particles[i, 1] = np.random.uniform(poseFrom[1], poseTo[1])
            self._particles[i, 2] = np.random.uniform(poseFrom[2], poseTo[2])
            self._particles[i, 3] = 1 # weighting for all particles
        self._particles[199] = [4,4,m.pi,1]


    def integrateMovement(self, steeringAction):
        # add random noise to the steering action
        T = self._robot.getTimeStep()
        sigma_motion = self._robot.getSigmaMotion()

        # TODO: Vermutung: Für jeden Partikel unterschiedlich verrauschter Steuerbefehl?
        for i in range(0, self._particles.shape[0]):
            # TODO: [0.0, np.sqrt(sigma_motion[0,0])] oder [-np.sqrt(sigma_motion[0,0]), np.sqrt(sigma_motion[0,0])]
            v = steeringAction[0] + random.gauss(0.0, np.sqrt(sigma_motion[0,0]))
            omega = steeringAction[1] + random.gauss(0.0, np.sqrt(sigma_motion[1,1]))

            # TODO: Lt. Definition der Gauss-Verteilung sind die Parameter µ und s², nicht s (sqrt(s²))
            self._particles[i][0] = self._particles[i][0] + T * v * np.cos(self._particles[i][2] + 0.5*T*omega)
            self._particles[i][1] = self._particles[i][1] + T * v * np.sin(self._particles[i][2] + 0.5*T*omega)
            self._particles[i][2] = self._particles[i][2] + T * omega

    def integrateMeasurement(self, dist_list, alpha_list, distantMap):
        #calculate likelihoodfield, chap. 5 slide 75
        sigma = 0.5 #self._robot  ._sensorNoise

        for i in range(0, self._particles.shape[0]):
            if i is 199:
                print("b")
            p = 1
            for j in range(0, len(dist_list)):
                # only calculate dist to next obstacle if a distance was actually measured by the laser
                if dist_list[j] is not None:
                    # Calculate global coordinates of the measured points
                    global_coords = []
                    angle_of_particle = self._particles[i][2]

                    real_angle = (((2*m.pi) + alpha_list[j]) % (2*m.pi)) + angle_of_particle
                    global_coords.append(self._particles[i][0] + np.cos(real_angle) * dist_list[j])
                    global_coords.append(self._particles[i][1] + np.sin(real_angle) * dist_list[j])
                    # Plot spots#
                    global_coords.append(0)
                    #l = []
                    #l.append(global_coords)

                    #plotPoseParticles(l)
                    # if the global coordinates for the measured distance at the position of a particle lies out of the map-bounds, the particle gets a minimal weight
                    if int(global_coords[0]/distantMap.cellSize) > distantMap.xSize or int(global_coords[0]/distantMap.cellSize) < 0 \
                            or int(global_coords[1]/distantMap.cellSize) > distantMap.ySize or int(global_coords[1]/distantMap.cellSize) < 0:
                        p = p * 0.001
                    else:
                        # Get distance to next obstacle
                        y = int(global_coords[0]/distantMap.cellSize)
                        x = int(global_coords[1]/distantMap.cellSize)
                        dist = distantMap.grid[y][x]
                        # Weight the probability with the self created factor f according to a gaussian function we created.
                        # This function represents what distance to the next obstacle we still assume as good.
                        gaussian_value = self.gaussian(0.0, sigma, dist)
                        # Normalize gaussian to [0,1]
                        gaussian_value = (gaussian_value - 0) / (self.gaussian(0.0, sigma, 0.0) - 0)
                        p = p * gaussian_value

            self._particles[i][3] = p

        # resampling, "Roulette-Rad"
        new_particles = np.empty((200,4))
        intervals = [0]
        for i in range(0, self._particles.shape[0]):
            intervals.append(intervals[i] + self._particles[i][3])

        for i in range(0, self._particles.shape[0]):
            z = random.uniform(0, intervals[len(intervals)-1])
            idx_found = intervals.index(self.find_lt(intervals, z))
            new_particles[i][0:4] = self._particles[idx_found]
        self._particles = new_particles

    def gaussian(self, mu, sigma, x):
        return np.exp(- ((mu - x) ** 2) / ((sigma ** 2) * 2.0)) / np.sqrt(2.0 * np.pi * (sigma ** 2))

    def find_lt(self, list, x):
        'Find rightmost value less than x'
        i = bisect_left(list, x)
        if i:
            return list[i - 1]
        raise ValueError

    def index(self, list, x):
        'Locate the leftmost value exactly equal to x'
        i = bisect_left(list, x)
        if i != len(list) and list[i] == x:
            return i
        raise ValueError

    def getPose(self):
        return self._particles.mean(axis=0)


    def getCovariance(self):
        return np.cov(self._particles, axis=0)