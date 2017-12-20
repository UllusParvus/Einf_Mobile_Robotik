import numpy as np

class ParticleFilterPoseEstimator():
    '''
    Estimates the current robot position with a particle filter (Monte-Carlo)
    '''

    def __init__(self):
        self._particles = None

    def initialize(self, poseFrom, poseTo, n=200):
        self._particles = np.empty((n, 3))