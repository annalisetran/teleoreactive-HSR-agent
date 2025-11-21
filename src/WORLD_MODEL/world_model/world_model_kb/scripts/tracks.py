#!/usr/bin/env python3

# Dependencies:
# pip install tabulate
# pip install filterpy

import numpy as np
import sys
import filterpy
from tabulate import tabulate
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

ASSOSCIATION_WEIGHTS = { 
        "mahalanobis_dist": 0.5,
        "hue_correlation": -1 # turning off hue matching. Was using -1
        }

# The Tracks class represents an object being tracked. When we see a new object in a scene we 
# instatiate a new Tracks object and add it to the list of objects being tracked.

class Tracks(object):
    nextTrackId = 0
    """Model:
        State Variables:
            loc x
            loc y
            loc z

        dt = time interval/refresh rate
        f.x = state estimate (initialise to 1)
        f.u = velocity
        f.H = measurement matrix
        f.Q = process noise
        
    """
    #def __init__(self, step_size, detection, objectId, object_class_id):
    def __init__(self, step_size, detection, object_id):
        super(Tracks, self).__init__()
        self.step_size = step_size        # step size
        self.kf = KalmanFilter(dim_x=6, dim_z=3)

        # Useful things for plots
        self.plot_patch = None
        self.plot_text = None

        # state estimate matrix
        '''
        # Linear velocity model
        self.kf.F = np.array([[1, dt, 0, 0, 0, 0],
                             [0, 1, 0, 0, 0, 0],
                             [0, 0, 1, dt, 0, 0],
                             [0, 0, 0, 1, 0, 0],
                             [0, 0, 0, 0, 1, dt],
                             [0, 0, 0, 0, 0, 1]
                             ])         
        '''
        # Stationary model
        self.kf.F = np.array([[1, 0, 0, 0, 0, 0],
                             [0, 1, 0, 0, 0, 0],
                             [0, 0, 1, 0, 0, 0],
                             [0, 0, 0, 1, 0, 0],
                             [0, 0, 0, 0, 1, 0],
                             [0, 0, 0, 0, 0, 1]
                             ])         

        # u = control input vector
        self.kf.u = 0.
        # H = measurement matrix
        self.kf.H = np.array([[1, 0, 0, 0, 0, 0],
                             [0, 0, 1, 0, 0, 0],
                             [0, 0, 0, 0, 1, 0]
                            ])

        # R = measurement noise matrix
        self.kf.R = np.eye(3) * (0.3**2) # 0.2m std dev. measurement noise
        # q = process noise
        self.kf.Q = Q_discrete_white_noise(dim=2, block_size=3, dt=self.step_size, var=0.5**2) # 0.1m std. dev process noise
        #print("Q:\n{}".format(tabulate(self.kf.Q)))
        self.kf.x = np.array([[0, 0, 0, 0, 0, 0]]).T
        # P = covariance matrix
        self.kf.P = np.eye(6) * 500.

        self.trackId = self.nextTrackId
        self.nextTrackId += 1
        self.objectId = object_id
        self.objectClassId = detection.class_id
        # add new features here for a track
        self.num_observations = 0

        # Update using the current detection
        self.update(detection)

    def mahalanobis_dist_of(self, point):
        # Calculate the likelihood of point being produced by this track
        # see https://en.wikipedia.org/wiki/Multivariate_normal_distribution#Likelihood_function
        x = point.reshape((-1, 1))
        mean = self.kf.x[[0, 2, 4]]
        cov = self.kf.P[[0, 2, 4], :][:, [0, 2, 4]]
        #print(x - mean)
        #print(np.max(cov))
        return filterpy.stats.mahalanobis(x, mean, cov)

    def mahalanobis_dist_of_tmp(self, point, occluded):
        # Calculate the likelihood of point being produced by this track
        # see https://en.wikipedia.org/wiki/Multivariate_normal_distribution#Likelihood_function
        x = point.reshape((-1, 1))
        mean = self.kf.x[[0, 2, 4]]
        cov = self.kf.P[[0, 2, 4], :][:, [0, 2, 4]]
        #print(x - mean)
        #print(np.max(cov))
        dist = filterpy.stats.mahalanobis(x, mean, cov)

        if occluded:
            dist = dist * 1.1

        return dist
    
    # This function allows us to ustilse different properties of the Tracks object to minimise the 
    # error in the association. E.g. if the object has a different class then it will be considered 
    # different and the cost will be set to the maximum, else it will calculate the costs
    def calculate_assosciation_cost(self, detection):

        if (detection.class_id != self.objectClassId):
            return sys.float_info.max
        else:
            # mahalanobis_dist = self.mahalanobis_dist_of(detection.point, detection.occluded)
            mahalanobis_dist = self.mahalanobis_dist_of(detection.point)

            # maybe need to add the adjustor here
            cost = ASSOSCIATION_WEIGHTS["mahalanobis_dist"] * mahalanobis_dist

            return cost

    def update(self,detection):
        if detection.point is not None:
            point = detection.point         # check if this line is needed
            self.kf.update(detection.point)


    def predict(self):
        self.kf.predict()

    def get_position(self):
        return self.kf.x[[0, 2, 4]]



