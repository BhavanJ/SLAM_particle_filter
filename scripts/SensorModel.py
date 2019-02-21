import numpy as np
import math
import time
from matplotlib import pyplot as plt
from scipy.stats import norm
import scipy.integrate as integrate
import pdb

from MapReader import MapReader

class SensorModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 6.3]
    """

    def __init__(self, occupancy_map):

        """
        TODO : Initialize Sensor Model parameters here
        """
        self.zmax = 100
        self.lambda_short = 10
        self.sigma_sq_hit = 10


    def get_Nu(z_tk,z_tk_star):
        return 1.0/math.sqrt(2*math.pi*self.sigma_sq_hit)*math.exp( ((z_tk - z_tk_star)**2)/(-2.0*self.sigma_sq_hit)  )

    def get_p_hit(z_tk,z_tk_star):

        if 0 <= z_tk <= self.zmax:
            Nu = self.get_Nu(z_tk,z_tk_star)
            eta = 1.0/(integrate.quad(lambda x: self.get_Nu(x,z_tk_star), 0, self.zmax))
            return eta*Nu
        else:
            return 0.0

    def get_p_short(z_tk,z_tk_star):
        if 0 <= z_tk <= z_tk_star:
            eta = 1.0/(1.0 - math.exp(-1.0*self.lambda_short*z_tk_star) )
            return eta*(self.lambda_short)*math.exp(-1.0*self.lambda_short*z_tk)
        else:
            return 0.0

    def get_p_max(z_tk):
        #TODO:BJ Verify
        if z_tk == self.zmax:
            return 1.0
        else:
            return 0.0

    def get_p_rand(z_tk):
        if 0<= z_tk < self.zmax:
            return 1.0/self.zmax
        else:
            return 0.0


    def beam_range_finder_model(self, z_t1_arr, x_t1):
        """
        param[in] z_t1_arr : laser range readings [array of 180 values] at time t
        param[in] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        param[out] prob_zt1 : likelihood of a range scan zt1 at time t
        """

        """
        TODO : Add your code here
        """

        return q    
 
if __name__=='__main__':
    pass