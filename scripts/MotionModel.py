import sys
import numpy as np
import math
import random

# Assign random seed when testing

# How to choose alphas. 
# How to check if the model works.
# How to implement the sampling function. return var/6 * sum_1^12 rand(-1,1)
# Check which angles must lie within [-pi, pi]

class MotionModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 5]
    """

    def __init__(self):

        """
        TODO : Initialize Motion Model parameters here
        """
        a = 0. 
        self.alphas = np.array([a,a,a,a,a,a])


    def sample(self, var):
        """
        Randomly sample from zero mean gaussian with variance var
        """
        return random.gauss(0, var) 

    def truncate_angle(self, theta):
        """
        Truncates theta to lie betwee [-pi, pi]
        """
        if theta > np.pi:
            return - (np.pi - (theta - np.pi))
        elif theta < -np.pi:
            return np.pi - (-theta - np.pi)
        else:
            return theta

    def update(self, u_t0, u_t1, x_t0):
        """
        param[in] u_t0 : particle state odometry reading [x, y, theta] at time (t-1) [odometry_frame]   
        param[in] u_t1 : particle state odometry reading [x, y, theta] at time t [odometry_frame]
        param[in] x_t0 : particle state belief [x, y, theta] at time (t-1) [world_frame]
        param[out] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        """

        rot1 = np.arctan2(u_t1[1] - u_t0[1], u_t1[0] - u_t0[0]) - u_t0[2]
        # rot1 = self.truncate_angle(rot1)
        trans = ((u_t1[0] - u_t0[0])**2 + (u_t1[1] - u_t0[1])**2)**0.5
        rot2 = u_t1[2] - u_t0[2] - rot1
        # rot2 = self.truncate_angle(rot2)

        rot1_hat = rot1 - self.sample(self.alphas[0] * rot1 + \
                self.alphas[1] * trans)
        # rot1_hat = self.truncate_angle(rot1_hat)
        trans_hat = trans - self.sample(self.alphas[2] * trans + \
                self.alphas[3] * self.truncate_angle(rot1 + rot2))
        rot2_hat = rot2 - self.sample(self.alphas[0] * rot2 + \
                self.alphas[1] * trans)
        # rot2_hat = self.truncate_angle(rot2_hat)

        x_prime = x_t0[0] + trans_hat * np.cos(x_t0[2] + rot1_hat)
        y_prime = x_t0[1] + trans_hat * np.sin(x_t0[2] + rot1_hat)
        theta_prime = x_t0[2] + rot1_hat + rot2_hat
        # theta_prime = self.truncate_angle(theta_prime)

        x_t1 = np.array([x_prime, y_prime, theta_prime])



        return x_t1

if __name__ == "__main__":
    pass
