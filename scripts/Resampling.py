import numpy as np
import random
import pdb

class Resampling:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 4.3]
    """

    def __init__(self):

        pass

    def multinomial_sampler(self, X_bar):

        """
        param[in] X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
        param[out] X_bar_resampled : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
        """

        assert np.sum(X_bar[:,-1]) == 1
        
        M = len(X_bar)
        wt = X_bar[:,3]

        X_bar_resampled = np.zeros_like(X_bar)
        M = len(X_bar)

        for m in range(M):
            ii = np.argmax(np.random.multinomial(M,wt))
            X_bar_resampled[m,:] = X_bar[ii,:]        	

        return X_bar_resampled

    def low_variance_sampler(self, X_bar):

        """
        param[in] X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
        param[out] X_bar_resampled : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
        """

        assert np.isclose(np.sum(X_bar[:,-1]), 1)

        X_bar_resampled = np.zeros_like(X_bar)
        M = len(X_bar)

        X_state = X_bar[:,0:3]
        wt = X_bar[:,3]

        r = random.uniform(0,1.0/M)
        c = wt[0]
        ii = 0
        for m in range(M):
            U = r + ((m)*1.0)/M
            while U>c:
                ii+=1
                c = c + wt[ii]
            X_bar_resampled[m,:] = X_bar[ii,:]


        return X_bar_resampled

if __name__ == "__main__":

    # X_bar = np.array([[1,1,1,0.2],
                                             # [2,2,2,0.1],
					  # [3,3,3,0.5],
					  # [4,4,4,0.05],
					  # [5,5,5,0.005],
					  # [6,6,6,0.005],
					  # [7,7,7,0.1],
					  # [8,8,8,0.04]])
    X_bar = np.array([[1,1,1,0.125],
   					  [2,2,2,0.125],
					  [3,3,3,0.125],
					  [4,4,4,0.125],
					  [5,5,5,0.125],
					  [6,6,6,0.125],
					  [7,7,7,0.125],
					  [8,8,8,0.125]])


    X_sampledd = X_bar
    for i in range(100):
        # X_sampled = Resampling().multinomial_sampler(X_sampledd)
    # print (X_sampled)
        X_sampled = Resampling().low_variance_sampler(X_sampledd)
        X_sampledd = X_sampled

    print (X_sampled)
    # pdb.set_trace()
