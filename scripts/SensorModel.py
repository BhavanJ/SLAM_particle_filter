import numpy as np
import math
import time
from matplotlib import pyplot as plt
# from scipy.stats import norm
import scipy.integrate as integrate
import pdb

from MapReader import MapReader
import bresenham

# np.random.seed(4)

def visualize_map(occupancy_map):
    fig = plt.figure()
    # plt.switch_backend('TkAgg')
    mng = plt.get_current_fig_manager();  # mng.resize(*mng.window.maxsize())
    plt.ion(); plt.imshow(occupancy_map, cmap='Greys'); plt.axis([0, 800, 0, 800]);


def visualize_raycast(ray):
    x_locs = [x[0] for x in ray]
    y_locs = [x[1] for x in ray]
    scat = plt.scatter(x_locs, y_locs, c='r', marker='o')
    plt.pause(0.1)
    scat.remove()


class SensorModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 6.3]
    """

    def __init__(self, occupancy_map):

        """
        TODO : Initialize Sensor Model parameters here
        """
        self.occupancy_map = occupancy_map
        self.deg_2_rad = np.pi/180
        self.zmax = 8000
        self.lambda_short =  0.02
        self.sigma_sq_hit = 80*3.

        self.z_hit = 9.5
        self.z_short = 10.
        self.z_max = 1.
        self.z_rand = 40.

        self.z_sum = self.z_hit + self.z_short + self.z_max + self.z_rand
        self.z_hit /= self.z_sum
        self.z_max /= self.z_sum
        self.z_rand /= self.z_sum
        self.z_short /= self.z_sum

    def ray_cast(self, pos, angle):
        angle *= self.deg_2_rad
        theta = pos[2] + angle
        x = (pos[0]/10)
        y = (pos[1]/10)

        x += (25*np.cos(pos[2]))
        x = int(x)
        y += (25*np.sin(pos[2]))
        y = int(y)

        x_initial, y_initial = x, y

        # max_dist = 800
        max_dist = np.max(self.occupancy_map.shape)

        stride = 5
        dist = self.zmax/10
        try:
            self.occupancy_map[y,x] != -1
        except:
            pdb.set_trace()
        while (self.occupancy_map[y,x] != -1):
            if (self.occupancy_map[y,x] > 0.9):
                dist = min(((x - x_initial)**2 + (y - y_initial)**2)**0.5, max_dist)
                break

            x += stride * np.cos(theta)
            y += stride * np.sin(theta)
            x = int(x)
            y = int(y)

        vis_flag = 0
        if vis_flag:
            finalRayPoints = list(bresenham.bresenham(int(x_initial), int(y_initial), int(x), int(y)))
            visualize_raycast(finalRayPoints)

        return dist

    def get_Nu(self,z_tk,z_tk_star):
        return (1.0/math.sqrt(2*math.pi*self.sigma_sq_hit))*math.exp( ((z_tk - z_tk_star)**2)/(-2.0*self.sigma_sq_hit)  )

    def get_p_hit(self,z_tk,z_tk_star):

        if 0 <= z_tk <= self.zmax:
            Nu = self.get_Nu(z_tk,z_tk_star)

            # pdb.set_trace()
            # o =integrate.quad(lambda x: self.get_Nu(x,z_tk_star), 0, self.zmax) 
            # eta = 2*math.pi*self.sigma_sq_hit*self.sigma_sq_hit
            # eta = eta**0.5
            # eta = 1.0/eta
            # eta = 1.0/(1e-11 + integrate.quad(lambda x: self.get_Nu(x,z_tk_star), 0, self.zmax)[0])
            return Nu
        else:
            return 0.0

    def get_p_short(self,z_tk,z_tk_star):
        if 0 <= z_tk <= z_tk_star:
            eta = 1.0/(1.0 - math.exp(-1.0*self.lambda_short*z_tk_star) )
            return eta*(self.lambda_short)*math.exp(-1.0*self.lambda_short*z_tk)
        else:
            return 0.0

    def get_p_max(self,z_tk):
        #TODO:BJ Verify
        if z_tk == self.zmax:
            return 1.0
        else:
            return 0.0

    def get_p_rand(self,z_tk):
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
        q = 0

#         try: self.occupancy_map[x_t1[1]/10,x_t1[0]/10] == -1
        # except:
#             pdb.set_trace()
 
        if self.occupancy_map[int(x_t1[1]/10),int(x_t1[0]/10)] == -1:
            return 0

        for k in range(0,180,20):
            z_tk      = z_t1_arr[k]
            z_tk_star = self.ray_cast(x_t1, k)
            p_hit     = self.get_p_hit(z_tk,z_tk_star)
            p_short   = self.get_p_short(z_tk,z_tk_star)
            p_max     = self.get_p_max(z_tk)
            p_rand    = self.get_p_rand(z_tk)

            p = self.z_hit*p_hit + self.z_short*p_short + self.z_max*p_max + self.z_rand*p_rand + 1e-10
            try: q += math.log(p,10)
            except: pdb.set_trace()

        return q    
 
def main():
    src_path_map = '../data/map/wean.dat'
    src_path_log = '../data/log/robotdata2.log'

    map_obj = MapReader(src_path_map)
    occupancy_map = map_obj.get_map() 
    logfile = open(src_path_log, 'r')

    sensor_model = SensorModel(occupancy_map)
    vis_flag = 1

    """
    Monte Carlo Localization Algorithm : Main Loop
    """
    if vis_flag:
        visualize_map(occupancy_map)

    first_time_idx = True
    X_bar = np.array([4250, 2270, 0])
    for time_idx, line in enumerate(logfile):

        # Read a single 'line' from the log file (can be either odometry or laser measurement)
        meas_type = line[0] # L : laser scan measurement, O : odometry measurement
        meas_vals = np.fromstring(line[2:], dtype=np.float64, sep=' ') # convert measurement values from string to double

        odometry_robot = meas_vals[0:3] # odometry reading [x, y, theta] in odometry frame
        time_stamp = meas_vals[-1]

        # if ((time_stamp <= 0.0) | (meas_type == "O")): # ignore pure odometry measurements for now (faster debugging) 
            # continue

        if (meas_type == "L"):
             odometry_laser = meas_vals[3:6] # [x, y, theta] coordinates of laser in odometry frame
             ranges = meas_vals[6:-1] # 180 range measurement values from single laser scan
        
        print ("Processing time step " + str(time_idx) + " at time " + str(time_stamp) + "s")

        if (first_time_idx):
            u_t0 = odometry_robot
            first_time_idx = False
            continue
        if (meas_type == "L"):
                z_t = ranges
                x_t1 = X_bar
                for r in z_t:
                    sensor_model.ray_cast(x_t1, r)
                
 

if __name__=='__main__':
    main()
