import numpy as np
import scipy as scipy
from numpy.random import uniform
import scipy.stats
import rospy

class PF:
    def __init__(self, n, range_x, range_y):
        self.N = n
        self.rangeX = range_x
        self.rangeY = range_y
        self.particles = np.empty((self.N, 3))
        self.weights = np.empty((self.N))
        self.create_particles()
        self.prev_time_stamp = 0
        self.initialized = False
        self.dt = 0
        self.control = np.empty((2,1))
        self.beacons = {1:[7.3, 3.0], 2:[1,1],3:[9,9],4:[1,8],5:[5.8,8]}
        self.R = 0.05

        from nav_msgs.msg import Odometry
        self.gt = rospy.Subscriber('base_pose_ground_truth', Odometry, self.initialize)

    def initialize(self, msg):
        self.prev_time_stamp = msg.header.stamp.secs + msg.header.stamp.nsecs*(10**-9)
        self.gt.unregister() # unregister subscriber. Function is implemented only once.
        self.create_particles()
        self.initialized = True
        self.gt.unregister()


    def create_particles(self):
        self.particles[:,0] = uniform(0,self.rangeX,size=self.N)
        self.particles[:,1] = uniform(0,self.rangeY,size=self.N)
        self.particles[:,2] = uniform(0, 2*np.pi, size=self.N)
        self.weights.fill(1./self.N)

    def predict(self, odometry):
        w = odometry.twist.twist.angular.z
        v = odometry.twist.twist.linear.x
        #print("Odometry (v,w):", v, w)
        self.dt = (odometry.header.stamp.secs + odometry.header.stamp.nsecs*(10**-9))-self.prev_time_stamp # timestamps
        self.prev_time_stamp = odometry.header.stamp.secs + odometry.header.stamp.nsecs*(10**-9) # timestamps
        self.control[0] = v
        self.control[1] = w
        self.q = np.array(([0.04, 0],[0,.05]))
        self.propagate_state()
        #print(np.mean(self.particles[:,0]), np.mean(self.particles[:,1]))

    def propagate_state(self):
        std = [0.05, 0.04]
        N = len(self.particles)

        # move in the (noisy) commanded direction
        distL = (self.control[0] * self.dt) + (np.random.randn(N) * std[0]) # 0,05 linear command noise
        distA = (self.control[1] * self.dt) + (np.random.randn(N) * std[1])
        if self.control[1] != 0:
            term = (self.control[0] + np.random.randn(1)[0]*std[0])/(self.control[1] + np.random.randn(1)[0]*std[1])
            self.particles[:,0] = self.particles[:,0] - term*np.sin(self.particles[:,2])+ term*np.sin(self.particles[:,2]+(self.dt * distA))
            self.particles[:,1] = self.particles[:,1] + term*np.cos(self.particles[:,2])- term*np.cos(self.particles[:,2]+(self.dt * distA))
            self.particles[:, 2] += self.control[1] + (np.random.randn(N) * std[1])*self.dt # 0,04 angular command noise
            self.particles[:, 2] %= 2 * np.pi

        else:
            term = self.control[0] + np.random.randn(1)[0]*std[0]
            self.particles[:,0] = self.particles[:,0] + self.control[0]*np.cos(self.particles[:,2])*self.dt + (np.random.randn(N) * std[0]) #self.control[0]*self.dt
            self.particles[:,1] = self.particles[:,1] + self.control[0]*np.sin(self.particles[:,2])*self.dt + (np.random.randn(N) * std[0]) #self.control[0]*self.dt
            #theta = self.wrap_to_pi(self.state_vector[2])
            
        #self.state_vector = np.array([x,y,theta])
    def update(self,msg):
        self.weights.fill(1.)
        info = msg.markers
        for i in range(len(info)):
            cur_id = self.beacons[info[i].ids[0]]
            print(cur_id)
            marker = info[i]
            landmark = np.array(([marker.pose.position.x, marker.pose.position.y]))# marker.pose.position
            rng = np.linalg.norm(self.particles[:,0:2] - cur_id, axis=1)
            self.weights *= scipy.stats.norm(rng, self.R).pdf(landmark)


