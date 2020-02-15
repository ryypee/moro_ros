#!/usr/bin/env python
import numpy as np
import math
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
import rospy
import pdb
import warnings
warnings.filterwarnings('error') # 32 sec error

class EKF:
    def __init__(self, state_vector_size, control_size, measurement_size):
        self.state_vector = np.zeros((state_vector_size, 1))
        self.cov_matrix = 1000. * np.identity(state_vector_size)
        self.q = np.zeros((control_size, control_size))
        self.R = np.zeros((measurement_size, measurement_size))
        self.motion_j_state = np.zeros((state_vector_size, state_vector_size))
        self.motion_j_noise = np.zeros((state_vector_size, control_size))
        self.obs_j_state = np.zeros((measurement_size, state_vector_size))
        self.Q = np.zeros((state_vector_size, state_vector_size))
        self.beacons = {1:[7.3, 3.0], 2:[1,1],3:[9,9],4:[1,8],5:[5.8,8]}
        #
        self.control = np.zeros((2,1))
        self.Z = np.zeros((2,1))
        self.v_sigma = [] # for sampling sigma
        self.w_sigma = [] # for sampling sigma
        self.prev_time_stamp = 0 # keeping the last time stamp
        self.prev_state = np.array((3,1))
        from nav_msgs.msg import Odometry
        self.gt = rospy.Subscriber('base_pose_ground_truth', Odometry, self.initialize_state_vector) # Initializing state_vector with ground truth
        #
        self.state_data_history = []
        self.ground_truth_state_history = []

    def initialize_state_vector(self, msg): # Function for initializing state_vector
        #print("initialize state", self.state_vector.shape)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])[2]
        self.state_vector[0] = x
        self.state_vector[1] = y
        self.state_vector[2] = self.wrap_to_pi(theta)
        self.prev_time_stamp = msg.header.stamp.secs + msg.header.stamp.nsecs*(10**-9)
        self.gt.unregister() # unregister subscriber. Function is implemented only once.


    def predict(self, odometry): # odometry added by me
        #TODO determine q-matrix
        #print('predict state', self.state_vector.shape)

        # Get v,w from odometry msg
        w = odometry.twist.twist.angular.z
        v = odometry.twist.twist.linear.x
        self.dt = (odometry.header.stamp.secs + odometry.header.stamp.nsecs*(10**-9))-self.prev_time_stamp
        #print(self.dt)
        #pdb.set_trace()
        #
        # get timestamp
        self.prev_time_stamp = odometry.header.stamp.secs + odometry.header.stamp.nsecs*(10**-9)
        #print('Seconds gone is', self.dt)
        #
        # form internal control vector
        self.control = np.array(([v,w]))
        #
        # determine q-matrix aka process noise
        self.q = np.array(([0.04, 0],[0,0.001])) # FOR TEST PURPOSES
        #
        self.propagate_state()
        self.calculate_cov()

    def update(self, msg): #
        #pdb.set_trace()
        #print('update state', self.state_vector.shape)
        #msg.pose.position.[x y z]     msg.pose.orientation.[x y z w]   msg.ids
        #print(msg.ids[0])
        self.cur_id = self.beacons[msg.ids[0]] # coordinates of current transmitter
        pos_x = msg.pose.position.x
        pos_y = msg.pose.position.y
        theta = self.wrap_to_pi(euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])[2])
        #print("Position is",[pos_x, pos_y], "Heading is:", theta)
        self.observation_jacobian_state_vector()
        
        floor = self.cov_matrix.dot(self.obs_j_state.transpose()).astype(np.float32)
        
        bottom = (self.obs_j_state.dot(self.cov_matrix).dot(self.obs_j_state.transpose()) + np.eye(2)*0.001).astype(np.float32)
        self.K = floor.dot(np.linalg.inv(bottom)) # K is 3x2
        expected_meas = self.measurement_model(self.state_vector)
        new_meas = self.measurement_model([pos_x, pos_y, theta])
        #FIXME correct it
        #FIXME
        tempterm = np.array(([new_meas[0] - expected_meas[0], [new_meas[1] - expected_meas[1]]]))
        #FIXME
        #FIXME correct it
       
        self.state_vector = self.state_vector + self.K.dot(tempterm) 
        try:
            self.cov_matrix = (np.eye(3) - self.K.dot(self.obs_j_state)).dot(self.cov_matrix)
        except RuntimeWarning:
            pdb.set_trace()
        
        self.prev_state = self.state_vector
        print(self.state_vector)
        #print(self.cov_matrix)



    def propagate_state(self):
        if self.control[1] == 0:
            term = self.control[0]
            x = self.state_vector[0] + self.control[0]*self.dt
            y = self.state_vector[1] + self.control[1]*self.dt
            theta = self.state_vector[2]
            #x = self.state_vector[0] + (term)*np.cos(self.state_vector[2]) # my #FIXME dt 
            #y = self.state_vector[1] - (term)*np.sin(self.state_vector[2]) # my #FIXME dt 
            #theta = self.wrap_to_pi(self.state_vector[2]) 
        else:
            term = self.control[0]/self.control[1]
            x = self.state_vector[0] + (term)*np.sin(self.state_vector[2] + self.control[1]*self.dt)
            y = self.state_vector[1] - (term)*np.cos(self.state_vector[2] + self.control[1]*self.dt)
            theta = self.wrap_to_pi(self.state_vector[2] + self.control[1]*self.dt) 
        self.state_vector = np.array([x,y,theta])
        


    def measurement_model(self,state):
        x = state[0]
        y = state[1]
        theta = state[2]
        px = self.cur_id[0]
        py = self.cur_id[1]

        r = np.sqrt((px-x)**2 + (py-y)**2)      #Distance
        phi = np.arctan2(py-y, px-x) - theta    #Bearing
        #phi = np.arctan((py - y)/(px - x)) - theta #FIXME test

        self.Z[0] = r
        self.Z[1] = phi #self.wrap_to_pi(phi)
        return self.Z
        #self.Z = np.array([r,phi])              



    def calculate_cov(self):
        self.Q = self.motion_j_noise.dot(self.q).dot(self.motion_j_noise.transpose())
        self.cov_matrix = self.motion_j_state.dot(self.cov_matrix).dot(self.motion_j_state.transpose()) + self.Q

        self.motion_jacobian_state_vector()
        self.motion_jacobian_noise_components()

    def motion_jacobian_state_vector(self):
        #print('Motion jacobian', self.state_vector.shape)
        if self.control[1] != 0:
            term = self.control[0]/self.control[1]
            row1term3 = 0 #term*np.cos(self.state_vector[2] + self.control[1]*self.dt) simple model experiment
            row2term3 = 0 #term*np.sin(self.state_vector[2] + self.control[1]*self.dt) simple model experiment
        else:
            row1term3 = -self.control[0]*np.sin(self.state_vector[2] + self.dt)
            row2term3 = -self.control[0]*np.cos(self.state_vector[2] + self.dt)
        #print(row1term3, row2term2)
        # if len(row1term3) > 1:
        #     #pdb.set_trace()
        #     pass
        self.motion_j_state = np.array(([1,0,row1term3],[0,1,row2term3],[0,0,1]))
        #print(self.motion_j_state) # 
        # self.motion_j_state
        pass

    def motion_jacobian_noise_components(self): # trailing zeros!
        #print('Motion noise jacobian', self.state_vector.shape)
        # TO DO
        #print(self.state_vector[1])
        if self.control[1] != 0: # if angular velocity is not zero
            row1term1 = np.sin(self.state_vector[2] + self.control[1]*self.dt)/self.control[1] # checked
            
            #row1term2 = (-np.sin(self.state_vector[2] + self.control[1]*self.dt) + self.control[1]*self.dt*np.cos(self.control[1]*self.dt))/(self.control[1]**2) # check
            row1term2 = (np.cos(self.control[1]*self.dt + self.state_vector[2])*self.control[0]*self.dt)/self.control[1] \
                - ((np.sin(self.control[1]*self.dt + self.state_vector[2])*self.control[0])/self.control[1]**2) # checked

            row2term1 = -np.cos(self.state_vector[2] + self.control[1]*self.dt)/self.control[1] # checked

            tempterm = self.state_vector[2] + self.control[1]*self.dt

            row2term2 = ((np.cos(tempterm)*self.control[0])/self.control[1]**2) \
                + ((self.dt*np.sin(tempterm)*self.control[0])/self.control[1])
            #row2term2 = -self.control[0]*(-np.cos(tempterm) - self.control[1]*self.dt*np.sin(tempterm)) # check

            row3term1 = 0
            row3term2 = self.dt
        else:
            row1term1 = self.dt #np.cos(self.state_vector[2]) # EXPERIMENT +dt taken out of equation
            row1term2 = 0
            row2term1 = self.dt #-np.sin(self.state_vector[2]) # EXPERIMENT +dt taken out of equation
            row2term2 = 0
            row3term1 = 0
            row3term2 = 0 #self.state_vector[2] # dt = 1, possibly wrong
        self.motion_j_noise = np.array(([row1term1, row1term2],[row2term1,row2term2],[row3term1,row3term2]))
        #print(row1term1, row1term2, row2term1, row2term2, row3term1, row3term2)

        # self.motion_j_noise
        #pass

    def observation_jacobian_state_vector(self):
        #print('Observation jacobian', self.state_vector.shape)
        # To DO
        row1term1 = (self.state_vector[0] - self.cur_id[0])/np.sqrt((self.state_vector[0] - self.cur_id[0])**2 + (self.state_vector[1] - self.cur_id[1])**2) #checked
        row1term2 = (self.state_vector[1] - self.cur_id[1])/np.sqrt((self.state_vector[0] - self.cur_id[0])**2 + (self.state_vector[1] - self.cur_id[1])**2) #checked
        row1term3 = 0
        row2term1 = (self.cur_id[1] - self.state_vector[1]) / ((self.cur_id[0] - self.state_vector[0])**2 + (self.cur_id[1] - self.state_vector[1])**2) #checked
        #row2term1 = (self.cur_id[1] - self.state_vector[1])/((self.cur_id[0] - self.state_vector[0])**2* \
         #   ((self.cur_id[1] - self.state_vector[1])**2/(self.cur_id[0] - self.state_vector[0])**2 + 1)) # matlab version
        #row2term2 = (self.cur_id[0] - self.state_vector[0]) / ((self.cur_id[0] - self.state_vector[0])**2 + (self.cur_id[1] - self.state_vector[1])**2)
        #row2term2 = -1/((self.cur_id[0] - self.state_vector[0])*((self.cur_id[1] - self.state_vector[1])**2/(self.cur_id[0] - self.state_vector[0])**2 + 1))
        row2term2 = -1/((((self.cur_id[1]-self.state_vector[1])**2)/(self.cur_id[0]-self.state_vector[0]))+(self.cur_id[0]- self.state_vector[0])) #checked
        row2term3 = -1
        self.obs_j_state = np.array(([row1term1, row1term2, row1term3],[row2term1,row2term2,row2term3]))
        #print(self.obs_j_state)
        # self.obs_j_state
        #pass

    def print_initials(self):
        pass
        #print("State vector is", self.state_vector)
        #print(self.cov_matrix)
        #print("The initial stated is {}").format(self.state_vector)
        #print("The initial cov. matrix is {}").format(self.cov_matrix)

    def wrap_to_pi(self,angle):
        #return np.arctan2(np.sin(angle), np.cos(angle))
        return (angle + np.pi) % (2 * np.pi) - np.pi

##======================
# def wrap_to_pi(self, num, lower=-0.5*np.pi, upper=0.5*np.pi, b=True):
#     from math import floor, ceil
#     # abs(num + upper) and abs(num - lower) are needed, instead of
#     # abs(num), since the lower and upper limits need not be 0. We need
#     # to add half size of the range, so that the final result is lower +
#     # <value> or upper - <value>, respectively.
#     res = num
#     if not b:
#         if lower >= upper:
#             raise ValueError("Invalid lower and upper limits: (%s, %s)" %
#                              (lower, upper))

#         res = num
#         if num > upper or num == lower:
#             num = lower + abs(num + upper) % (abs(lower) + abs(upper))
#         if num < lower or num == upper:
#             num = upper - abs(num - lower) % (abs(lower) + abs(upper))

#         res = lower if res == upper else num
#     else:
#         total_length = abs(lower) + abs(upper)
#         if num < -total_length:
#             num += ceil(num / (-2 * total_length)) * 2 * total_length
#         if num > total_length:
#             num -= floor(num / (2 * total_length)) * 2 * total_length
#         if num > upper:
#             num = total_length - num
#         if num < lower:
#             num = -total_length - num

#         res = num * 1.0  # Make all numbers float, to be consistent

#     return res
##======================