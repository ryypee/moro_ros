#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
import pickle

with open('ground_truth.pickle','rb') as file:
    gt_data = pickle.load(file)
with open('states.pickle','rb') as file:
    states_data = pickle.load(file)
#    print(states_data)
with open('cov_params.pickle','rb') as file:
    cov_data = pickle.load(file)
cov = np.array(cov_data)
covx = []
covy = []
cov_theta = []
for item in cov:
	covx.append(3*item[0])
	covy.append(item[1])
	cov_theta.append(item[2])
mcovx = [-3*i for i in covx]
#print(states_data[:0])
gt_data = np.array(gt_data)
states_data = np.array(states_data)
xaxis = np.arange(len(states_data))
fig, ax = plt.subplots(4)
##
ax[0].plot(xaxis, states_data[:,0],'b',label='Estimated X')
ax[0].plot(xaxis, gt_data[:,0],'r',label='GT X')
ax[0].legend()
##
ax[1].plot(xaxis, states_data[:,1],'b',label='Estimated Y')
ax[1].plot(xaxis, gt_data[:,1],'r',label='GT Y')
ax[1].legend()
##
ax[2].plot(xaxis, states_data[:,2],'b',label='Estimated Th')
ax[2].plot(xaxis, gt_data[:,2],'r',label='GT Th')
ax[2].legend()
##
#ax[3].plot(states_data[:,0],states_data[:,1],'b')
#ax[3].plot(gt_data[:,0],gt_data[:,1],'r')
#plt.show()
