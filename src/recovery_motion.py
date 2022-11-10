#!/usr/bin/python

from cartesian_interface.pyci_all import *
import numpy as np
import sys

if sys.version_info.major == 2:
    input = raw_input

# ros client
ci = pyci.CartesianInterfaceRos()

# get handle to postural task
postural = ci.getTask('Postural')

# save postural weight value
wstart = postural.getWeight()

# set a higher weight to recover from bad posture
wsize = wstart.shape[0]
wrecover = np.eye(wsize)  # w = 1 looks like a reasonable value
postural.setWeight(wrecover)

# wait for user input
input('press ENTER to set original gain to postural task')

# restore gain
postural.setWeight(wstart)
