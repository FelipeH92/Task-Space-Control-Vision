#!/usr/bin/python
# -*- coding: utf-8 -*-
# Step generator
import numpy as np
from numpy.linalg import norm
from scipy.io import savemat
import matplotlib.pyplot as plot
import struct
import UR5Class
import socket
import time
import sys
import csv
#import json
import Transformations as tf
import os
import threading

def stepGenerator(ur5,stepType,initArray,positionToStep,step):

	if (stepType == 'joint'):

		print('\n\n\n')
		print('Step Generator:')
		print('Init Array is (joint space): ' + str(initArray))
		print('Step is (radians): ' + str(step))

		initArray[positionToStep] = initArray[positionToStep] + step

		targetPose = ur5.ur5_direct_kinematics(initArray, vector = True, rpy = True, apply_offset = True)

		print('Final Array is (joint space): ' + str(initArray))
		print('Final Array is (Cartesian space rpy): ' + str(targetPose))
	elif(stepType == 'cartesian'):

		print('\n\n\n')
		print('Step Generator:')
		print('Init Array is (Cartesian space): ' + str(initArray))
		print('Step is (Cartesian): ' + str(step))

		targetPose = initArray
		targetPose[positionToStep] = targetPose[positionToStep] + step

		print('Final Array is (Cartesian space rpy): ' + str(targetPose))
	else:
		exit(1)

	return targetPose