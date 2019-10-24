#!/usr/bin/python
# -*- coding: utf-8 -*-
# Object Trajectory Generator
import numpy as np 
from numpy.linalg import norm
import struct
import UR5Class
import time
import sys
import csv
#import json
import Transformations as tf
import os
import trajectoryGenerator

def getObjectPose(robot1Pose, robot2Pose):

	vector = (robot1Pose[0:3] - robot2Pose[0:3])/norm(robot1Pose[0:3] - robot2Pose[0:3])

	objectPose = robot1Pose
	objectPose[0:3] = robot1Pose[0:3] + (vector*norm/2)

	return objectPose

def getTrajectories(robot1Pose, robot2Pose, robot2Transform, step = 0.008, radius = 0.2, speed = 0.05, trajectoryType = "circularXY"):

	robot2PoseMatrix = tf.rotationVector2Matrix(robot2Pose, homogeneous = True)

	robot2PoseMatrix = np.dot(robot2Transform,robot2PoseMatrix)

	objectPose = getObjectPose(robot1Pose,tf.matrix2RotationVector(robot2PoseMatrix, homogeneous = True))

	objectTrajectory = trajectoryGenerator(objectPose, step, radius, speed, trajectoryType)

	vector = (robot1Pose[0:3] - robot2Pose[0:3])/norm(robot1Pose[0:3] - robot2Pose[0:3])

	robot1Trajectory = objectTrajectory

	for i in range(0, robot1Trajectory.shape[0]):

		robot1Trajectory[i,0:3] = robot1Trajectory[i,0:3] - (vector*norm/2)

	robot2Trajectory = objectTrajectory

	for i in range(0, robot2Trajectory.shape[0]):

		robot2Trajectory[i,0:3] = robot2Trajectory[i,0:3] + (vector*norm/2)

		robot2Matrix = tf.rotationVector2Matrix(robot2Trajectory[i,0:3], homogeneous = True)

		robot2Matrix = np.dot(inv(robot2Transform),robot2Matrix)

		robot2Trajectory[i] = tf.matrix2RotationVector(robot2Matrix, homogeneous = True)