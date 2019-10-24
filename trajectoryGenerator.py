#!/usr/bin/python
# -*- coding: utf-8 -*-
# Generate Path
import numpy as np 
from numpy.linalg import norm
from numpy.linalg import inv
import struct
import UR5Class
import time
import sys
import csv
#import json
import Transformations as tf
import os
from mpl_toolkits.mplot3d import Axes3D

class trajectoryGenerator:


	def __init__(self, startPose, step = 0.008, radius = 0.2, speed = 0.05, trajectoryType = "circularXY", direction = "left"):

		self.step = step
		self.startPose = startPose
		self.speed = speed
		self.radius = radius
		self.trajectoryType = trajectoryType
		self.direction = direction

		self.stepVector = speed*step

		self.maxStepNumber = self.getMaxStepCounter()

		self.database = np.zeros([self.maxStepNumber,6])

	def setRadius(self, radius):
		self.radius = radius

	def setStartPose(self, startPose):
		self.startPose = startPose

	def setSpeed(self, speed):
		self.speed = speed
		self.updateStepVector()

	def setStep(self, step):
		self.step = step
		self.updateStepVector()
		self.maxStepNumber = self.getMaxStepCounter()
		self.database = np.zeros([self.maxStepNumber,6])

	def updateStepVector(self, speed):
		self.stepVector = self.speed*self.step

	def getMaxStepCounter(self):

		if (self.trajectoryType == "circularXY"):
			counter = (0.2 + self.radius + 2*np.pi*self.radius)/self.stepVector
			return int(counter+1)
		else:
			return None

	def generateTrajectory(self):

		if (self.trajectoryType == "circularXY"):
			self.generateCircularTrajectory()
		else:
			return None

	def generateCircularTrajectory(self):

		distanceFromCenter = 0;
		distanceFromInitialPoint = 0;
		previousPoint = self.startPose[np.newaxis];

		self.database[0,:] = self.startPose

		dataCounter = 1

		if (self.direction == "left"):
			dirAux = 1
		else:
			dirAux = -1

    	# First step - Z up

		while (distanceFromInitialPoint < 0.2):
			nextPoint = previousPoint + np.array([0, 0, self.stepVector, 0, 0, 0])[np.newaxis]
			self.database[dataCounter,:] = nextPoint

			dataCounter = dataCounter + 1

			previousPoint = nextPoint
			distanceFromInitialPoint = norm(nextPoint[0,0:3] - self.startPose[0:3][np.newaxis])

    	# Second step - Initial radius

		startPoseAux = nextPoint

		while (distanceFromCenter <= self.radius):
			rotInv = inv(tf.pose2Matrix(previousPoint[0], rotOnly = True))
			rotInv6 = np.eye(6)
			rotInv6[0:3,0:3] = rotInv
			point = np.dot(rotInv6,previousPoint.transpose())

			nextPoint = point.transpose() + np.array([0, 0, dirAux*self.stepVector, 0, 0, 0])[np.newaxis]
			
			rotInv = inv(rotInv)
			rotInv6[0:3,0:3] = rotInv
			nextPoint = np.dot(rotInv6,nextPoint.transpose()).transpose()
			
			self.database[dataCounter,:] = nextPoint
        
			dataCounter = dataCounter + 1
			previousPoint = nextPoint
			distanceFromCenter = norm(nextPoint[0,0:3] - startPoseAux[0,0:3])

        # Final step - Radius

		stepAngle = dirAux*self.stepVector/distanceFromCenter

		circumference = 2*np.pi*distanceFromCenter
		traveledCircumference = 0

		while (traveledCircumference <= circumference):
			directionVector = np.zeros((1,3))
			directionVector[0,:] = previousPoint[0,0:3] - startPoseAux[0,0:3]
			directionVector[0,2] = 0
			directionVector = directionVector

			tangentVector = np.dot(((directionVector)/norm(directionVector))[np.newaxis],tf.Rot_z(np.pi/2))
			incrementPosition = np.dot(self.stepVector*tangentVector,tf.Rot_z(stepAngle))
			incrementPose = np.concatenate((incrementPosition, np.array([[0, 0, 0]])), axis=1)
			nextPoint = previousPoint + incrementPose
			self.database[dataCounter,:] = nextPoint
	        
			dataCounter = dataCounter + 1
			if (dataCounter >= self.maxStepNumber):
				break
			previousPoint = nextPoint
			traveledCircumference = traveledCircumference + self.stepVector;


	def saveDatabase(self, path = "/home/nascimento/Projects/MESTRADO - Task Space Control/generatedPaths/UR5/", fileName = "Arm1"):

		if not os.path.exists(path):
			os.makedirs(path)

		np.save(path + fileName, self.database)

	def loadDatabase(self, path = "/home/nascimento/Projects/MESTRADO - Task Space Control/generatedPaths/UR5/", fileName = "Arm1"):

		if not os.path.exists(path):
			print("[ERROR] File not found: " + path + fileName + ".npy")
			return None

		self.database = np.load(path + fileName + ".npy")