#!/usr/bin/python
# -*- coding: utf-8 -*-
# Movement_test
import numpy as np
from numpy.linalg import norm
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
import trajectoryGenerator
import StepGenerator

HOST = "192.168.0.98" # The remote host 
PORT = 30003 # The same port as used by the server

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
s.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)

def getData(host, port):
  s.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
  data = s.recv(1060)
  return data
# getData abre uma conexao com o robo e recebe o pacote de dados de 1060 Bytes

def sendString(host, port, string, move_time = 8, pose = False):
  string = "[" + str(string[0]) + "," +  str(string[1]) + "," + str(string[2]) + "," + str(string[3]) + "," + str(string[4]) + "," + str(string[5]) + "]"
  if (pose == True):
  	p = "p"
  else:
  	p = ""
  str_data = "movej(" + p + string + ", t = " + str(move_time) + ")" + "\n"
  s.send(str_data.encode('ascii'))
  return

def speedJ(host, port, string, a = 2*np.pi):
  string = "[" + str(string[0]) + "," +  str(string[1]) + "," + str(string[2]) + "," + str(string[3]) + "," + str(string[4]) + "," + str(string[5]) + "]"
  str_data = "speedj(" + string + ", a = " + str(a) + ",t=0.02)" + "\n"
  s.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
  s.send(str_data.encode('ascii'))
  return

def main(args):
  print ("Starting Program")

  delta_standard_DH = np.mat([[7.80880090239748336e-05, 0.361257734372265993, 0.00128388035686166635, 1.67232993846963135e-05, 2.02354943719599362e-05, 0], \
    [-0.000718642187888640649, 0.00106284384336133905, -0.022893992683020014, -0.00115732902891929612, 0.000201414435319735574, 0], \
    [7.02198637382578372e-05, -395.302340315824551, 396.777096992026259, -1.47374645443299634,0.000169498815833071803, 0.000364725429982712401], \
    [2.91984009971350678e-05, -1.42023254669109278, 1.33410045447338699, 0.0861037286066216462, -3.46593927803766182e-05, -2.71063161709674666e-05]])

  delta_standard_DH_2 = np.mat([[ -5.39038176483263552e-06, 0.200686268169445209, 0.00228952454238523506, 2.04485825460639469e-05, -1.56897709565794351e-05, 0],\
    [ 0.00039024637623907843, 0.000904178045744563359, 0.0145652098260125283, -0.000690055586142879207, 0.000644539557413503772, 0],\
    [ 0.000178790506571227525, 399.392832822527851, -396.49020940525736, -2.90172143203552535, 0.000311791168683808739, 0.000378711630321493242], \
    [ 7.05887359599974621e-05, 1.01499272342048541, -0.906943504886603802, -6.39125177018525026, 2.3011110588447593e-05, 5.9590107063629152e-05]])
  # Dados de calibracao do robo

  ur5 = UR5Class.UR5Class(delta_standard_DH_2)

  # process = threading.Thread(target=speedJ,args=[HOST,PORT])
  # process.start()

  time.sleep(0.3)

  ur5.setRobotData(getData(HOST, PORT))

  initialPose = ur5.getPosition()
  initialPose[3:6] = tf.rotationVector2RollPitchYaw(initialPose[3:6])

  print("InitialPose (rpy): ", initialPose)
  print("InitialPose calculated: ", ur5.ur5_direct_kinematics(ur5.getJointPosition(), vector = True, apply_offset = True))

  # jointControl = ur5.controlLoopPseudoInverse(initialPose + np.array([0,0,0.1,0,0,0]), initialPose)
  # jointControlConst = jointControl
  # speedJ(HOST, PORT, jointControl)

  StepGenerator.stepGenerator(ur5,'cartesian',ur5.getPosition(),0,float(args[1]))

main(sys.argv)