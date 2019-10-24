#!/usr/bin/python
# -*- coding: utf-8 -*-
# Dual Task Space Control
import numpy as np
import argparse
from numpy.linalg import norm
from numpy.linalg import inv
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
import trajectoryGenerator
import StepGenerator
from numpy.linalg import inv
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

HOST1 = "192.168.0.99" # The remote host
HOST2 = "192.168.0.98" 
PORT = 30003 # The same port as used by the server

def getData(comm):
  #s.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
  data = comm.recv(1060)
  return data
# getData abre uma conexao com o robo e recebe o pacote de dados de 1060 Bytes

def sendString(comm, string, move_time = 8, pose = False):
  string = "[" + str(string[0]) + "," +  str(string[1]) + "," + str(string[2]) + "," + str(string[3]) + "," + str(string[4]) + "," + str(string[5]) + "]"
  if (pose == True):
  	p = "p"
  else:
  	p = ""
  str_data = "movej(" + p + string + ", t = " + str(move_time) + ")" + "\n"
  comm.send(str_data.encode('ascii'))
  return

def speedJ(comm, string, a = 4*np.pi):
  string = "[" + str(string[0]) + "," +  str(string[1]) + "," + str(string[2]) + "," + str(string[3]) + "," + str(string[4]) + "," + str(string[5]) + "]"
  str_data = "speedj(" + string + ", a = " + str(a) + ",t=0.1)" + "\n"
  #s.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
  comm.send(str_data.encode('ascii'))
  return

def connect(HOST,PORT):
  s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  s.connect((HOST, PORT))
  s.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
  return s

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

  ur5_1 = UR5Class.UR5Class(delta_standard_DH)
  ur5_2 = UR5Class.UR5Class(delta_standard_DH_2)

  s1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  s1.connect((HOST1, PORT))
  s1.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)

  s2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  s2.connect((HOST2, PORT))
  s2.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)


  # process = threading.Thread(target=speedJ,args=[HOST,PORT])
  # process.start()

  time.sleep(0.3)

  ur5_1.setRobotData(getData(s1))
  ur5_2.setRobotData(getData(s2))

  s1.close()
  s2.close()
  initialPose1 = ur5_1.getPosition()
  initialPose2 = ur5_2.getPosition()
  #initialPose[3:6] = tf.rotationVector2RollPitchYaw(initialPose[3:6])

  print("InitialPose UR5-1: ", initialPose1)
  print("InitialPose UR5-1 calculated: ", ur5_1.ur5_direct_kinematics(ur5_1.getJointPosition(), vector = True, apply_offset = True))
  
  initialPose1[3:6] = tf.rotationVector2RollPitchYaw(initialPose1[3:6])
  initialPose1RPY = initialPose1.copy()

  print("Initial Pose UR5-1 RPY", initialPose1RPY)
  print("Initial Pose UR5-1 RPY calculated", ur5_1.ur5_direct_kinematics(ur5_1.getJointPosition(), vector = True, apply_offset = True,rpy=True))



  print("InitialPose UR5-2: ", initialPose2)
  print("InitialPose UR5-2 calculated: ", ur5_2.ur5_direct_kinematics(ur5_2.getJointPosition(), vector = True, apply_offset = True))
  
  initialPose2[3:6] = tf.rotationVector2RollPitchYaw(initialPose2[3:6])
  initialPose2RPY = initialPose2.copy()

  print("Initial Pose UR5-2 RPY", initialPose2RPY)
  print("Initial Pose UR5-2 RPY calculated", ur5_2.ur5_direct_kinematics(ur5_2.getJointPosition(), vector = True, apply_offset = True,rpy=True))



  freq = 0.008

  trajectoryUR51 = trajectoryGenerator.trajectoryGenerator(initialPose1, step = freq, speed = 0.05, direction = "left")
  trajectoryUR51.generateTrajectory()
  trajectoryUR51.saveDatabase()

  trajectoryUR52 = trajectoryGenerator.trajectoryGenerator(initialPose2, step = freq, speed = 0.05, direction = "right")
  trajectoryUR52.generateTrajectory()
  trajectoryUR52.saveDatabase()


  trajectorySpeedList_1 = []
  trajectorySpeed_1 = []
  trajectorySpeedList_2 = []
  trajectorySpeed_2 = []

  for i in range(0,np.shape(trajectoryUR51.database)[0]-1):
    if (i == 0):
      trajectorySpeedList_1.append(np.array([0,0,0,0,0,0]))
    else:
      trajectorySpeedList_1.append((trajectoryUR51.database[i,:] - trajectoryUR51.database[i-1,:])/0.008)


  for i in range(0,np.shape(trajectoryUR52.database)[0]-1):
    if (i == 0):
      trajectorySpeedList_2.append(np.array([0,0,0,0,0,0]))
    else:
      trajectorySpeedList_2.append((trajectoryUR52.database[i,:] - trajectoryUR52.database[i-1,:])/0.008)


  trajectorySpeed_1 = np.asarray(trajectorySpeedList_1, dtype = np.float64)
  trajectorySpeed_2 = np.asarray(trajectorySpeedList_2, dtype = np.float64)

  print('Trajectory 1 number of steps is: ' + str(trajectoryUR51.maxStepNumber -1))
  print('Trajectory 2 number of steps is: ' + str(trajectoryUR52.maxStepNumber -1))
  
  countMax = trajectoryUR51.maxStepNumber -1

  input('Aperte Enter pro play')

  s1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  s1.connect((HOST1, PORT))
  s1.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)

  s2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  s2.connect((HOST2, PORT))
  s2.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)


  count = 0
  countAux = 0

  timeList = []
  timeListPerStep = []


  #Pose
  poseListUR5_1 = []
  poseSpeedListUR5_1 = []
  #Junta
  jointList_1 = []
  speedList_1 = []
  speedCalculatedListUR5_1 = []
  #Força
  TCPForceList_UR5_1 = []

  #Pose
  poseListUR5_2 = []
  poseSpeedListUR5_2 = []
  #Junta
  jointList_2 = []
  speedList_2 = []
  speedCalculatedListUR5_2 = []
  #Força
  TCPForceList_UR5_2 = []

  lostTime = 0

  t1 = time.perf_counter()
  timeInit = t1
  timeListRobot_1 = []
  timeListRobot_2 = []
  ur5_1.setRobotData(getData(s1))
  timeInitRobot_1 = ur5_1.getTime()
  ur5_2.setRobotData(getData(s2))
  timeInitRobot_2 = ur5_2.getTime()

  s1.close()
  s2.close()

  jointControl_1 = np.zeros(6)
  jointControl_2 = np.zeros(6)

  auxiliar = False

  # s1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  # s1.connect((HOST1, PORT))
  # s1.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)

  # s2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  # s2.connect((HOST1, PORT))
  # s2.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)


  #parameters
  parser = argparse.ArgumentParser()
  parser.add_argument('--host', default='192.168.0.99', help='name of host to connect to (localhost)')
  parser.add_argument('--port', type=int, default=30004, help='port number (30004)')
  parser.add_argument('--samples', type=int, default=0, help='number of samples to record')
  parser.add_argument('--frequency', type=int, default=125, help='the sampling frequency in Herz')
  parser.add_argument('--config', default='record_configuration.xml', help='data configuration file to use (record_configuration.xml)')
  parser.add_argument('--output', default='robot_data.csv', help='data output file to write to (robot_data.csv)')
  parser.add_argument("--verbose", help="increase output verbosity", action="store_true")
  args = parser.parse_args()

  if args.verbose:
      logging.basicConfig(level=logging.INFO)

  conf = rtde_config.ConfigFile(args.config)
  output_names, output_types = conf.get_recipe('out')
  setp_names, setp_types = conf.get_recipe('setp')
  watchdog_names, watchdog_types = conf.get_recipe('watchdog')

  
  con1 = rtde.RTDE(args.host, args.port)
  con2 = rtde.RTDE('192.168.0.98', 30004)
  con1.connect()
  con2.connect()

  # get controller version
  con1.get_controller_version()
  con2.get_controller_version()

  con1.send_output_setup(output_names, output_types)
  setp1 = con1.send_input_setup(setp_names, setp_types)
  watchdog1 = con1.send_input_setup(watchdog_names, watchdog_types)

  con2.send_output_setup(output_names, output_types)
  setp2 = con2.send_input_setup(setp_names, setp_types)
  watchdog2 = con2.send_input_setup(watchdog_names, watchdog_types)

  print(setp_names)


  # setup recipes
  # if not con.send_output_setup(output_names, output_types, frequency = args.frequency):
  #     logging.error('Unable to configure output')
  #     sys.exit()

  #start data synchronization
  # if not con.send_start():
  #     logging.error('Unable to start synchronization')
  #     sys.exit()




  # fig = plot.figure(1)
  # ax = fig.gca(projection='3d')
  # ax.plot(trajectoryUR51.database[:,0],trajectoryUR51.database[:,1],trajectoryUR51.database[:,2])


  # fig = plot.figure(2)
  # ax = fig.gca(projection='3d')
  # ax.plot(trajectoryUR52.database[:,0],trajectoryUR52.database[:,1],trajectoryUR52.database[:,2])

  # plot.show()

  # exit()

  setp1.input_double_register_0 = 0
  setp1.input_double_register_1 = 0
  setp1.input_double_register_2 = 0
  setp1.input_double_register_3 = 0
  setp1.input_double_register_4 = 0
  setp1.input_double_register_5 = 0
  setp1.input_double_register_6 = 0

  watchdog1.input_int_register_0 = 0

  setp2.input_double_register_0 = 0
  setp2.input_double_register_1 = 0
  setp2.input_double_register_2 = 0
  setp2.input_double_register_3 = 0
  setp2.input_double_register_4 = 0
  setp2.input_double_register_5 = 0
  setp2.input_double_register_6 = 0

  watchdog2.input_int_register_0 = 0

  # con2.disconnect()
  # con2.connect()
  # con2.get_controller_version()
  # con2.send_output_setup(output_names, output_types)
  # setp2 = con2.send_input_setup(setp_names, setp_types)
  # setp2.input_double_register_0 = 0
  # setp2.input_double_register_1 = 0
  # setp2.input_double_register_2 = 0
  # setp2.input_double_register_3 = 0
  # setp2.input_double_register_4 = 0
  # setp2.input_double_register_5 = 0
  # setp2.input_double_register_6 = 0

  print(ur5_1.getPosition())
  print(ur5_1.getJointPosition())
  print(trajectoryUR51.database[0,:])
  print(trajectorySpeed_1[0,:])

  if not con1.send_output_setup(output_names, output_types, frequency = args.frequency):
    logging.error('Unable to configure output')
    sys.exit()

  #start data synchronization
  if not con1.send_start():
      logging.error('Unable to start synchronization')
      sys.exit()

  if not con2.send_output_setup(output_names, output_types, frequency = args.frequency):
    logging.error('Unable to configure output')
    sys.exit()

  #start data synchronization
  if not con2.send_start():
      logging.error('Unable to start synchronization')
      sys.exit()

  timeInit = time.perf_counter()

  con1.send(setp1)
  con2.send(setp2)
  input('RTDE configurado, pressione enter')

  # s1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  # s1.connect((HOST1, PORT))
  # s1.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)

  auxiliar = True

  while(count < countMax):
    
    # dataUR51 = getData(s1)
    # dataUR52 = getData(s2)

    t1 = time.perf_counter()
    state1 = con1.receive()
    state2 = con2.receive()
    ur5_1.setRobotDataRTDE(state1)
    ur5_2.setRobotDataRTDE(state2)
    # ur5_1.setRobotData(dataUR51)
    # ur5_2.setRobotData(dataUR52)
    #tcp_pose = np.asarray(state.target_TCP_pose)

    # print(tcp_pose)
    # print("\n")

    #print(ur5_1.getPosition())



    #Choose your Control

    #jointControl = ur5.controlLoopInverse(trajectory.database[count,:])
    #jointControl = ur5.controlLoopPseudoInverse(trajectory.database[count,:])
    jointControl_1 = ur5_1.controlLoopDLS(trajectoryUR51.database[count,:], cartesianSpeedReference = trajectorySpeed_1[count,:])
    jointControl_2 = ur5_2.controlLoopDLS(trajectoryUR52.database[count,:], cartesianSpeedReference = trajectorySpeed_2[count,:])
    # jointControl = ur5.controlLoopDLS(target)
    #jointControl = ur5.speedTransform(desiredSpeed)

    #jointControl = np.array([0, 0, 0, 0, 0, 0.1])

    if (auxiliar == True):


      # timeListRobot_1.append(state.timestamp)
      # speedList_1.append(state.target_qd)
      # poseListUR5_1.append(state.target_TCP_pose)
      # speedCalculatedListUR5_1.append(jointControl_1)
      #poseListUR5_2.append(poseUR5_2)


      timeListRobot_1.append(ur5_1.getTime()-timeInitRobot_1)
      timeListRobot_2.append(ur5_2.getTime()-timeInitRobot_2)
      jointList_1.append(ur5_1.getJointPosition())
      jointList_2.append(ur5_2.getJointPosition())
      speedList_1.append(ur5_1.getJointSpeedTarget())
      speedList_2.append(ur5_2.getJointSpeedTarget())
      #jointPositionList.append(ur5.getJointPositionTarget())
      poseUR5_1 = ur5_1.getPosition()
      poseUR5_2 = ur5_2.getPosition()
      poseUR5_1[3:6] = tf.rotationVector2RollPitchYaw(poseUR5_1[3:6])
      poseUR5_2[3:6] = tf.rotationVector2RollPitchYaw(poseUR5_2[3:6])
      poseListUR5_1.append(poseUR5_1)
      poseListUR5_2.append(poseUR5_2)
      poseSpeedListUR5_1.append(ur5_1.getTCPSpeedTarget())
      poseSpeedListUR5_2.append(ur5_2.getTCPSpeedTarget())
      speedCalculatedListUR5_1.append(jointControl_1)
      speedCalculatedListUR5_2.append(jointControl_2)

      TCPForceList_UR5_1.append(ur5_1.getTCPForce())
      TCPForceList_UR5_2.append(ur5_2.getTCPForce())
      
    else:
      if (norm(ur5_1.getJointSpeedTarget()) > 0):
        auxiliar = True
    count = count + 1


    setp1.__dict__["input_double_register_0"] = jointControl_1[0]
    setp1.__dict__["input_double_register_1"] = jointControl_1[1]
    setp1.__dict__["input_double_register_2"] = jointControl_1[2]
    setp1.__dict__["input_double_register_3"] = jointControl_1[3]
    setp1.__dict__["input_double_register_4"] = jointControl_1[4]
    setp1.__dict__["input_double_register_5"] = jointControl_1[5]
    setp1.__dict__["input_double_register_6"] = 1
    con1.send(setp1)
    con1.send(watchdog1)

    setp2.__dict__["input_double_register_0"] = jointControl_2[0]
    setp2.__dict__["input_double_register_1"] = jointControl_2[1]
    setp2.__dict__["input_double_register_2"] = jointControl_2[2]
    setp2.__dict__["input_double_register_3"] = jointControl_2[3]
    setp2.__dict__["input_double_register_4"] = jointControl_2[4]
    setp2.__dict__["input_double_register_5"] = jointControl_2[5]
    setp2.__dict__["input_double_register_6"] = 1
    con2.send(setp2)
    con2.send(watchdog2)
    #speedJ(s1, jointControl_1)
    #speedJ(s2, jointControl_2)
    
    t2 = time.perf_counter()
    timeListPerStep.append(t2 - t1) 
    timeList.append(t2-timeInit)
    #print(t2 - t1)

  setp1.__dict__["input_double_register_0"] = 0
  setp1.__dict__["input_double_register_1"] = 0
  setp1.__dict__["input_double_register_2"] = 0
  setp1.__dict__["input_double_register_3"] = 0
  setp1.__dict__["input_double_register_4"] = 0
  setp1.__dict__["input_double_register_5"] = 0
  setp1.__dict__["input_double_register_6"] = 0
  con1.send(setp1)
  con1.send(watchdog1)

  setp2.__dict__["input_double_register_0"] = 0
  setp2.__dict__["input_double_register_1"] = 0
  setp2.__dict__["input_double_register_2"] = 0
  setp2.__dict__["input_double_register_3"] = 0
  setp2.__dict__["input_double_register_4"] = 0
  setp2.__dict__["input_double_register_5"] = 0
  setp2.__dict__["input_double_register_6"] = 0
  con2.send(setp2)
  con2.send(watchdog2)

  print("FOI: " + str(count))

  # print(timeListRobot_1)
  # print("\n\n\n\n\n\n")
  # print(speedList_1)
  # print("\n\n\n\n\n\n")
  # print(poseListUR5_1)
  # exit()
  # s1.close()
  # s2.close()

  errorArray_1 = np.asarray(ur5_1.errorDB, dtype=np.float32)
  errorArray_2 = np.asarray(ur5_2.errorDB, dtype=np.float32)
  
  #np.save("/home/nascimento/Projects/MESTRADO - Task Space Control/generatedPaths/UR5/Error", errorArray)

  timeArray_1 = np.asarray(timeListRobot_1, dtype=np.float32)
  timeArray_2 = np.asarray(timeListRobot_2, dtype=np.float32)

  normErrorPosition_1 = []
  normErrorRotation_1 = []
  normErrorPosition_2 = []
  normErrorRotation_2 = []

  speedListArray_1 = np.asarray(speedList_1, dtype=np.float32)
  speedListArray_2 = np.asarray(speedList_2, dtype=np.float32)
  speedCalculatedListArrayUR5_1 = np.asarray(speedCalculatedListUR5_1, dtype=np.float32)
  speedCalculatedListArrayUR5_2 = np.asarray(speedCalculatedListUR5_2, dtype=np.float32)
  poseListArrayUR5_1 = np.asarray(poseListUR5_1, dtype = np.float32)
  poseListArrayUR5_2 = np.asarray(poseListUR5_2, dtype = np.float32)

  TCPForceArrayUR5_1 = np.asarray(TCPForceList_UR5_1, dtype = np.float32)
  TCPForceArrayUR5_2 = np.asarray(TCPForceList_UR5_2, dtype = np.float32)

  poseSpeedArrayUR5_1 = np.asarray(poseSpeedListUR5_1, dtype = np.float32)
  poseSpeedArrayUR5_2 = np.asarray(poseSpeedListUR5_2, dtype = np.float32)

  jointArray_1 = np.asarray(jointList_1, dtype = np.float32)
  jointArray_2 = np.asarray(jointList_2, dtype = np.float32)

  #jointPositionListArray = np.asarray(jointPositionList, dtype=np.float32)


  for i in range(np.shape(errorArray_1)[0] - np.shape(timeArray_1)[0],np.shape(errorArray_1)[0]):
    normErrorPosition_1.append(norm(errorArray_1[i,0:3]))
    normErrorRotation_1.append(norm(errorArray_1[i,3:6]))

  for i in range(np.shape(errorArray_2)[0] - np.shape(timeArray_2)[0],np.shape(errorArray_2)[0]):
    normErrorPosition_2.append(norm(errorArray_2[i,0:3]))
    normErrorRotation_2.append(norm(errorArray_2[i,3:6]))

  # for i in range(0,np.shape(speedListArray)[0]):
  #   print (speedListArray[i,:])

  normErrorPositionArray_1 = np.asarray(normErrorPosition_1, dtype = np.float32)
  normErrorRotationArray_1 = np.asarray(normErrorRotation_1, dtype = np.float32)
  normErrorPositionArray_2 = np.asarray(normErrorPosition_2, dtype = np.float32)
  normErrorRotationArray_2 = np.asarray(normErrorRotation_2, dtype = np.float32)

  wDBarray_1 = np.asarray(ur5_1.wDB, dtype = np.float32)
  wDBarray_2 = np.asarray(ur5_2.wDB, dtype = np.float32)

  print("Lost steps are: " + str(lostTime))

  mdict = {'normErrorPosition_UR5_1': normErrorPosition_1, 'normErrorPosition_UR5_2': normErrorPosition_2, 
  'normErrorRotation_UR5_1':normErrorRotation_1,'normErrorRotation_UR5_2':normErrorRotation_2, 'expTime_1':timeArray_1,'expTime_2':timeArray_2,'speedListJointArray_UR5_1':speedListArray_1,
  'speedListJointArray_UR5_2':speedListArray_2, 'speedCalculatedListJointArray_UR5_1':speedCalculatedListArrayUR5_1,'speedCalculatedListJointArray_UR5_2':speedCalculatedListArrayUR5_2,
  'wDBarray_UR5_1':wDBarray_1, 'wDBarray_UR5_2':wDBarray_2, 'PoseArray_UR5_1':poseListArrayUR5_1, 'PoseArray_UR5_2':poseListArrayUR5_2, 'TCPForceArray_1':TCPForceArrayUR5_1, 'TCPForceArray_2':TCPForceArrayUR5_2,
  'poseSpeedArray_1':poseSpeedArrayUR5_1, 'poseSpeedArray_2':poseSpeedArrayUR5_2, 'jointArray_1':jointArray_1, 'jointArray_2':jointArray_2,'TrajectoryUR51Database':trajectoryUR51.database,'TrajectoryUR52Database':trajectoryUR52.database}

  savemat('DualBox_10',mdict)

  # for i in range(0,len(timeListPerStep)):
  #   print(timeListPerStep[i])

  plot.figure(1)
  plot.plot(timeArray_1,normErrorPositionArray_1)
  plot.xlabel("Tempo(s)")
  plot.ylabel("Erro(m)")
  plot.suptitle("UR5 1 - Erro absoluto - Posicao")

  plot.figure(2)
  plot.plot(timeArray_2,normErrorPositionArray_2)
  plot.xlabel("Tempo(s)")
  plot.ylabel("Erro(m)")
  plot.suptitle("UR5 2 - Erro absoluto - Posicao")

  # plot.figure(2)
  # plot.plot(timeArray,speedListArray[:,0], color = "red", label="Velocidade Real")
  # plot.plot(timeArray,speedCalculatedListArray[:,0], color = "blue", label="Velocidade Calculada")
  # plot.xlabel("Tempo(s)")
  # plot.ylabel("Velocidade(rad/s)")
  # plot.suptitle("Velocidade - Junta 1")
  # plot.legend(loc='upper left')

  # plot.figure(3)
  # plot.plot(timeArray,speedListArray[:,1], color = "red", label="Velocidade Real")
  # plot.plot(timeArray,speedCalculatedListArray[:,1], color = "blue", label="Velocidade Calculada")
  # plot.xlabel("Tempo(s)")
  # plot.ylabel("Velocidade(rad/s)")
  # plot.suptitle("Velocidade - Junta 2")
  # plot.legend(loc='upper left')

  # plot.figure(4)
  # plot.plot(timeArray,speedListArray[:,2], color = "red", label="Velocidade Real")
  # plot.plot(timeArray,speedCalculatedListArray[:,2], color = "blue", label="Velocidade Calculada")
  # plot.xlabel("Tempo(s)")
  # plot.ylabel("Velocidade(rad/s)")
  # plot.suptitle("Velocidade - Junta 3")
  # plot.legend(loc='upper left')

  # plot.figure(5)
  # plot.plot(timeArray,speedListArray[:,3], color = "red", label="Velocidade Real")
  # plot.plot(timeArray,speedCalculatedListArray[:,3], color = "blue", label="Velocidade Calculada")
  # plot.xlabel("Tempo(s)")
  # plot.ylabel("Velocidade(rad/s)")
  # plot.suptitle("Velocidade - Junta 4")
  # plot.legend(loc='upper left')

  # plot.figure(6)
  # plot.plot(timeArray,speedListArray[:,4], color = "red", label="Velocidade Real")
  # plot.plot(timeArray,speedCalculatedListArray[:,4], color = "blue", label="Velocidade Calculada")
  # plot.xlabel("Tempo(s)")
  # plot.ylabel("Velocidade(rad/s)")
  # plot.suptitle("Velocidade - Junta 5")
  # plot.legend(loc='upper left')

  # plot.figure(7)
  # plot.plot(timeArray,speedListArray[:,5], color = "red", label="Velocidade Real")
  # plot.plot(timeArray,speedCalculatedListArray[:,5], color = "blue", label="Velocidade Calculada")
  # plot.xlabel("Tempo(s)")
  # plot.ylabel("Velocidade(rad/s)")
  # plot.suptitle("Velocidade - Junta 6")
  # plot.legend(loc='upper left')

  plot.figure(3)
  plot.plot(timeArray_1,normErrorRotationArray_1)
  plot.xlabel("Tempo(s)")
  plot.ylabel("Erro(rad)")
  plot.suptitle("UR5 1 - Erro absoluto - Rotacao")

  plot.figure(4)
  plot.plot(timeArray_2,normErrorRotationArray_2)
  plot.xlabel("Tempo(s)")
  plot.ylabel("Erro(rad)")
  plot.suptitle("UR5 2 - Erro absoluto - Rotacao")

  # plot.figure(9)
  # plot.plot(timeArray,wDBarray)
  # plot.xlabel("Tempo(s)")
  # plot.ylabel("Distância de Singularidade")
  # plot.suptitle("Análise de Singularidade")
  plot.show()

  # fig = plot.figure(5)
  # ax = fig.gca(projection='3d')
  # ax.plot(poseListArrayUR5_1[:,0],poseListArrayUR5_1[:,1],poseListArrayUR5_1[:,2])
  # plot.xlabel("X")
  # plot.ylabel("Y")
  # plot.zlabel("Z")
  # plot.suptitle("Trajetória - UR5 1")

  fig = plot.figure(5)
  ax = fig.gca(projection='3d')
  ax.plot(poseListArrayUR5_1[:,0],poseListArrayUR5_1[:,1], poseListArrayUR5_1[:,2],color = "red", label="Trajetória Real - UR5-1")
  ax.plot(trajectoryUR51.database[:,0],trajectoryUR51.database[:,1],trajectoryUR51.database[:,2],color = "blue", label="Trajetória Calculada - UR5-1")
  ax.set_xlabel("X")
  ax.set_ylabel("Y")
  ax.set_zlabel("Z")


  fig2 = plot.figure(6)
  ax2 = fig2.gca(projection='3d')
  ax2.plot(poseListArrayUR5_2[:,0],poseListArrayUR5_2[:,1], poseListArrayUR5_2[:,2],color = "red", label="Trajetória Real - UR5-2")
  ax2.plot(trajectoryUR52.database[:,0],trajectoryUR52.database[:,1],trajectoryUR52.database[:,2],color = "blue", label="Trajetória Calculada - UR5-2")
  ax2.set_xlabel("X")
  ax2.set_ylabel("Y")
  ax2.set_zlabel("Z")
  ax.legend()
  #plot.suptitle("Trajetória - UR5 2")

  plot.show()


main(sys.argv)