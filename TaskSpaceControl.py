#!/usr/bin/python
# -*- coding: utf-8 -*-
# Get Pose
import numpy as np
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

HOST = "192.168.0.98" # The remote host 
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
  str_data = "speedj(" + string + ", a = " + str(a) + ",t=0.08)" + "\n"
  #s.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
  comm.send(str_data.encode('ascii'))
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

  s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  s.connect((HOST, PORT))
  s.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)

  # process = threading.Thread(target=speedJ,args=[HOST,PORT])
  # process.start()

  time.sleep(0.3)

  ur5.setRobotData(getData(s))

  s.close()
  initialPose = ur5.getPosition()
  #initialPose[3:6] = tf.rotationVector2RollPitchYaw(initialPose[3:6])

  print("InitialPose: ", initialPose)
  print("InitialPose calculated: ", ur5.ur5_direct_kinematics(ur5.getJointPosition(), vector = True, apply_offset = True))
  print(threading.enumerate())

  initialPose[3:6] = tf.rotationVector2RollPitchYaw(initialPose[3:6])
  initialPoseRPY = initialPose.copy()

  print("Initial Pose RPY", initialPoseRPY)
  print("Initial Pose RPY calculated", ur5.ur5_direct_kinematics(ur5.getJointPosition(), vector = True, apply_offset = True,rpy=True))

  freq = 0.008

  trajectory = trajectoryGenerator.trajectoryGenerator(initialPose, step = freq, speed = 0.05, direction = "right")
  trajectory.generateTrajectory()
  trajectory.saveDatabase()

  trajectoryJointList = []
  trajectoryJointSpeedReferenceList = []
  trajectorySpeedList = []
  trajectorySpeed = []

  for i in range(0,np.shape(trajectory.database)[0]-1):
  #   print('Calculating Inverse of Step: ' + str(i))
  #   trajectoryJointList.append(ur5.ur5_inverse_kinematics_newthon_raphson(trajectory.database[i,:].copy(), chosen_theta = 2, rpy = True))
    if (i == 0):
  #     trajectoryJointSpeedReferenceList.append(np.array([0,0,0,0,0,0]))
      trajectorySpeedList.append(np.array([0,0,0,0,0,0]))
    else:
  #     speedReference = (trajectoryJointList[i] - trajectoryJointList[i - 1])/0.008
  #     trajectoryJointSpeedReferenceList.append(speedReference)
      trajectorySpeedList.append((trajectory.database[i,:] - trajectory.database[i-1,:])/0.008)

  # trajectoryJointSpeedReferenceArray = np.asarray(trajectoryJointSpeedReferenceList, dtype = np.float64)
  trajectorySpeed = np.asarray(trajectorySpeedList, dtype = np.float64)

  # trajectoryJointReference = np.asarray(trajectoryJointList, dtype = np.float64)
  # for i in range(0,np.shape(trajectory.database)[0]-1):
  #   print(trajectoryJointSpeedReferenceArray[i])

  countMax = trajectory.maxStepNumber -1
  #countMax = 10/0.008
  print(countMax)

  # print(trajectoryJointReference.shape[0])

  fig = plot.figure(22)
  ax = fig.gca(projection='3d')
  ax.plot(trajectory.database[:,0],trajectory.database[:,1],trajectory.database[:,2])

  plot.show()

  input('Aperte Enter pro play')

  s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  s.connect((HOST, PORT))
  s.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)

  count = 0
  countAux = 0

  timeList = []
  timeListPerStep = []
  jointPositionList = []
  poseList = []
  speedList = []
  speedCalculatedList = []

  lostTime = 0


  # jointControl = ur5.controlLoopPseudoInverse(initialPose + np.array([0,0,0.1,0,0,0]), initialPose)
  # jointControlConst = jointControl
  # speedJ(HOST, PORT, jointControl)


  t1 = time.perf_counter()
  timeInit = t1
  timeListRobot = []
  ur5.setRobotData(getData(s))
  timeInitRobot = ur5.getTime()

  jointControl = np.zeros(6)

  if (str(args[1]) == 'joint'):
    pose = ur5.getJointPosition()
  elif(str(args[1]) == 'cartesian'):
    pose = initialPose

  rot = tf.pose2Matrix(pose, rotOnly = True)
  rot6 = np.eye(6)
  rot6[0:3,0:3] = inv(rot)
  pose = np.dot(rot6,pose)

  target = StepGenerator.stepGenerator(ur5,str(args[1]),pose.copy(),int(args[2]),float(args[3]))
  inverse = ur5.ur5_inverse_kinematics_newthon_raphson(target.copy(), chosen_theta = 2, theta = np.zeros(6), rpy = True)
  print(inverse*180/np.pi)


  jacob1 = ur5.jacobian(ur5.getJointPosition()[np.newaxis].transpose(),(np.ones(6)*10e-6)[np.newaxis].transpose(), rpy = True)
  jacob2 = ur5.jacobian2(ur5.getJointPosition())

  print('First Jacobian is: ')
  print(jacob1)
  print('\n\n\n')
  print('Second Jacobian is: ')
  print(jacob2)
  print('\n\n\n')
  
  print('Initial Pose RPY is: ' + str(initialPoseRPY))
 
  print('Third Jacob is: ')

  jacob3 = ur5.jacobianAnalytic(ur5.getJointPosition())
  print(jacob3)

  desiredSpeed = np.array([0.01,0,0,0,0,0.024])
  velocity = ur5.speedTransform(desiredSpeed)

  print('desired speed: ' + str(desiredSpeed))
  print('joint velocity: ' + str(velocity))

  auxiliar = False

  timeInit = time.perf_counter()

  # print("\n\n\n")
  # print('target is: ' + str(target))
  # rot = tf.pose2Matrix(initialPoseRPY, rotOnly = True)
  # rot6 = np.eye(6)
  # rot6[0:3,0:3] = rot
  # #rot6[3:6,3:6] = rot6[0:3,0:3]
  # print('rot6 is: ')
  # print(rot6)
  # newTarget = np.dot(rot6,target)
  # print('new target is: ' + str(newTarget))

  # fig = plot.figure(22)
  # ax = fig.gca(projection='3d')
  # ax.plot(trajectory.database[:,0],trajectory.database[:,1],trajectory.database[:,2])

  # plot.show()

  # s.close()
  # exit()
  s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  s.connect((HOST, PORT))
  s.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
  
  while(count < countMax):
    
    data = getData(s)

    t1 = time.perf_counter()
    ur5.setRobotData(data)


    #Choose your Control

    #jointControl = ur5.controlLoopInverse(trajectory.database[count,:])
    #jointControl = ur5.controlLoopPseudoInverse(trajectory.database[count,:])
    jointControl = ur5.controlLoopDLS(trajectory.database[count,:], cartesianSpeedReference = trajectorySpeed[count,:])
    # jointControl = ur5.controlLoopDLS(target)
    #jointControl = ur5.speedTransform(desiredSpeed)

    #jointControl = np.array([0, 0, 0, 0, 0, 0.1])

    if (auxiliar == True):
      timeListRobot.append(ur5.getTime()-timeInitRobot)
      speedList.append(ur5.getJointSpeedTarget())
      jointPositionList.append(ur5.getJointPositionTarget())
      pose = ur5.getPosition()
      pose[3:6] = tf.rotationVector2RollPitchYaw(pose[3:6])
      poseList.append(pose)
      speedCalculatedList.append(jointControl)
      count = count + 1
    else:
      if (norm(ur5.getJointSpeedTarget()) > 0):
        auxiliar = True

    if np.any(np.isnan(jointControl)):
      print('[INFO] NaN value found on control.')
      pass
    else:
      #pass
      #sendFloats(jointControl)
      speedJ(s, jointControl)
    
    t2 = time.perf_counter()
    timeListPerStep.append(t2 - t1) 
    timeList.append(t2-timeInit)

  print("\n\n\n\n\n\nJACOBIAN PROCESS TIME IS: \n\n" )
  print(ur5.processTimeList)
  print("\n\n\nMean is: \n")
  print(np.mean(np.asarray(ur5.processTimeList,dtype=np.float64)))

  #exit(0)

  dead = True

  errorArray = np.asarray(ur5.errorDB, dtype=np.float32)
  #np.save("/home/nascimento/Projects/MESTRADO - Task Space Control/generatedPaths/UR5/Error", errorArray)

  timeArray = np.asarray(timeListRobot, dtype=np.float32)

  normErrorPosition = []
  normErrorRotation = []

  speedListArray = np.asarray(speedList, dtype=np.float32)
  speedCalculatedListArray = np.asarray(speedCalculatedList, dtype=np.float32)
  poseListArray = np.asarray(poseList, dtype = np.float32)

  jointPositionListArray = np.asarray(jointPositionList, dtype=np.float32)


  for i in range(np.shape(errorArray)[0] - np.shape(timeArray)[0],np.shape(errorArray)[0]):
    normErrorPosition.append(norm(errorArray[i,0:3]))
    normErrorRotation.append(norm(errorArray[i,3:6]))

  # for i in range(0,np.shape(speedListArray)[0]):
  #   print (speedListArray[i,:])

  normErrorPositionArray = np.asarray(normErrorPosition, dtype = np.float32)
  normErrorRotationArray = np.asarray(normErrorRotation, dtype = np.float32)
  wDBarray = np.asarray(ur5.wDB, dtype = np.float32)

  print("Lost steps are: " + str(lostTime))

  mdict = {'normErrorPosition_EXP': normErrorPosition, 
  'normErrorRotation_EXP':normErrorRotation, 'expTime':timeArray,'speedListJointArray_EXP':speedListArray,
  'speedCalculatedListJointArray_EXP':speedCalculatedListArray,'wDBarray_EXP':wDBarray, 'jointPositionListArray':jointPositionListArray, 'PoseArray':poseListArray}

  #savemat('StepPINV',mdict)

  targetTimeArray = np.zeros([timeArray.shape[0],6])
  inverseTimeArray = np.zeros([timeArray.shape[0],6])

  for i in range(0,targetTimeArray.shape[0]):
    targetTimeArray[i,:] = target
    inverseTimeArray[i,:] = inverse

  # for i in range(0,len(timeListPerStep)):
  #   print(timeListPerStep[i])

  plot.figure(1)
  plot.plot(timeArray,normErrorPositionArray)
  plot.xlabel("Tempo(s)")
  plot.ylabel("Erro(m)")
  plot.suptitle("Erro absoluto - Posicao")

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

  plot.figure(8)
  plot.plot(timeArray,normErrorRotationArray)
  plot.xlabel("Tempo(s)")
  plot.ylabel("Erro(rad)")
  plot.suptitle("Erro absoluto - Rotacao")

  # plot.figure(9)
  # plot.plot(timeArray,wDBarray)
  # plot.xlabel("Tempo(s)")
  # plot.ylabel("Distância de Singularidade")
  # plot.suptitle("Análise de Singularidade")

  plot.figure(10)
  plot.plot(timeArray,jointPositionListArray[:,0], color = "blue", label = 'Real')
  # plot.plot(timeArray,trajectoryJointReference[:,0],color = "red", label = 'Alvo')
  plot.xlabel("Tempo(s)")
  plot.ylabel("Radianos")
  plot.suptitle("Posição - Junta 1")
  plot.legend(loc='upper right')

  plot.figure(11)
  plot.plot(timeArray,jointPositionListArray[:,1], color = "blue", label = 'Real')
  # plot.plot(timeArray,trajectoryJointReference[:,1],color = "red", label = 'Alvo')
  plot.xlabel("Tempo(s)")
  plot.ylabel("Radianos")
  plot.suptitle("Posição - Junta 2")
  plot.legend(loc='upper right')

  plot.figure(12)
  plot.plot(timeArray,jointPositionListArray[:,2], color = "blue", label = 'Real')
  # plot.plot(timeArray,trajectoryJointReference[:,2],color = "red", label = 'Alvo')
  plot.xlabel("Tempo(s)")
  plot.ylabel("Radianos")
  plot.suptitle("Posição - Junta 3")
  plot.legend(loc='upper right')

  plot.figure(13)
  plot.plot(timeArray,jointPositionListArray[:,3], color = "blue", label = 'Real')
  # plot.plot(timeArray,trajectoryJointReference[:,3],color = "red", label = 'Alvo')
  plot.xlabel("Tempo(s)")
  plot.ylabel("Radianos")
  plot.suptitle("Posição - Junta 4")
  plot.legend(loc='upper right')

  plot.figure(14)
  plot.plot(timeArray,jointPositionListArray[:,4], color = "blue", label = 'Real')
  # plot.plot(timeArray,trajectoryJointReference[:,4],color = "red", label = 'Alvo')
  plot.xlabel("Tempo(s)")
  plot.ylabel("Radianos")
  plot.suptitle("Posição - Junta 5")
  plot.legend(loc='upper right')

  plot.figure(15)
  plot.plot(timeArray,jointPositionListArray[:,5], color = "blue", label = 'Real')
  # plot.plot(timeArray,trajectoryJointReference[:,5],color = "red", label = 'Alvo')
  plot.xlabel("Tempo(s)")
  plot.ylabel("Radianos")
  plot.suptitle("Posição - Junta 6")
  plot.legend(loc='upper right')

  plot.figure(16)
  plot.plot(timeArray,poseListArray[:,0], color = "blue", label = 'Real')
  plot.plot(timeArray,targetTimeArray[:,0],color = "red", label = 'Alvo')
  plot.xlabel("Tempo(s)")
  plot.ylabel("X")
  plot.suptitle("Pose - X")
  plot.legend(loc='upper left')

  plot.figure(17)
  plot.plot(timeArray,poseListArray[:,1], color = "blue", label = 'Real')
  plot.plot(timeArray,targetTimeArray[:,1],color = "red", label = 'Alvo')
  plot.xlabel("Tempo(s)")
  plot.ylabel("Y")
  plot.suptitle("Pose - Y")
  plot.legend(loc='upper left')

  plot.figure(18)
  plot.plot(timeArray,poseListArray[:,2], color = "blue", label = 'Real')
  plot.plot(timeArray,targetTimeArray[:,2],color = "red", label = 'Alvo')
  plot.xlabel("Tempo(s)")
  plot.ylabel("Z")
  plot.suptitle("Pose - Z")
  plot.legend(loc='upper left')

  plot.figure(19)
  plot.plot(timeArray,poseListArray[:,3], color = "blue", label ='Posição Real (rpy)')
  plot.plot(timeArray,targetTimeArray[:,3],color = "red",label ='Posição Calculada (rpy)')
  plot.xlabel("Tempo(s)")
  plot.ylabel("RX")
  plot.suptitle("Pose - RX")
  plot.legend(loc='upper left')

  plot.figure(20)
  plot.plot(timeArray,poseListArray[:,4], color = "blue", label ='Posição Real (rpy)')
  plot.plot(timeArray,targetTimeArray[:,4],color = "red",label ='Posição Calculada (rpy)')
  plot.xlabel("Tempo(s)")
  plot.ylabel("RY")
  plot.suptitle("Pose - RY")
  plot.legend(loc='upper left')

  plot.figure(21)
  plot.plot(timeArray,poseListArray[:,5], color = "blue", label ='Posição Real (rpy)')
  plot.plot(timeArray,targetTimeArray[:,5],color = "red",label ='Posição Calculada (rpy)')
  plot.xlabel("Tempo(s)")
  plot.ylabel("RZ")
  plot.suptitle("Pose - RZ")
  plot.legend(loc='best')

  fig = plot.figure(22)
  ax = fig.gca(projection='3d')
  ax.plot(trajectory.database[:,0],trajectory.database[:,1],trajectory.database[:,2],color = "blue", label="Trajetória Calculada - UR5")
  ax.plot(poseListArray[:,0],poseListArray[:,1], poseListArray[:,2],color = "red", label="Trajetória Real - UR5-1")
  ax.set_xlabel("X")
  ax.set_ylabel("Y")
  ax.set_zlabel("Z")
  ax.legend()

  plot.show()


main(sys.argv)