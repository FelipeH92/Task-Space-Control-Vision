#!/usr/bin/python
# -*- coding: utf-8 -*-

## Transformations

import numpy as np
from numpy.linalg import inv
from numpy.linalg import norm
from numpy.linalg import pinv
import struct
import time
import csv


def getPoseRobot(worldToPoint, robotToPoint):

    worldToPoint = rotationVector2Matrix(worldToPoint, homogeneous = True)
    robotToPoint = rotationVector2Matrix(robotToPoint, homogeneous = True)

    robotPose = np.dot(worldToPoint,inv(robotToPoint))

    return robotPose


## Realiza uma transformação de um ponto no espaço com relação a uma referência para um novo ponto em nova referência. Utilizado para transformação de dedos. Recebe apenas dados RV.
#  @param self O ponteiro do objeto
#  @param past_ref Matriz de posição 4x4 da referência antiga.
#  @param new_ref Matriz de posição 4x4 da nova referência.
#  @param point_to_transform Matriz de posição 4x4 a ser transformado.
def transformPoseRef(past_ref, new_ref, point_to_transform):
            
            past_ref = rotationVector2Matrix(past_ref, homogeneous = True)
            new_ref = rotationVector2Matrix(new_ref, homogeneous = True)
            point_to_transform = rotationVector2Matrix(point_to_transform, homogeneous = True)

            try:
                transform_past_to_new = np.dot(inv(past_ref),new_ref)
            except:
                print("[ERROR] Could not calculate inverse: inv(x), x = ", past_ref)
                return None
            #print transform_past_to_new

            transformed_point = np.dot(point_to_transform,transform_past_to_new)

            transformed_point = matrix2RotationVector(transformed_point, homogeneous = True)

            return transformed_point


## Realiza uma transformação de um ponto no espaço com relação a uma referência para um novo ponto em nova referência. Utilizado para calibração. Recebe apenas dados em RV.
#  @param self O ponteiro do objeto.
#  @param previous_calib Vetor de posição 1x6 da referência antiga.
#  @param new_calib Vetor de posição 1x6 da nova referência.
#  @param point_to_transform Vetor de posição 1x6 a ser transformado.

def calibrationTransform(previous_calib, new_calib, point_to_transform):

    previous_calib = rotationVector2Matrix(previous_calib, homogeneous=True)
    new_calib = rotationVector2Matrix(new_calib, homogeneous=True)
    point_to_transform = rotationVector2Matrix(point_to_transform, homogeneous=True)

    try:
        transformCalib1ToCalib2 = np.dot(inv(previous_calib),new_calib)
        transformationCalib1ToPoint = np.dot(inv(previous_calib),point_to_transform)
    except:
        print("[ERROR] Could not calculate inverse: inv(x), x = ", previous_calib)
        return None

    transformed_point = np.dot(previous_calib,transformCalib1ToCalib2)
    transformed_point = np.dot(transformed_point,transformationCalib1ToPoint)

    transformed_point = matrix2RotationVector(transformed_point, homogeneous = True)

    return transformed_point


# coordTransform transforma um determinado ponto point_2_transform que estava no espaço referente a previous_reference em um novo ponto no espaço referente a new_reference

## Realiza uma transformação de uma rotação em padrão aeronáutico RPY para uma rotação RV.
#  @param self O ponteiro do objeto.
#  @param rpy Vetor de rotação 1x3.
def rollPitchYaw2RotationVector(rpy):

        alpha = rpy[2]
        beta = rpy[1]
        gamma = rpy[0]

        ca = np.cos(alpha)
        cb = np.cos(beta)
        cg = np.cos(gamma)
        sa = np.sin(alpha)
        sb = np.sin(beta)
        sg = np.sin(gamma)

        r11 = ca*cb
        r12 = ca*sb*sg-sa*cg
        r13 = ca*sb*cg+sa*sg
        r21 = sa*cb
        r22 = sa*sb*sg+ca*cg
        r23 = sa*sb*cg-ca*sg
        r31 = -sb
        r32 = cb*sg
        r33 = cb*cg

        try:
            theta = np.arccos((r11+r22+r33-1)/2)
        except:  
            print("[ERROR] Could not calculate transformation RPY to RV")
            print("[ERROR] Arccos(x), x = ",(r11+r22+r33-1)/2)
            return None
        sth = np.sin(theta)
        kx = (r32-r23)/(2*sth)
        ky = (r13-r31)/(2*sth)
        kz = (r21-r12)/(2*sth)

        rv = np.array([theta*kx, theta*ky, theta*kz])

        return rv
# rollPitchYaw2RotationVector transforma o vetor de padrão aeronautica de Roll, Pitch e Yaw para o vetor de rotação utilizado e recebido pelo UR5

## Realiza uma transformação de uma rotação em RV para uma rotação em padrão aeronáutico RPY.
#  @param self O ponteiro do objeto.
#  @param rv Vetor de rotação 1x3.
def rotationVector2RollPitchYaw(rv):

        rx = rv[0]
        ry = rv[1]
        rz = rv[2]

        theta = np.sqrt(rx*rx + ry*ry + rz*rz)
        kx = rx/theta
        ky = ry/theta
        kz = rz/theta
        cth = np.cos(theta)
        sth = np.sin(theta)
        vth = 1-cth

        r11 = kx*kx*vth + cth
        r12 = kx*ky*vth - kz*sth
        r13 = kx*kz*vth + ky*sth
        r21 = kx*ky*vth + kz*sth
        r22 = ky*ky*vth + cth
        r23 = ky*kz*vth - kx*sth
        r31 = kx*kz*vth - ky*sth
        r32 = ky*kz*vth + kx*sth
        r33 = kz*kz*vth + cth

        beta = np.arctan2(-r31, np.sqrt(r11*r11+r21*r21))

        if beta > np.radians(89.99):
                print('[INFO][rotationVector2RollPitchYaw] Singularity found on transform: ' + str(beta))
                beta = np.radians(89.99)
                alpha = 0
                gamma = np.arctan2(r12,r22)
        elif beta < -np.radians(89.99):
                print('[INFO][rotationVector2RollPitchYaw] Singularity found on transform: ' + str(beta))
                beta = -np.radians(89.99)
                alpha = 0
                gamma = -np.arctan2(r12,r22)
        else:
                cb = np.cos(beta)
                alpha = np.arctan2(r21/cb,r11/cb)
                gamma = np.arctan2(r32/cb,r33/cb)
        
        rpy = np.array([gamma, beta, alpha])  

        return rpy
# rotationVector2RollPitchYaw transforma o vetor de rotação utilizado e recebido pelo UR5 para o padrão aeronautica de Roll, Pitch e Yaw

def Rot_x(theta):
        rot_mat_x = np.mat([[1, 0, 0], [ 0, np.cos(theta), -np.sin(theta)], [ 0, np.sin(theta), np.cos(theta)]])

        return rot_mat_x

def Rot_y(theta):
        rot_mat_y = np.mat([[np.cos(theta), 0, np.sin(theta)],[ 0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]])

        return rot_mat_y

def Rot_z(theta):
        rot_mat_z = np.mat([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])

        return rot_mat_z

## Computa a diferença entre duas poses.
# @param pose_a Primeira pose.
# @param pose_b Segunda pose.
# @param rotOnly Parâmetro opcional computar apenas vetores de rotação.

def computeDifference(pose_a, pose_b, rotOnly = False, positionOnly = False):

    diff = pose_a - pose_b

    if (positionOnly == False):
        if (rotOnly == False):
            for i in range(3,6):
                if (diff[i] > np.pi):
                    diff[i] -= (2*np.pi)
                elif (diff[i] < (-np.pi)):
                    diff[i] += (2*np.pi)
        else:
            for i in range(0,3):
                if (diff[i] > np.pi):
                    diff[i] -= (2*np.pi)
                elif (diff[i] < (-np.pi)):
                    diff[i] += (2*np.pi)
    else:
        diff[3:6] = diff[3:6]
    return diff


## Computa a soma entre duas poses.
# @param pose_a Primeira pose.
# @param pose_b Segunda pose.
# @param rotOnly Parâmetro opcional computar apenas vetores de rotação.
def computeSum(pose_a, pose_b, rotOnly = False, positionOnly = False):

    soma = pose_a + pose_b

    if (positionOnly == False):
        if (rotOnly == False):
            for i in range(3,6):
                if (soma[i] > np.pi):
                    soma[i] -= (2*np.pi)
                elif (soma[i] < (-np.pi)):
                    soma[i] += (2*np.pi)
        else:
            for i in range(0,3):
                if (soma[i] > np.pi):
                    soma[i] -= (2*np.pi)
                elif (soma[i] < (-np.pi)):
                    soma[i] += (2*np.pi)
    else:
        soma[3:6] = pose_a[3:6]
    return soma


## Realiza uma transformação de um vetor de posições de três rotações para uma matriz de rotação 3x3.
#  @param self O ponteiro do objeto.
#  @param pos Vetor de posições em formato [x, y, z, rx, ry, rz].
# def pose2Matrix(pos, rotOnly = False):

#     rot = Rot_y(pos[4])*Rot_z(pos[5])*Rot_x(pos[3])

#     if (rotOnly == True):
#         trans = np.mat([[rot[0,0], rot[0,1], rot[0,2], pos[0]], [rot[1,0], rot[1,1], rot[1,2], pos[1]], [rot[2,0], rot[2,1], rot[2,2], pos[2]], [0, 0, 0, 1]])
#     else:
#         trans = np.eye(4,4)
#         trans[0:3,0:3] = rot
#         trans[0:3,3] = pos[0:3]
#     return trans

def pose2Matrix(pos, rotOnly = False):

    rot = Rot_z(pos[5])*Rot_y(pos[4])*Rot_x(pos[3])

    if (rotOnly == True):
        trans = np.mat([[rot[0,0], rot[0,1], rot[0,2]], [rot[1,0], rot[1,1], rot[1,2]], [rot[2,0], rot[2,1], rot[2,2]]])
    else:
        trans = np.eye(4,4)
        trans[0:3,0:3] = rot
        trans[0:3,3] = pos[0:3]
    return trans

# def matrix2Pose(matrix, rotOnly = False):

#     rz = *np.arcsin(matrix[1,0])

#     if rz > (np.pi/2):
#         rz -= np.pi
#     elif rz < (-np.pi/2):
#         rz += np.pi

#     if (rz == 0) or (rz == np.pi):
#         print("[INFO][matrix2Pose] Singularity found on transform.")
#         rz = 10e-4

#     ry = np.arcsin(-matrix[2,0]/np.cos(rz))

#     if ry > (np.pi/2):
#         ry -= np.pi
#     elif ry < (-np.pi/2):
#         ry += np.pi

#     #rx = np.arcsin(-matrix[1,2]/np.cos(rz))
#     rx = np.arccos(matrix[1,1]/np.cos(rz))

#     if rx > (np.pi):
#         rx -= np.pi
#     elif rx < 0:
#         ry += np.pi

#     if (rotOnly == True):
#         pos = np.array([rx, ry, rz])
#     else:
#         pos = np.array([matrix[3,0],matrix[3,1],matrix[3,2],rx, ry, rz])

#     return pos

def matrix2Pose(matrix, rotOnly = False):

    if ((matrix[0,0]) == 0 and (matrix[1,0] == 0)):
        ry = np.pi/2
        rz = 0
        rx = np.arctan2(matrix[0,1],matrix[1,1])
    else:
        ry = np.arctan2(-matrix[2,0],np.sqrt(np.power(matrix[0,0],2) + np.power(matrix[1,0],2)))
        rz = np.arctan2(matrix[1,0],matrix[0,0])
        rx = np.arctan2(matrix[2,1],matrix[2,2])

    if (rotOnly == True):
        pos = np.array([rx, ry, rz])
    else:
        pos = np.array([matrix[3,0],matrix[3,1],matrix[3,2],rx, ry, rz])

    return pos


## Realiza uma transformação de uma matriz de rotação ZYX para um vetor rpy 1x3.
#  @param self O ponteiro do objeto.
#  @param matrix Matriz de rotação 3x3.
def matrix2RotationVector(matrix, homogeneous = False):

    epsilon = 10e-14

    if homogeneous == True:
        pose = np.array([matrix[0,3], matrix[1,3], matrix[2,3]])
        matrix = matrix[0:3,0:3]

    if ((np.abs(np.trace(matrix) - 3)) <= epsilon):
        print("[INFO][matrix2RotationVector] Exception find in transform: Theta is 0")

        rpy = matrix2Pose(matrix[0:3,0:3],rotOnly = True)
        rv_vector = rollPitchYaw2RotationVector(rpy)

    elif ((np.abs(np.trace(matrix) + 1) <= epsilon)):

        print("[INFO][matrix2RotationVector] Exception find in transform: Theta is pi")

        rpy = matrix2Pose(matrix[0:3,0:3],rotOnly = True)
        rv_vector = rollPitchYaw2RotationVector(rpy)
    else:
        theta = np.arccos((np.trace(matrix) - 1)/2)
        rv_vector = np.array([(matrix[2,1] - matrix[1,2]), (matrix[0,2] - matrix[2,0]), (matrix[1,0] - matrix[0,1])])
        rv_vector = rv_vector * (theta/(2*np.sin(theta)))

    if homogeneous == True:
        rv_vector = np.array([pose[0],pose[1],pose[2],rv_vector[0],rv_vector[1],rv_vector[2]])

    return rv_vector

## Realiza a transformação de uma pose em RV para uma matrix.
# @param rv Pose em RV.
# @param homogeneous Parametro para definir transformação apenas de rotação ou matriz homogênea.
def rotationVector2Matrix(rv, homogeneous = False):

    epsilon = 10e-4

    if homogeneous == True:
        pose = rv[0:3]
        rv = rv[3:6]

    theta = norm(rv)
    if (theta > -epsilon) and (theta < epsilon):
        print("[INFO][rotationVector2Matrix] Exception find in transform: Theta is " + str(theta))
        rv_normalized = np.zeros(6)
    else:
        rv_normalized = rv/theta
    skew = np.array([[0, -rv_normalized[2], rv_normalized[1]],[rv_normalized[2], 0, -rv_normalized[0]],[-rv_normalized[1], rv_normalized[0], 0]])

    rot_matrix = np.eye(3) + np.sin(theta)*skew + (1 - np.cos(theta))*np.dot(skew,skew)

    if homogeneous == True:
        homogeneous_matrix = np.array([[rot_matrix[0,0],rot_matrix[0,1],rot_matrix[0,2],pose[0]],[rot_matrix[1,0],rot_matrix[1,1],rot_matrix[1,2],pose[1]],[rot_matrix[2,0],rot_matrix[2,1],rot_matrix[2,2],pose[2]],[0,0,0,1]])
        return homogeneous_matrix

    return rot_matrix