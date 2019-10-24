#!/usr/bin/python
# -*- coding: utf-8 -*-
## @package UR5
#  Documentação para o pacote de classes UR5.
#
#  Documentação do código produzido para controle do manipulador UR5 e geração e controle de suas posições.
#  Cada código aqui documentado possui uma breve descrição de sua função, suas entradas e saídas.
import numpy as np
from numpy.linalg import inv
from numpy.linalg import norm
from numpy.linalg import pinv
from scipy.signal import butter,lfilter
from scipy.signal import freqz
import struct
import time
import csv
import Transformations as tf
import os

## Documentação da Classe UR5Class para controle remoto do manipulador Universal Robots 5 (UR5).
#
#  Essa classe é responsável por interpretar os dados recebidos pela caixa de controle do UR5 e controlar seu funcionamento ao longo do projeto.
#  A ela cabe as funções dos cálculos de cinemática direta e inversa para as diversas posições do robô, interpretar os dados do robô, verificar
# seu estado de segurança e funcionamento, assim como realizar qualquer cálculo de calibração ou posição necessário.
class UR5Class:
        _standard_DH = np.mat([[0,-.425,-.39225,0,0,0], [1.570796327, 0, 0, 1.570796327, -1.570796327, 0], [.089159,0,0,.10915,.09465,.0823], [0, 0, 0, 0, 0, 0]])
        # _standard_DH é a tabela DH tradicional do Robô. As linhas correspondem respectivamente a (a, alpha, d,q)
        
        _robot_data = []
        # Lista vazia para receber os dados do robô

        _data_pack_max = 133
        # Tamanho maximo e esperado de valores recebidos em lista no pacote de dados
        processTimeList = []

        errorDB = []
        error_D_DB = []
        wDB = []
        u = np.array([0, 0, 0, 0, 0, 0],dtype=np.float64)
        errorSaturation = np.array([0, 0, 0, 0, 0, 0],dtype=np.float64)
        errorPrevious = np.array([0, 0, 0, 0, 0, 0],dtype=np.float64)
        errorSum = np.array([0, 0, 0, 0, 0, 0],dtype=np.float64)

        normErro = np.zeros(6,dtype=np.float64)

        ## Construtor da classe.
        # @param self O ponteiro do objeto.
        # @param delta_DH Os dados de calibração da matriz Denavit-Hartenberg do robô a ser controlado. 
        def __init__(self, delta_DH = np.zeros((5,6))):
            self.delta_standard_DH = delta_DH

            self._effective_a = self._standard_DH[0,:] + self.delta_standard_DH[0,:]
            self._effective_alpha = self._standard_DH[1,:] + self.delta_standard_DH[1,:]
            self._effective_d = self._standard_DH[2,:] + self.delta_standard_DH[2,:]
            self._effective_q = np.array(self._standard_DH[3,:] + self.delta_standard_DH[3,:])
            
            # Os dados efetivos equivalem aos dados esperados do UR5 mais os dados de calibração do robô específico.

            Rot_x_1 = np.mat([[1, 0, 0, 0], [0, np.cos(self._effective_alpha[0,0]), -np.sin(self._effective_alpha[0,0]), 0], [0, np.sin(self._effective_alpha[0,0]),  np.cos(self._effective_alpha[0,0]), 0], [ 0, 0, 0, 1]])
            Rot_x_2 = np.mat([[1, 0, 0, 0], [0, np.cos(self._effective_alpha[0,1]), -np.sin(self._effective_alpha[0,1]), 0], [0, np.sin(self._effective_alpha[0,1]),  np.cos(self._effective_alpha[0,1]), 0], [ 0, 0, 0, 1]])
            Rot_x_3 = np.mat([[1, 0, 0, 0], [0, np.cos(self._effective_alpha[0,2]), -np.sin(self._effective_alpha[0,2]), 0], [0, np.sin(self._effective_alpha[0,2]),  np.cos(self._effective_alpha[0,2]), 0], [ 0, 0, 0, 1]])
            Rot_x_4 = np.mat([[1, 0, 0, 0], [0, np.cos(self._effective_alpha[0,3]), -np.sin(self._effective_alpha[0,3]), 0], [0, np.sin(self._effective_alpha[0,3]),  np.cos(self._effective_alpha[0,3]), 0], [ 0, 0, 0, 1]])
            Rot_x_5 = np.mat([[1, 0, 0, 0], [0, np.cos(self._effective_alpha[0,4]), -np.sin(self._effective_alpha[0,4]), 0], [0, np.sin(self._effective_alpha[0,4]),  np.cos(self._effective_alpha[0,4]), 0], [ 0, 0, 0, 1]])
            Rot_x_6 = np.mat([[1, 0, 0, 0], [0, np.cos(self._effective_alpha[0,5]), -np.sin(self._effective_alpha[0,5]), 0], [0, np.sin(self._effective_alpha[0,5]),  np.cos(self._effective_alpha[0,5]), 0], [ 0, 0, 0, 1]])

            Trans_d_1 = np.mat([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, self._effective_d[0,0]], [0, 0, 0, 1]])
            Trans_d_2 = np.mat([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, self._effective_d[0,1]], [0, 0, 0, 1]])
            Trans_d_3 = np.mat([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, self._effective_d[0,2]], [0, 0, 0, 1]])
            Trans_d_4 = np.mat([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, self._effective_d[0,3]], [0, 0, 0, 1]])
            Trans_d_5 = np.mat([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, self._effective_d[0,4]], [0, 0, 0, 1]])
            Trans_d_6 = np.mat([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, self._effective_d[0,5]], [0, 0, 0, 1]])

            Trans_a_1 = np.mat([[1, 0, 0, self._effective_a[0,0]], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            Trans_a_2 = np.mat([[1, 0, 0, self._effective_a[0,1]], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            Trans_a_3 = np.mat([[1, 0, 0, self._effective_a[0,2]], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            Trans_a_4 = np.mat([[1, 0, 0, self._effective_a[0,3]], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            Trans_a_5 = np.mat([[1, 0, 0, self._effective_a[0,4]], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            Trans_a_6 = np.mat([[1, 0, 0, self._effective_a[0,5]], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

            self._A_0_1 = Trans_d_1 * Trans_a_1 * Rot_x_1
            self._A_0_2 = Trans_d_2 * Trans_a_2 * Rot_x_2
            self._A_0_3 = Trans_d_3 * Trans_a_3 * Rot_x_3
            self._A_0_4 = Trans_d_4 * Trans_a_4 * Rot_x_4
            self._A_0_5 = Trans_d_5 * Trans_a_5 * Rot_x_5
            self._A_0_6 = Trans_d_6 * Trans_a_6 * Rot_x_6
            # Transformações comuns, indiferentes a movimentação, utilizadas em cálculos futuros.

            return
        ## Método que recebe e configura o pacote de dados do robô.
        #  @param self O ponteiro do objeto.
        #  @param data O pacote de dados recebido pela conexão Ethernet com o robô.
        def setRobotData(self, data):
            size = len(data)
            self._robot_data = []
            # O primeiro dado recebido, de tempo, é um inteiro de 4 bytes.
            self._robot_data.append(struct.unpack('!i', data[0:4]))
            i = 4
            # O resto dos dados recebidos vem em formato de double de 8 bytes.
            while i < size:
                    self._robot_data.append(struct.unpack('!d', data[i:i+8])[0])
                    i += 8
            # Já atualiza os dados de juntas do robô.
            if (size < (4+(34*8))):
                print("[WARNING] Data size smaller than expected. Bytes: " + str(size))
                return

            self._effective_q = np.array(self._robot_data[32:38]) + self.delta_standard_DH[3,:]
            return 
        # setRobotData recebe o pacote de 1060 bytes e os separa nos 160 valores da lista de dados.

        def setRobotDataRTDE(self, data):

            #print(data.actual_TCP_pose)
            self._robot_data[1] = np.asarray(data.timestamp, dtype = np.float64)
            self._robot_data[2:8] = np.asarray(data.target_q, dtype = np.float64)
            self._robot_data[8:14] = np.asarray(data.target_qd, dtype = np.float64)

            self._robot_data[32:38] = np.asarray(data.actual_q, dtype = np.float64)
            self._robot_data[38:44] = np.asarray(data.actual_qd, dtype = np.float64)

            self._robot_data[56:62] = np.asarray(data.actual_TCP_pose, dtype = np.float64)

            self._robot_data[62:68] = np.asarray(data.actual_TCP_speed, dtype = np.float64)
            self._robot_data[68:74] = np.asarray(data.actual_TCP_force, dtype = np.float64)

            self._robot_data[74:80] = np.asarray(data.target_TCP_pose, dtype = np.float64)
            self._robot_data[80:86] = np.asarray(data.target_TCP_speed, dtype = np.float64)

            self._robot_data[102] = np.asarray(data.safety_mode, dtype = np.int32)

            self._robot_data[132] = np.asarray(data.runtime_state, dtype = np.uint32)



            q = np.asarray(data.actual_q)

            self._effective_q = q + self.delta_standard_DH[3,:]
            # <field name="timestamp" type="DOUBLE"/>
            # <field name="target_q" type="VECTOR6D"/>
            # <field name="target_qd" type="VECTOR6D"/>
            # <field name="target_qdd" type="VECTOR6D"/>
            # <field name="target_current" type="VECTOR6D"/>
            # <field name="target_moment" type="VECTOR6D"/>
            # <field name="actual_q" type="VECTOR6D"/>
            # <field name="actual_qd" type="VECTOR6D"/>
            # <field name="actual_current" type="VECTOR6D"/>
            # <field name="joint_control_output" type="VECTOR6D"/>
            # <field name="actual_TCP_pose" type="VECTOR6D"/>
            # <field name="actual_TCP_speed" type="VECTOR6D"/>
            # <field name="actual_TCP_force" type="VECTOR6D"/>
            # <field name="target_TCP_pose" type="VECTOR6D"/>
            # <field name="target_TCP_speed" type="VECTOR6D"/>
            # <field name="actual_digital_input_bits" type="UINT64"/>
            # <field name="joint_temperatures" type="VECTOR6D"/>
            # <field name="actual_execution_time" type="DOUBLE"/>
            # <field name="robot_mode" type="INT32"/>
            # <field name="joint_mode" type="VECTOR6INT32"/>
            # <field name="safety_mode" type="INT32"/>
            # <field name="actual_tool_accelerometer" type="VECTOR3D"/>
            # <field name="speed_scaling" type="DOUBLE"/>
            # <field name="target_speed_fraction" type="DOUBLE"/>
            # <field name="actual_momentum" type="DOUBLE"/>
            # <field name="actual_main_voltage" type="DOUBLE"/>
            # <field name="actual_robot_voltage" type="DOUBLE"/>
            # <field name="actual_robot_current" type="DOUBLE"/>
            # <field name="actual_joint_voltage" type="VECTOR6D"/>
            # <field name="actual_digital_output_bits" type="UINT64"/>
            # <field name="runtime_state" type="UINT32"/>
            return

        ## Retorna verdadeiro ou falso para o estado de segurança do robô.
        #  @param self O ponteiro do objeto.
        def checkSafety(self):
            try:
                if self._robot_data[102] == 1:
                        safety = True
                else:
                        safety = False
                return safety
            except:
                print("[ERROR] Could not find Robot Data!")
                return None
        # checkSafety verifica se a variável de segurança do robô está apta a operar

        ## Retorna verdadeiro ou falso para o estado de operação do robô.
        #  @param self O ponteiro do objeto.
        def programStateCheck(self):
            try:
                if self._robot_data[132] == 1:
                    state = True
                else:
                    state = False
                return state
            except:
                print("[ERROR] Could not find Robot Data!")
                return None
        # programStateCheck verifica se a variável de estado do robô está apta a operar

        ## Imprime em prompt de comando as 133 informações recebidas pelo pacote de dados do robô.
        #  @param self O ponteiro do objeto.
        def printRobotData(self):
            size = len(self._robot_data)

            if size == self._datapackmax:
                    print("[INFO] Message Size in Bytes: " + str(self._robot_data[0]))
                    print("[INFO] Time: " + str(self._robot_data[1]))
                    print("[INFO] q target" + str(self._robot_data[2:8]))
                    print("[INFO] qd target" + str(self._robot_data[8:14]))
                    print("[INFO] qdd target" + str(self._robot_data[14:20]))
                    print("[INFO] I target" + str(self._robot_data[20:26]))
                    print("[INFO] M target" + str(self._robot_data[26:32]))
                    print("[INFO] q actual" + str(self._robot_data[32:38]))
                    print("[INFO] qd actual" + str(self._robot_data[38:44]))
                    print("[INFO] I actual" + str(self._robot_data[44:50]))
                    print("[INFO] I control" + str(self._robot_data[50:56]))
                    print("[INFO] Tool Vector Actual" + str(self._robot_data[56:62]))
                    print("[INFO] TCP Speed Actual" + str(self._robot_data[62:68]))
                    print("[INFO] TCP Force" + str(self._robot_data[68:74]))
                    print("[INFO] Tool Vector Target" + str(self._robot_data[74:80]))
                    print("[INFO] TCP Speed Target" + str(self._robot_data[80:86]))
                    print("[INFO] digital input bits" + str(self._robot_data[86]))
                    print("[INFO] Motor Temperatures" + str(self._robot_data[87:93]))
                    print("[INFO] Controller Timer" + str(self._robot_data[93]))
                    print("[INFO] Test Value" + str(self._robot_data[94]))
                    print("[INFO] Robot Mode" + str(self._robot_data[95]))
                    print("[INFO] Joint Modes" + str(self._robot_data[96:102]))
                    print("[INFO] Safety Mode" + str(self._robot_data[102]))
                    print("[INFO] Tool Acceleration Values" + str(self._robot_data[109:112]))
                    print("[INFO] Speed Scaling" + str(self._robot_data[118]))
                    print("[INFO] Linear Momentum Norm" + str(self._robot_data[119]))
                    print("[INFO] V Main" + str(self._robot_data[122]))
                    print("[INFO] V Robot" + str(self._robot_data[123]))
                    print("[INFO] I Robot" + str(self._robot_data[124]))
                    print("[INFO] V actual" + str(self._robot_data[125:131]))
                    print("[INFO] Digital Outputs" + str(self._robot_data[131]))
                    print("[INFO] Program State" + str(self._robot_data[132]))
            # Exceção caso o pacote venha menor que 1060 Bytes
            else:
                    print("[WARNING] Size of data smaller than expected: ", size)
            return
        # printRobotData imprime em tela todos os valores do pacote de dados traduzido, para depuração

        ## Retorna o vetor de posição do efetuador do robô, em formato [x, y, z, rx, ry, rz].
        #  @param self O ponteiro do objeto.
        def getPositionTarget(self):
            try:
                array = np.array(self._robot_data[74:80])
                return array
            except:
                print("[ERROR] Could not find Robot Data!")
                return None
        # getPosition retorna a posição atual do vetor da ferramenta.

        def getPosition(self):
            try:
                array = np.array(self._robot_data[56:62])
                return array
            except:
                print("[ERROR] Could not find Robot Data!")
                return None
        # getPosition retorna a posição atual do vetor da ferramenta.

        ## Retorna o vetor de velocidade do efetuador do robô, em formato [dx, dy, dz, drx, dry, drz].
        #  @param self O ponteiro do objeto.
        def getTCPSpeed(self):
            try:
                array = np.array(self._robot_data[62:68])
                return array
            except:
                print("[ERROR] Could not find Robot Data!")
                return None
        # getTCPSpeed retorna a velocidade  da ferramenta.

        ## Retorna o vetor de velocidade do efetuador do robô, em formato [dx, dy, dz, drx, dry, drz].
        #  @param self O ponteiro do objeto.
        def getTCPSpeedTarget(self):
            try:
                array = np.array(self._robot_data[80:86])
                return array
            except:
                print("[ERROR] Could not find Robot Data!")
                return None
        # getTCPSpeed retorna a velocidade  da ferramenta.

        ## Retorna o vetor de velocidade modular do efetuador do robô, em formato [v].
        #  @param self O ponteiro do objeto.
        def getTCPSpeedMod(self):
            try:
                v = np.sqrt(self._robot_data[62]*self._robot_data[62] + self._robot_data[63]*self._robot_data[63] + self._robot_data[64]*self._robot_data[64])
                return v
            except:
                print("[ERROR] Could not find Robot Data!")
                return None
        # getTCPSpeed retorna a velocidade  da ferramenta.

        ## Retorna o vetor de posição das seis juntas do robô.
        #  @param self O ponteiro do objeto.
        def getJointPosition(self):
            try:
                array = np.array(self._robot_data[32:38])
                return array
            except:
                print("[ERROR] Could not find Robot Data!")
                return None

        ## Retorna o vetor de posição das seis juntas do robô.
        #  @param self O ponteiro do objeto.
        def getJointPositionTarget(self):
            try:
                array = np.array(self._robot_data[2:8])
                return array
            except:
                print("[ERROR] Could not find Robot Data!")
                return None
        # Retorna o valor das articulações da ferramenta

        ## Retorna o vetor de velocidade das seis juntas do robô.
        #  @param self O ponteiro do objeto.
        def getJointSpeed(self):
            try:
                array = np.array(self._robot_data[38:44])
                return array
            except:
                print("[ERROR] Could not find Robot Data!")
                return None
        # getJointSpeed retorna a velocidade da ferramenta.

        ## Retorna o vetor de velocidade das seis juntas do robô.
        #  @param self O ponteiro do objeto.
        def getJointSpeedTarget(self):
            try:
                array = np.array(self._robot_data[8:14])
                return array
            except:
                print("[ERROR] Could not find Robot Data!")
                return None
        # getJointSpeed retorna a velocidade da ferramenta.

        def getTCPForce(self):
            try:
                array = np.array(self._robot_data[68:74])
                return array
            except:
                print("[ERROR] Could not find Robot Data!")
                return None
        # getJointSpeed retorna a velocidade da ferramenta.

        ## Retorna o tempo atual do robô desde que foi ligado.
        #  @param self O ponteiro do objeto.
        def getTime(self):
            return self._robot_data[1]

        # Retorna o valor do tempo de uso atual

        ## Realiza a cinemática direta do UR5 para a posição de juntas atual. O método retorna a matriz homogênea 4x4 da posição atual, ou um vetor em RV ou RPY.
        #  @param self O ponteiro do objeto.
        #  @param q O vetor de coordenadas de junta.
        #  @param vector parâmetro que define se o tipo de retorno como vetor de posições em RV.
        #  @param rpy parâmetro que, juntamente de vector, define o retorno como vetor de posições em RPY.
        def ur5_direct_kinematics(self, q, vector = False, rpy = False, apply_offset = False):

            if (apply_offset == True):
                # q = q + self.delta_standard_DH[3,:]
                q = np.squeeze(np.asarray(q + self.delta_standard_DH[3,:]))

            _rot_z_1 = np.mat([[np.cos(q[0]), -np.sin(q[0]), 0, 0],[np.sin(q[0]), np.cos(q[0]), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            _rot_z_2 = np.mat([[np.cos(q[1]), -np.sin(q[1]), 0, 0],[np.sin(q[1]), np.cos(q[1]), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            _rot_z_3 = np.mat([[np.cos(q[2]), -np.sin(q[2]), 0, 0],[np.sin(q[2]), np.cos(q[2]), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            _rot_z_4 = np.mat([[np.cos(q[3]), -np.sin(q[3]), 0, 0],[np.sin(q[3]), np.cos(q[3]), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            _rot_z_5 = np.mat([[np.cos(q[4]), -np.sin(q[4]), 0, 0],[np.sin(q[4]), np.cos(q[4]), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            _rot_z_6 = np.mat([[np.cos(q[5]), -np.sin(q[5]), 0, 0],[np.sin(q[5]), np.cos(q[5]), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

            # Utiliza as matrizes definidas no construtor e as de rotação das juntas atuais para retornar a matriz final.
            self._A_1 = _rot_z_1 * self._A_0_1
            self._A_2 = _rot_z_2 * self._A_0_2
            self._A_3 = _rot_z_3 * self._A_0_3
            self._A_4 = _rot_z_4 * self._A_0_4
            self._A_5 = _rot_z_5 * self._A_0_5
            self._A_6 = _rot_z_6 * self._A_0_6

            self._H = self._A_1 * self._A_2 * self._A_3 * self._A_4 * self._A_5 * self._A_6
            #print self._H

            if (vector == False):
                return self._H
            else:
                vetor = tf.matrix2RotationVector(self._H[0:3,0:3])
                array = np.array([self._H[0,3], self._H[1,3], self._H[2,3]], float)
                vetor = np.hstack((array,vetor))
                #print vetor
                if (rpy == False):
                    return vetor
                else:
                    vetor[3:6] = tf.rotationVector2RollPitchYaw(vetor[3:6])
                    return vetor
        # ur5_direct_kinematics executa a cinemática direta do UR5 e retorna a matriz 4x4 de posição e orientação do UR5


        def verifyDelta(self, epsilon = 10e-6):

            direct = self.ur5_direct_kinematics(self.getJointPosition(), vector = True, apply_offset = True)
            real = self.getPosition()

            diff = tf.computeDifference(real,direct)

            print("[INFO] Direct Kinematics calculated with Delta: " + str(direct))
            print("[INFO] Direct Kinematics real: " + str(real))

            error = norm(diff[0:3])

            print("[INFO] Error: ", error)


            if (error < epsilon):
                print("[INFO] Correct Delta Matrix!")
                return True
            else:
                print("[WARNING] Incorrect Delta Matrix!")
                return False


        def _DH(self, a, alpha, d, theta):

            Td = np.asmatrix(np.eye(4))
            Td[2,3] = d
            Ta = np.asmatrix(np.eye(4))
            Ta[0,3] = a
            Rtheta = tf.Rot_z(theta)
            Rtheta = np.mat([[Rtheta[0,0], Rtheta[0,1], Rtheta[0,2], 0], [Rtheta[1,0], Rtheta[1,1], Rtheta[1,2], 0], [Rtheta[2,0], Rtheta[2,1], Rtheta[2,2], 0], [0,0,0,1]])
            Ralpha = tf.Rot_x(alpha)
            Ralpha = np.mat([[Ralpha[0,0], Ralpha[0,1], Ralpha[0,2], 0], [Ralpha[1,0], Ralpha[1,1], Ralpha[1,2], 0], [Ralpha[2,0], Ralpha[2,1], Ralpha[2,2], 0], [0,0,0,1]])

            G = Td * Rtheta * Ta * Ralpha

            return G
        # _DH retorna uma matrix 4x4 de junta especifica, utilizado na cinemática inversa analítica


        def _analytic_ur5_inverse_kinematics(self, p):


            rvMatrix = tf.rotationVector2Matrix(p[3:6])

            gd = np.mat(([[rvMatrix[0,0], rvMatrix[0,1], rvMatrix[0,2], p[0]], [rvMatrix[1,0], rvMatrix[1,1], rvMatrix[1,2], p[1]], [rvMatrix[2,0], rvMatrix[2,1], rvMatrix[2,2], p[2]], [0, 0, 0, 1]]))

            theta = np.zeros((6, 8))

            d1 = self._standard_DH[2,0]
            d2 = self._standard_DH[2,1]
            d3 = self._standard_DH[2,2]
            d4 = self._standard_DH[2,3]
            d5 = self._standard_DH[2,4]
            d6 = self._standard_DH[2,5]

            a1 = self._standard_DH[0,0]
            a2 = self._standard_DH[0,1]
            a3 = self._standard_DH[0,2]
            a4 = self._standard_DH[0,3]
            a5 = self._standard_DH[0,4]
            a6 = self._standard_DH[0,5]

            alpha1 = self._standard_DH[1,0]
            alpha2 = self._standard_DH[1,1]
            alpha3 = self._standard_DH[1,2]
            alpha4 = self._standard_DH[1,3]
            alpha5 = self._standard_DH[1,4]
            alpha6 = self._standard_DH[1,5]

            # Calculating theta1
            p05 = gd * np.mat([[0], [0], [-d6], [1]])
            p05 = p05 - np.mat([[0], [0], [0], [1]])
            psi = np.arctan2(p05[1], p05[0])
            p05xy = np.sqrt(p05[1]*p05[1] + p05[0]*p05[0])
            if (d4 > p05xy):
                print ("[WARNING] No solution for Theta1: d4 > P05xy")
                print ("[WARNING] Creating aproximation highly inaccurate")
                d4 = p05xy - 1e-10
            try:
                phi = np.arccos(d4 / p05xy)
            except:
                print("[ERROR] Division by zero: " + str(p05xy))
                return None
            theta[0, 0:4] = np.radians(90) + psi + phi
            theta[0, 4:8] = np.radians(90) + psi - phi
            theta = np.real(theta)

            # Calculating theta5
            cols = np.array([0, 4])
            for i in range(0, cols.size):
                c = cols[i];
                try:
                    T10 = inv(self._DH(a1, alpha1, d1, theta[0,c]))
                except:
                    print("[ERROR] Could not find inverse: " + str(self._DH(a1, alpha1, d1, theta[0,c])))
                    return None
                T16 = T10 * gd
                p16z = T16[2,3]
                try:
                    if (((p16z-d4)/d6) > 1):
                        print ("[WARNING] No solution for Theta5: (p16z-d4)/d6) > 1")
                        print ("[WARNING] Creating aproximation highly inaccurate")
                        d6 = (p16z-d4) + 1e-10
                    t5 = np.arccos((p16z-d4)/d6)
                except:
                    print("[ERROR] Division by zero: " + str(d6))
                    return None
                theta[4, c:c+2] = t5
                theta[4, c+2:c+4] = -t5
            theta = np.real(theta)

            # Calculating theta6
            cols = np.array([0, 2, 4, 6])
            for i in range(0, cols.size):
                c = cols[i]
                T01 = self._DH(a1, alpha1, d1, theta[0,c])
                try:
                    T61 = inv(gd) * T01
                except:
                    print("[ERROR] Could not find inverse: " + str(gd))
                    return None
                T61zy = T61[1, 2]
                T61zx = T61[0, 2]
                t5 = theta[4, c]
                if (np.sin(t5) == 0):
                    theta[5, c:c+2] = 0
                else:    
                    theta[5, c:c+2] = np.arctan2(-T61zy/np.sin(t5), T61zx/np.sin(t5))
            theta = np.real(theta)

            # Calculating theta3
            cols = np.array([0, 2, 4, 6])
            for i in range (0, cols.size):
                c = cols[i]
                try:
                    T10 = inv(self._DH(a1, alpha1, d1, theta[0,c]))
                    T65 = inv(self._DH(a6, alpha6, d6, theta[5,c]))
                    T54 = inv(self._DH(a5, alpha5, d5, theta[4,c]))
                except T10:
                    print("[ERROR] Could not find inverse: Theta3, inverse 1, " + str(T10))
                    return None
                except T65:
                    print("[ERROR] Could not find inverse: Theta3, inverse 2, " + str(T65))
                    return None
                except T54:
                    print("[ERROR] Could not find inverse: Theta3, inverse 3, " + str(T54))
                    return None
                T14 = T10 * gd * T65 * T54
                p13 = T14 * np.mat([[0], [-d4], [0], [1]])
                p13 = p13 - np.mat([[0], [0], [0], [1]])
                p13norm2 = norm(p13) * norm(p13)
                arg = (p13norm2-a2*a2-a3*a3)/(2*a2*a3)
                if (arg > 1 or arg < -1):
                    print ("[WARNING] No solution for Theta3: arg < -1 or arg > 1")
                    print ("[WARNING] Creating aproximation highly inaccurate")
                    if (arg >1):
                        arg = 1 - 1e-10
                    else:
                        arg = -1 + 1e-10
                t3p = np.arccos(arg)
                theta[2, c] = t3p
                theta[2, c+1] = -t3p
            theta = np.real(theta)

            # Calculating theta2 and theta4
            cols = np.array([0, 1, 2, 3, 4, 5, 6, 7])
            for i in range (0, cols.size):
                c = cols[i]
                try:
                    T10 = inv(self._DH(a1, alpha1, d1, theta[0,c]))
                    T65 = inv(self._DH(a6, alpha6, d6, theta[5,c]))
                    T54 = inv(self._DH(a5, alpha5, d5, theta[4,c]))
                except T10:
                    print("[ERROR] Could not find inverse: Theta2 inverse 1, " + str(T10))
                    return None
                except T65:
                    print("[ERROR] Could not find inverse: Theta2, inverse 2, " + str(T65))
                    return None
                except T54:
                    print("[ERROR] Could not find inverse: Theta2, inverse 3, " + str(T54))
                    return None
                T14 = T10 * gd * T65 * T54
                p13 = T14 * np.mat([[0], [-d4], [0], [1]]) - np.mat([[0], [0], [0], [1]])
                p13norm = norm(p13)
                theta[1, c] = -np.arctan2(p13[1], -p13[0])+np.arcsin(a3*np.sin(theta[2,c])/p13norm)
                try:
                    T32 = inv(self._DH(a3, alpha3, d3, theta[2,c]))
                    T21 = inv(self._DH(a2, alpha2, d2, theta[1,c]))
                except T10:
                    print("[ERROR] Could not find inverse: Theta4 inverse 1, " + str(T32))
                    return None
                except T65:
                    print("[ERROR] Could not find inverse: Theta4, inverse 2, " + str(T21))
                    return None
                T34 = T32 * T21 * T14;
                theta[3, c] = np.arctan2(T34[1,0], T34[0,0])
            theta = np.real(theta)

            for i in range (0, 5):
                for j in range(0,7):
                    if theta[i,j] > np.pi:
                        theta[i,j] -= 2*np.pi
                    elif theta[i,j] < -np.pi:
                        theta[i,j] += 2*np.pi

            return theta
        # _analytic_ur5_inverse_kinematics retorna a matriz 6x8 com as 8 possiveis posições de 6 angulos dos motores que inferem na posição atual do UR5

        ## Cálcula a matriz Jacobiana da relação entre juntas e vetor de pose.
        #  @param self O ponteiro do objeto.
        #  @param q_Past Um vetor de juntas inicial a ser aplicado a derivada.
        #  @param deltaTheta Um vetor de diferença de juntas em um tempo infinitesimal para o cálculo de derivada.
        def jacobian(self, q_Past, deltaTheta, rpy = False):

            jacobian_matrix = np.zeros((6,6))
            FK_init = self.ur5_direct_kinematics(np.squeeze(np.asarray(q_Past.transpose() + self.delta_standard_DH[3,:])), vector = True, rpy = rpy)
            step = deltaTheta
            NaN_check = False

            for i in range(0,6):
                q_aux = np.array([[0],[0],[0],[0],[0],[0]], float)
                q_aux[i] += step[i]
                q_aux = q_Past + q_aux
                q_aux = np.squeeze(np.asarray(q_aux.transpose() + self.delta_standard_DH[3,:]))
                FK_next = self.ur5_direct_kinematics(q_aux, vector = True, rpy = rpy)
                jacobian_matrix[i,:] = (tf.computeDifference(FK_next, FK_init)/(step[i]))
                if(np.any(np.isnan(jacobian_matrix[i,:]))):
                    jacobian_matrix[i,:] = np.zeros(6)
                    NaN_check = True
            
            if(NaN_check):
                print("[WARNING] NaN found on Jacobian.")

            return jacobian_matrix.transpose()

        def jacobian2(self, q):

            jacobian_matrix = np.zeros((6,6))

            # Atualiza as matrizes

            self.ur5_direct_kinematics(np.squeeze(np.asarray(q.transpose() + self.delta_standard_DH[3,:])))

            # R^0_{i-1}dot(0,0,1)cross(d^0_n - d^0_{i-1})

            auxRow = np.array([[0],[0],[1]])
            # Row 1

            jacobian_matrix[0:3,0] = np.cross(np.dot(np.eye(3),auxRow),self._H[0:3,3],axisa=0,axisb=0,axisc=1)
            jacobian_matrix[3:6,0] = np.dot(np.eye(3),auxRow).transpose()

            # Row 2
            
            jacobian_matrix[0:3,1] = np.cross(np.dot(self._A_1[0:3,0:3],auxRow),(self._H[0:3,3] - self._A_1[0:3,3]),axisa=0,axisb=0,axisc=1)
            jacobian_matrix[3:6,1] = np.dot(self._A_1[0:3,0:3],auxRow).transpose()

            # Row 3

            aux = self._A_1 * self._A_2

            jacobian_matrix[0:3,2] = np.cross(np.dot(aux[0:3,0:3],auxRow),(self._H[0:3,3] - aux[0:3,3]),axisa=0,axisb=0,axisc=1)
            jacobian_matrix[3:6,2] = np.dot(aux[0:3,0:3],auxRow).transpose()

            # Row 4

            aux = aux * self._A_3

            jacobian_matrix[0:3,3] = np.cross(np.dot(aux[0:3,0:3],auxRow),(self._H[0:3,3] - aux[0:3,3]),axisa=0,axisb=0,axisc=1)
            jacobian_matrix[3:6,3] = np.dot(aux[0:3,0:3],auxRow).transpose()

            # Row 5

            aux = aux * self._A_4

            jacobian_matrix[0:3,4] = np.cross(np.dot(aux[0:3,0:3],auxRow),(self._H[0:3,3] - aux[0:3,3]),axisa=0,axisb=0,axisc=1)
            jacobian_matrix[3:6,4] = np.dot(aux[0:3,0:3],auxRow).transpose()

            # Row 6

            aux = aux * self._A_5

            jacobian_matrix[0:3,5] = np.cross(np.dot(aux[0:3,0:3],auxRow),(self._H[0:3,3] - aux[0:3,3]),axisa=0,axisb=0,axisc=1)
            jacobian_matrix[3:6,5] = np.dot(aux[0:3,0:3],auxRow).transpose()

            return jacobian_matrix

        def jacobianEndEffectorReference(self,jacobian):

            fowardKinematics = self._H

            jacobianTransform = np.eye(6)
            #jacobianTransform[0:3,0:3] = fowardKinematics[0:3,0:3].transpose()
            jacobianTransform[3:6,3:6] = fowardKinematics[0:3,0:3].transpose()

            newJacobian = np.dot(jacobianTransform,jacobian)

            return newJacobian


        def jacobianAnalytic(self, q):

            pose = self.ur5_direct_kinematics(np.squeeze(np.asarray(q.transpose() + self.delta_standard_DH[3,:])),vector = True, rpy = True)

            jacobian = self.jacobian2(q)
            jacobian = self.jacobianEndEffectorReference(jacobian)

            # r = pose[3]
            # p = pose[4]
            # #y = pose[5]

            # B = np.array([[1,0,np.sin(p)],[0,np.cos(r),-np.cos(p)*np.sin(r)],[0,np.sin(r),np.cos(p)*np.cos(r)]])
            # invB = inv(B)
            # auxMat = np.eye(6)
            # auxMat[3:6,3:6] = invB

            # jacobianAnalytic = np.dot(auxMat,jacobian)

            #jacobianAnalytic = self.jacobianEndEffectorReference(jacobianAnalytic)

            return jacobian

        ## Esse método realiza a cinemática inversa de uma posição espacial para uma das oito configurações possíveis no espaço utilizando aproximação numérica por Newton-Raphson. 
        # Ele retorna um vetor com as seis juntas que representam a configuração escolhida.
        #  @param self O ponteiro do objeto.
        #  @param cartesian_position Vetor [1x6] da posição a ser transformada.
        #  @param chosen_theta Configuração escolhida. Default = 2.
        #  @param theta Um parametro que pode ser usado como posição proxima inicial para aproximação numérica
        #  @param rpy Um parâmetro que especifica se a posição cartesiana dada foi em RV ou RPY.
        def ur5_inverse_kinematics_newthon_raphson(self, cartesian_position, chosen_theta = 2, theta = np.zeros(6), rpy = False):

            #t = time.clock()

            if (rpy == True):
                cartesian_position[3:6] = tf.rollPitchYaw2RotationVector(cartesian_position[3:6])
            # A cinemática inversa analitica é inicialmente calculada
            if (np.all(theta == 0)):
                theta = self._analytic_ur5_inverse_kinematics(cartesian_position)
                joint_analytic_IK = theta[:,chosen_theta]
            else:
                joint_analytic_IK = theta

            NaN_check = np.isnan(joint_analytic_IK) 

            if (np.any(NaN_check)):
                joint_analytic_IK = self.getJointPosition()
                print ("[WARNING] Nan position found in analytic IK solution, using Actual Joint Position as start position.")

            # O vetor de juntas inicial a ser corrigido numéricamente é escolhido
            
            #print joint_analytic_IK

            q_i = np.array([0,0,0,0,0,0], float)
            q_i += joint_analytic_IK
            
            joint_analytic_IK = joint_analytic_IK + self.delta_standard_DH[3,:]
            joint_analytic_IK = np.squeeze(np.asarray(joint_analytic_IK))
            FK = self.ur5_direct_kinematics(joint_analytic_IK, True)


            # Transformação de RV para RPY é realizada para se iniciar o cálculo.
            cartesian_position_rpy = cartesian_position
            erro = tf.computeDifference(cartesian_position_rpy, FK)
            
            norm_erro = norm(erro)

            episilon = 0.0001*0.0001
            max_iteractions = 500
            iteraction = 1
            q_i = np.array([[q_i[0]], [q_i[1]],[q_i[2]], [q_i[3]],[q_i[4]], [q_i[5]]])
            erro = np.array([[erro[0]], [erro[1]],[erro[2]], [erro[3]],[erro[4]], [erro[5]]])

            delta_theta = np.ones(6)*0.000006
            delta_theta = np.array([[delta_theta[0]], [delta_theta[1]],[delta_theta[2]], [delta_theta[3]],[delta_theta[4]], [delta_theta[5]]])
            while (norm_erro > episilon):
                # Calcula
                j = self.jacobian(q_i, delta_theta)
                try:
                    jt = pinv(j)
                except:
                    print("[WARNING] Pseudo Inverse with SVD diverged")
                    jt = np.dot(j.transpose(),inv(np.dot(j,j.transpose())))

                q_in = np.array([[0],[0],[0],[0],[0],[0]], float)
                q_in = q_i + np.dot(jt,erro)

                delta_theta = q_in - q_i
                q_i = np.array([[0],[0],[0],[0],[0],[0]], float)
                q_i += q_in
                q_i = np.squeeze(np.asarray(q_i.transpose()))
                FK = self.ur5_direct_kinematics(np.squeeze(np.asarray(q_i + self.delta_standard_DH[3,:])), True)
                erro = tf.computeDifference(cartesian_position_rpy, FK)
                norm_erro = norm(erro)

                erro = np.array([[erro[0]], [erro[1]],[erro[2]], [erro[3]],[erro[4]], [erro[5]]])
                
                q_i = np.array([[q_i[0]], [q_i[1]],[q_i[2]], [q_i[3]],[q_i[4]], [q_i[5]]])
                
                iteraction += 1
                if (iteraction > max_iteractions):
                    print ("[ERROR] Maximum interactions reached.")
                    break

            #t2 = time.clock()

            #print ("Tempo de convergencia NRa: ", t2 - t)

            q_i = q_i.transpose()
            q_aux = np.array([q_i[0,0],q_i[0,1],q_i[0,2],q_i[0,3],q_i[0,4],q_i[0,5]], float)

            return q_aux

        ## Esse método realiza a cinemática inversa de uma posição espacial para uma das oito configurações possíveis no espaço utilizando aproximação numérica por Cyclic Coordinate Descent. 
        # Ele retorna um vetor com as seis juntas que representam a configuração escolhida. Obs.: Lento.
        #  @param self O ponteiro do objeto.
        #  @param cartesian_position Vetor [1x6] da posição a ser transformada.
        #  @param chosen_theta Configuração escolhida. Default = 2.
       
        def ur5_inverse_kinematics_ccd(self, cartesian_position, chosen_theta = 2):

            # A cinemática inversa analitica é inicialmente calculada

            t = time.clock()

            theta = self._analytic_ur5_inverse_kinematics(cartesian_position)

            # O vetor de juntas inicial a ser corrigido numéricamente é escolhido
            joint_analytic_IK = theta[:,chosen_theta]

            self._effective_q = joint_analytic_IK + self.delta_standard_DH[3,:]
            Initial_DK = self.ur5_direct_kinematics(np.squeeze(np.asarray(self._effective_q.transpose())), True)
            Initial_DK[3:6] = tf.rotationVector2RollPitchYaw(Initial_DK[3:6])
            # Cyclic Coordinate Descent
            cartesian_position_rpy = np.hstack((cartesian_position[0:3], tf.rotationVector2RollPitchYaw(cartesian_position[3:6])))

            # Constantes a serem utilizadas
            epsilon = 0.0001
            quad_epsilon = epsilon*epsilon
            joint_count = 5
            max_interection = 5000
            interection_count = 1
            interection_count_joint = 1
            direction = 1
            min_step = 0.000017
            max_step = 0.1
            alpha_step = max_step

            Radius = np.sqrt(cartesian_position[0:3].transpose()*cartesian_position[0:3])

            joint_interact = np.zeros(6)
            joint_interact += joint_analytic_IK

            # Erros Iniciais

            Error_Position = cartesian_position[0:3] - Initial_DK[0:3]
            Mean_Position = np.mean(np.dot(Error_Position.transpose(),Error_Position))

            Error_Rotation = tf.computeDifference(cartesian_position_rpy[3:6],Initial_DK[3:6], True)
            Linear_Rotation_Error = Radius*Error_Rotation
            Mean_Rotation = np.mean(np.dot(Linear_Rotation_Error,Linear_Rotation_Error.transpose()))

            erro_quad = (Mean_Position + Mean_Rotation)/2

            erro_quad_aux = erro_quad

            # Correção numérica.
            while erro_quad > quad_epsilon:
    
                joint_interact[joint_count] = joint_interact[joint_count] + direction*alpha_step

                self._effective_q = joint_interact + self.delta_standard_DH[3,:]

                DK = self.ur5_direct_kinematics(np.squeeze(np.asarray(self._effective_q.transpose())), True)
                DK[3:6] = rotationVector2RollPitchYaw(DK[3:6])

                Error_Position = cartesian_position[0:3] - DK[0:3] 
                Mean_Position = np.mean(np.dot(Error_Position.transpose(),Error_Position))

                Error_Rotation = computeDifference(cartesian_position_rpy[3:6],DK[3:6], True)
                Linear_Rotation_Error = Radius*Error_Rotation
                Mean_Rotation = np.mean(np.dot(Linear_Rotation_Error,Linear_Rotation_Error.transpose()))

                erro_quad = (Mean_Position + Mean_Rotation)/2

                if erro_quad > erro_quad_aux:
                    if interection_count_joint == 1:
                        direction = -1*direction
                        joint_interact[joint_count] = joint_interact[joint_count] + direction*alpha_step
                        interection_count_joint = 0
                        error_direction = erro_quad
                    else:
                        if alpha_step > min_step:
                            joint_interact[joint_count] = joint_interact[joint_count] - direction*alpha_step
                            alpha_step = alpha_step/2
                            interection_count_joint = 1
                        else:
                            joint_interact[joint_count] = joint_interact[joint_count] - direction*alpha_step
                            alpha_step = max_step
                            interection_count_joint = 1
                            joint_count -=1
                            if joint_count < 0:
                                joint_count = 5
                            interection_count +=1
                else:
                    alpha_step = alpha_step/2
                    interection_count_joint = 1
                    erro_quad_aux = erro_quad

                #if interection_count_joint == 1:
                    #if erro_quad < erro_quad_aux:
                        #erro_quad_aux = erro_quad
                        #interection_count_joint += 1
                        #joint_interact[joint_count] = joint_interact[joint_count] - direction*alpha_step
                        #alpha_step = alpha_step/2
                    #else:
                        #direction = -1*direction
                        #joint_interact[joint_count] = joint_interact[joint_count] + direction*alpha_step
                        #interection_count_joint += 1
                #else:
                    #if erro_quad < erro_quad_aux:
                        #erro_quad_aux = erro_quad
                        #interection_count_joint += 1
                        #joint_interact[joint_count] = joint_interact[joint_count] - direction*alpha_step
                        #alpha_step = alpha_step/2
                    #else:
                        #if (alpha_step < 0.000017)
                            #joint_interact[joint_count] = joint_interact[joint_count] - direction*alpha_step
                            #alpha_step = alpha_step*2
                            #joint_interact[joint_count] = joint_interact[joint_count] + direction*alpha_step
                            #alpha_step = np.pi
                            #interection_count_joint = 1
                            #joint_count -=1
                            #if joint_count < 0:
                                #joint_count = 5
                            #interection_count +=1
                        #else:        
                            #joint_interact[joint_count] = joint_interact[joint_count] - direction*alpha_step
                            #interection_count_joint = 1
                            #joint_count -=1
                            #if joint_count < 0:
                                #joint_count = 5
                            #interection_count +=1
                if interection_count > max_interection:
                    print ("[ERROR] Maximum interations reached.")
                    break

            t2 = time.clock()

            print ("[INFO] CCD Total time: "+ str(t2 - t))

            return joint_interact


        def getMeanValueVector(self, vectorArray):

            print("[INFO] Mean Value: Array, Mean, " + str(vectorArray) + ", " + str(np.mean(vectorArray, axis = 0, dtype=np.float64)))


        def controlLoopTranspose(self, desiredPose, poseActual = None):

            if (poseActual == None):
                poseActual = self.getPosition()
                poseActual[3:6] = tf.rotationVector2RollPitchYaw(poseActual[3:6])

            poseActualFK = tf.pose2Matrix(poseActual)
            desiredPoseFK = tf.pose2Matrix(desiredPose)

            poseError = desiredPose[0:3] - poseActual[0:3]

            rotationError = tf.matrix2Pose(np.dot(poseActualFK[0:3,0:3].transpose(),desiredPoseFK[0:3,0:3]), True)

            if np.any(np.isnan(rotationError)):
                np.nan_to_num(rotationError, False)

            error = np.concatenate((poseError, rotationError),axis=0)[np.newaxis]
            self.normErro = norm(poseError)

            self.errorDB.append(error)

            jacob = self.jacobian(self.getJointPosition()[np.newaxis].transpose(),(np.ones(6)*10e-3)[np.newaxis].transpose())

            # Control

            K = 0.5*np.eye(6,6)

            jointControl = np.dot(np.dot(jacob.transpose(),K),error.transpose())

            return np.squeeze(np.asarray(jointControl))

        def controlLoopPseudoInverse(self, desiredPose, poseActual = None):

            if (poseActual == None):
                poseActual = self.getPosition()
                poseActual[3:6] = tf.rotationVector2RollPitchYaw(poseActual[3:6])

            poseActualFK = tf.pose2Matrix(poseActual)
            desiredPoseFK = tf.pose2Matrix(desiredPose)

            poseError = desiredPose[0:3] - poseActual[0:3]

            rotationError = tf.matrix2Pose(np.dot(poseActualFK[0:3,0:3].transpose(),desiredPoseFK[0:3,0:3]), True)

            if np.any(np.isnan(rotationError)):
                np.nan_to_num(rotationError, False)

            error = np.concatenate((poseError, rotationError),axis=0)[np.newaxis]
            self.normErro = norm(poseError)

            self.errorDB.append(error)

            jacob = self.jacobian(self.getJointPosition()[np.newaxis].transpose(),(np.ones(6)*10e-3)[np.newaxis].transpose())

            # Control

            K = 0.5*np.eye(6,6)

            jointControl = np.dot(np.dot(pinv(jacob),K),error.transpose())

            return np.squeeze(np.asarray(jointControl))


        def controlLoopInverse(self, desiredPose, poseActual = None):

            if (poseActual == None):
                poseActual = self.getPosition()
                poseActual[3:6] = tf.rotationVector2RollPitchYaw(poseActual[3:6])

            poseActual = self.getPosition()
            poseActual[3:6] = tf.rotationVector2RollPitchYaw(poseActual[3:6])

            poseActualFK = tf.pose2Matrix(poseActual)
            desiredPoseFK = tf.pose2Matrix(desiredPose)

            poseError = desiredPose[0:3] - poseActual[0:3]

            rotationError = tf.matrix2Pose(np.dot(poseActualFK[0:3,0:3].transpose(),desiredPoseFK[0:3,0:3]), True)

            if np.any(np.isnan(rotationError)):
                np.nan_to_num(rotationError, False)

            error = np.concatenate((poseError, rotationError),axis=0)[np.newaxis]

            self.normErro = norm(poseError)
            self.errorDB.append(error)

            jacob = self.jacobian(self.getJointPosition()[np.newaxis].transpose(),(np.ones(6)*10e-6)[np.newaxis].transpose())

            # Control

            K = 0.5*np.eye(6,6)

            jointControl = np.dot(np.dot(inv(jacob),K),error.transpose())

            return np.squeeze(np.asarray(jointControl))

        def controlLoopDLS(self, desiredPose, poseActual = None, step = 0.008, jointSpeedReference = np.array([0, 0, 0, 0, 0, 0]), cartesianSpeedReference = np.array([0, 0, 0, 0, 0, 0])):

            if (poseActual == None):
                poseActual = self.getPosition()
                poseActual[3:6] = tf.rotationVector2RollPitchYaw(poseActual[3:6])

            #print(self.getPosition())
            #print(self.getJointPosition())

            poseActual = self.getPosition()
            poseActual[3:6] = tf.rotationVector2RollPitchYaw(poseActual[3:6])

            poseActualFK = tf.pose2Matrix(poseActual)
            desiredPoseFK = tf.pose2Matrix(desiredPose)

            poseError = desiredPose[0:3] - poseActual[0:3]

            rotationError = tf.matrix2Pose(np.dot(poseActualFK[0:3,0:3].transpose(),desiredPoseFK[0:3,0:3]), True)

            if np.any(np.isnan(rotationError)):
                print('[INFO][ControlLoopDLS] NaN found on control')
                np.nan_to_num(rotationError, False)

            # Error Calculation

            #Kp
            error = np.hstack((poseError, rotationError))

            #Kd
            error_D = (error - self.errorPrevious)/step
            self.error_D_DB.append(error_D)
            self.errorPrevious = error
            errorFiltered = butter_lowpass_filter(np.asarray(self.error_D_DB, dtype=np.float32), 3, 125, order=2)
            error_D = errorFiltered[errorFiltered.shape[0]-1]
            
            #Ki
            self.errorSum = self.errorSum + error
            # for i in range(0,6):
            #     if (self.errorSum[i] > 0.1):
            #         self.errorSum[i] = 0.1
            #     elif(self.errorSum[i] < -0.1):
            #         self.errorSum[i] = -0.1

            # print('Error Sum ' + str(self.errorSum))
            # if (len(self.errorDB) > 1000):
            #     self.errorSum = self.errorSum - np.asarray(self.errorDB[len(self.errorDB) - 1000], dtype=np.float32)

            #DB
            self.normErro = norm(poseError)
            self.errorDB.append(error)

            #jacob = self.jacobian(self.getJointPosition()[np.newaxis].transpose(),(np.ones(6)*10e-6)[np.newaxis].transpose(), rpy = True)
            #jacob = self.jacobian2(self.getJointPosition())
            jacob = self.jacobianAnalytic(self.getJointPosition())

            # Control

            Kp = 5*np.eye(6,6) #10 #5
            # Kp[0,0] = 1.5
            # Kp[1,1] = 1.5
            # Kp[2,2] = 1.5
            # Kp[0,3] = 0.2#0.5
            # Kp[0,4] = 0.1#0.5
            # Kp[0,5] = 0.1#0.5
            # Kp[1,3] = 0#0.5
            # Kp[1,4] = 0#0.5
            # Kp[1,5] = 0#0.5
            # Kp[2,3] = 0#0.5
            # Kp[2,4] = 0#0.5
            # Kp[2,5] = 0#0.5
            #Kp[3,3] = 16#0.5
            # Kp[3,4] = 0#0.5
            # Kp[3,5] = 0#0.5
            # Kp[4,3] = 0#0.5
            #Kp[4,4] = 16#0.5
            # Kp[4,5] = 0#0.5
            # Kp[5,3] = 0#0.5
            # Kp[5,4] = 0#0.5
            #Kp[5,5] = 16#0.5

            Kd = 2*np.eye(6,6)

            # Kd[3,3] = 0.1
            # Kd[4,4] = 0.1
            # Kd[5,5] = 0.1

            Ki = 0.25*np.eye(6,6)
            # Ki[3,3] = 0.00055 #0.55
            # Ki[4,4] = 0.00055
            # Ki[5,5] = 0.00055
            # WindupUpperLimit = np.array([0.15, 0.15, 0.15, 0.15, 0.15, 0.15])
            # WindupLowerLimit = -np.array([0.15, 0.15, 0.15, 0.15, 0.15, 0.15])

            k0 = 0.01

            w0 = 0.01

            

            KpControl = np.dot(Kp,error.transpose())
            KdControl = np.dot(Kd,error_D.transpose())
            KiControl = np.dot(Ki,self.errorSum.transpose())
            # print(KiControl)
            # print('\n')
            # for i in range(0,6):
            #     if (KiControl[i] > 0.02):
            #         KiControl[i] = 0.02
            #     elif(KiControl[i] < -0.02):
            #         KiControl[i] = -0.02
            ControlSum = KpControl + cartesianSpeedReference #+ KiControl

            t1 = time.perf_counter()
            
            w = np.sqrt(np.linalg.det(np.dot(jacob,jacob.transpose())))

            if (w < w0):
                lamb = k0*(np.power((1 - (w/w0)),2))
                print('[WARNING] Near Singularity: ' + str(w))
            else:
                lamb = 0

            lamb2 = lamb*np.eye(6,6)
            invJacob = np.dot(jacob.transpose(),inv(np.dot(jacob,jacob.transpose()) + lamb2))
            t2 = time.perf_counter()
            
            #t1 = time.perf_counter()
            #invJacob = inv(jacob)
            #t2 = time.perf_counter()


            JacobianProcessTime = t2 - t1
            self.processTimeList.append(JacobianProcessTime)
            


            self.wDB.append(w)
            #invJacob = jacob.transpose()
            jointControl = np.dot(invJacob,ControlSum) #np.dot(np.dot(np.dot(jacob.transpose(),inv(np.dot(jacob,jacob.transpose()) + lamb2)),Kp),error.transpose())

            #jointControl = jointControl + jointSpeedReference

            # for i in range(0,6):
            #     if (jointControl[i] > WindupUpperLimit[i]):
            #         self.u[i] = WindupUpperLimit[i]
            #     elif(jointControl[i] < WindupLowerLimit[i]):
            #         self.u[i] = WindupLowerLimit[i]
            #     else:
            #         self.u[i] = jointControl[i]

            # self.errorSaturation = jointControl - self.u
            # print(self.errorSaturation)

            # print('Error Sum windup' + str((np.dot(jacob,jointControl) - KpControl)/Ki[0,0]))

            # for i in range(0,6):
            #     if (jointControl[i] > 0.4):
            #         jointControl[i] = 0.4
            #     elif (jointControl[i] < -0.4):
            #         jointControl[i] = -0.4

            return np.squeeze(np.asarray(jointControl))

        def speedTransform(self, desiredSpeed, q = None, step = 0.008):

            if (q == None):
                q = self.getJointPosition()

            #jacobian = self.jacobian(self.getJointPosition()[np.newaxis].transpose(),(np.ones(6)*10e-6)[np.newaxis].transpose(), rpy = True)
            #jacobian = self.jacobian2(q)
            jacobian = self.jacobianAnalytic(q)

            jointSpeed = np.dot(inv(jacobian),desiredSpeed.transpose())

            return jointSpeed

def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y