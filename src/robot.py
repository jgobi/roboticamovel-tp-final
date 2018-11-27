#!/usr/bin/env python
# coding=latin1

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion, quaternion_from_euler

import numpy as np
from bresenham import bresenham

from time import time
from random import randint

import subprocess

from utils import Ponto, bound


from definitions import MAP_WIDTH, MAP_HEIGHT, MAP_BL_POSITION, MAP_TR_POSITION, GRID_RESOLUTION_MULTIPLIER, LOG_ODDS_FREE, LOG_ODDS_OCC, MAP_SIDE, GRID_SIZE

# ============[ DEFINIÇÕES DO USUÁRIO - NAVEGAÇÃO ]============ #
kp = 0.8 # gain proporcional

# ============[ CLASSE ]=============
class Robot:
    def __init__ (self, cmd_vel_topic, base_scan_topic, odom_topic):

        self.goal = Ponto(0, 0)
        self.laser_msg = None
        self.odom_msg = None
        self.pose = None

        self.navegando = False

        self.v_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        rospy.Subscriber(base_scan_topic, LaserScan, self.laser_callback)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback)

        self.cmd_vel = Twist()


        # ============[ navegacao ]============ #
        self.inicio_desviando = time()
        self.desviando = False # variável de estado
        self.girando = True    # variável de estado
        self.retaParaGoal = [None, None] # [a, b]
        self.distanciaParaGoal = None

        self.inicio_jornada = time()

        self.chegou_goal = True


    # ===============[ loop principal ]===============
    def step(self, grid):
        if self.odom_msg == None or self.laser_msg == None or not self.navegando:
            return False
        
        # conversão de quatérnio para rollPitchYaw
        quaternion = (
            self.pose.orientation.x,
            self.pose.orientation.y,
            self.pose.orientation.z,
            self.pose.orientation.w)
        rollPitchYaw = euler_from_quaternion(quaternion)
        theta = rollPitchYaw[2] # Yaw


        # ===========[ NAVEGAÇÃO ]=========== #
        erroU = (self.goal.x-self.pose.position.x, self.goal.y-self.pose.position.y) # erro em referência ao sistema universal
        theta2 = np.arctan2(erroU[1], erroU[0]) # ângulo entre o x universal e o vetor para o goal

        erroCos = np.cos(theta2-theta) # cosseno do ângulo entre o sistema do robô e o vetor para o goal
        erroSen = np.sin(theta2-theta) # seno do ângulo entre o sistema do robô e o vetor para o goal
        distanciaEuclidiana = np.sqrt(erroU[0]**2 + erroU[1]**2)
        erro = (distanciaEuclidiana*erroCos, distanciaEuclidiana*erroSen, theta2-theta) # erro em referência ao sistema do robo

        if self.desviando: # estado de navegação
            if time() - self.inicio_desviando > 20: # se robô está girando sem controle a mais de 5 segundos
                # reseta a máquina de estados
                self.desviando = False
                self.girando = True
            # ----------[ DESVIO DE OBSTÁCULOS ]---------- #
            if min(self.laser_msg.ranges[110:250]) <= 0.7: # se obstáculo a frente, gira pra esquerda
                self.cmd_vel.angular.z = 3.0
                self.cmd_vel.linear.x = 0
                self.cmd_vel.linear.y = 0
            elif min(self.laser_msg.ranges[0:120]) <= 0.7: # se obstáculo a direita, segue em frente
                self.inicio_desviando = time()
                self.cmd_vel.angular.z = 0
                self.cmd_vel.linear.x = 0.575
                self.cmd_vel.linear.y = 0
            else: # do contrário, vira pra direita
                self.cmd_vel.angular.z = -3.0
                self.cmd_vel.linear.x = 0.05
                self.cmd_vel.linear.y = 0
            
            # verificação de leave point
            y = self.retaParaGoal[0]*self.pose.position.x + self.retaParaGoal[1]

            if distanciaEuclidiana < self.distanciaParaGoal and abs(y - self.pose.position.y) < 0.2:
                self.cmd_vel.angular.z = 0
                self.cmd_vel.linear.x = 0
                self.cmd_vel.linear.y = 0
                self.desviando = False
                self.girando = True

        elif self.girando: # estado de navegação
            # ----------[ GIRA PARA FICAR EM DIREÇÃO AO GOAL ]---------- #
            # acerta o ângulo
            self.cmd_vel.angular.z = kp*erro[2]
            self.cmd_vel.linear.x = 0
            self.cmd_vel.linear.y = 0
            if erro[2] < 0.01: # se chegou ao ângulo certo, próximo estado
                self.girando = False
                if self.retaParaGoal[0] == None: # se for o início da execução do programa, salva a reta para o goal
                    if (self.goal.x-self.pose.position.x) != 0:
                        self.retaParaGoal[0] = (self.goal.y-self.pose.position.y) / (self.goal.x-self.pose.position.x)
                        self.retaParaGoal[1] = self.goal.y - (self.retaParaGoal[0]*self.goal.x)
                        self.distanciaParaGoal = distanciaEuclidiana
                    else:
                        self.chegou_goal = True

        elif min(self.laser_msg.ranges[110:250]) <= 0.7 and distanciaEuclidiana > 0.35: # obstáculo à frente
            # salva hit point
            self.distanciaParaGoal = distanciaEuclidiana
            # começa a desviar do obstáculo
            self.desviando = True
            self.inicio_desviando = time()

        else: # movimento normal
            self.cmd_vel.angular.z = kp*erro[2]
            self.cmd_vel.linear.x = kp*erro[0]
            self.cmd_vel.linear.y = kp*erro[1]

        self.v_pub.publish(self.cmd_vel)

        # ===========[ MAPEAMENTO ]=========== #
        self.pose_robo = np.array([self.pose.position.x, self.pose.position.y])

        if np.all(np.abs(self.pose_robo-self.goal.to_array()) < 0.1):
            self.chegou_goal = True

        # self.pose do robo na grid
        self.pose_robo_grid_i = bound(int(np.floor((self.pose_robo[1]-MAP_BL_POSITION[1])/GRID_SIZE[2])), 0, GRID_SIZE[1])
        self.pose_robo_grid_j = bound(int(np.floor((self.pose_robo[0]-MAP_BL_POSITION[0])/GRID_SIZE[2])), 0, GRID_SIZE[0])

        # itera nos ranges do laser
        for idx, distancia in enumerate(self.laser_msg.ranges, start=0):

            angulo = self.laser_msg.angle_increment * idx + self.laser_msg.angle_min

            # calculo da posição do obstáculo
            ctrl = np.array([np.cos(theta+angulo), np.sin(theta+angulo)])
            self.pose_obs = self.pose_robo + distancia * ctrl

            # calculo da posição do obstáculo na grid
            self.pose_obs_grid_i = bound(int(np.floor((self.pose_obs[1]-MAP_BL_POSITION[1])/GRID_SIZE[2])), 0, GRID_SIZE[1])
            self.pose_obs_grid_j = bound(int(np.floor((self.pose_obs[0]-MAP_BL_POSITION[0])/GRID_SIZE[2])), 0, GRID_SIZE[0])

            """
            Para agilizar o mapeamento, mesmo se o sensor não detectar um obstáculo, suas informações são usadas
            para formar um mapa. Nesse caso, considera-se que se ele não encontrou um obstáculo no seu range máximo,
            então todas as células estão livres.
            Porém, para evitar que isso atrapalhe em alguma célula já dada como ocupada, o acima só é feito caso
            a célula dada como "obstáculo" (ou seja, a célula coincidente com a posição máxima alcançada pelo laser)
            não esteja com probabilidade alta de ser ocupada. Aqui, foi usado o número máximo 61.
            """

            if distancia < self.laser_msg.range_max or grid[self.pose_obs_grid_i, self.pose_obs_grid_j] < 61:
                # para cada célula da linha de bresenham que liga a self.pose do robô no grid à self.pose obstáculo no grid
                # decremente log_odds_free
                for cell in list(bresenham(self.pose_robo_grid_i, self.pose_robo_grid_j, self.pose_obs_grid_i, self.pose_obs_grid_j))[:-1]:
                    if grid[cell[0],cell[1]] >= LOG_ODDS_FREE: # evita que o número fique negativo
                        grid[cell[0],cell[1]] -= LOG_ODDS_FREE
                    else:
                        grid[cell[0],cell[1]] = 0

            # se aplicável, incremente log_odds_occ na célula do grid equivalente à self.pose do obstáculo
            if distancia < self.laser_msg.range_max:
                if grid[self.pose_obs_grid_i, self.pose_obs_grid_j] <= 100-LOG_ODDS_OCC: # evita que o número passe de 100
                    grid[self.pose_obs_grid_i, self.pose_obs_grid_j] += LOG_ODDS_OCC
                else:
                    grid[self.pose_obs_grid_i, self.pose_obs_grid_j] = 100
        return True

    def laser_callback (self, msg):
        self.laser_msg = msg

    def odom_callback (self, msg):
        self.odom_msg = msg
        self.pose = self.odom_msg.pose.pose
    
    def set_goal (self, x, y):
        # define goal
        self.goal = Ponto(x,y)

        # reinicia estados de navegação
        self.desviando = False
        self.girando = True
        self.retaParaGoal = [None, None] # [a, b]
        self.distanciaParaGoal = None
        self.chegou_goal = False

    def start_navigation (self):
        self.navegando = True

    def stop_navigation (self):
        self.navegando = False
