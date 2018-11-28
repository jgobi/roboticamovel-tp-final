#!/usr/bin/env python
# coding=latin1
import sys
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion, quaternion_from_euler

import numpy as np
from bresenham import bresenham

from time import time
from random import random, randint

import subprocess

from utils import Ponto, bound

from srt import SRT, Point

#======================= REMOVER


# ============[ DEFINIÇÕES DO USUÁRIO - MEPEAMENTO ]============ #
MAP_WIDTH = 16 # largura do mapa
MAP_HEIGHT =16 # altura do mapa
MAP_BL_POSITION = [-MAP_WIDTH/2, -MAP_HEIGHT/2] # posição do canto inferior esquerdo do mapa
MAP_TR_POSITION = [MAP_WIDTH/2, MAP_HEIGHT/2] # posição do canto inferior esquerdo do mapa

GRID_RESOLUTION_MULTIPLIER = 10 # nível de detalhe do mapa. quanto maior, mais subdividido o grid


LOG_ODDS_FREE = 5 # constante l_{free} - l_0 = 40 - 35
LOG_ODDS_OCC  = 25 # constante l_{occ} - l_0 = 60 - 35


# ============[ INICIALIZAÇÕES 1 ]============ #
MAP_SIDE = int(np.ceil(max(MAP_WIDTH, MAP_HEIGHT))) # o mapa precisa ser quadrado para o algoritmo funcionar bem
GRID_SIZE = (MAP_SIDE*GRID_RESOLUTION_MULTIPLIER, MAP_SIDE*GRID_RESOLUTION_MULTIPLIER, 1.0/GRID_RESOLUTION_MULTIPLIER) # The last one is the resolution


#======================== FIM REMOVER


# ============[ DEFINIÇÕES DO USUÁRIO - NAVEGAÇÃO ]============ #
kp = 0.4 # gain proporcional

DMIN = 2.8

# ============[ CLASSE ]=============
class Robot:
    def __init__ (self, cmd_vel_topic, base_scan_topic, odom_topic):

        self.goal = Ponto(0, 0)
        self.laser_msg = None
        self.odom_msg = None
        self.pose = None
        self.theta = None

        self.navegando = False

        self.v_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        rospy.Subscriber(base_scan_topic, LaserScan, self.laser_callback)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback)

        self.cmd_vel = Twist()


        # ============[ navegacao ]============ #
        self.inicio_desviando = time()
        self.girando = True    # variável de estado
        self.retaParaGoal = [None, None] # [a, b]
        self.distanciaParaGoal = None

        self.inicio_jornada = time()

        self.chegou_goal = True

        self.T = SRT()
        self.cur_node = None


    # ===============[ loop principal ]===============
    def do_movement(self): # navega até um goal, definindo se chegou nele ou não

        self.theta = self.current_theta()

        # ===========[ NAVEGAÇÃO ]=========== #
        erroU = (self.goal.x-self.pose.position.x, self.goal.y-self.pose.position.y) # erro em referência ao sistema universal
        theta2 = np.arctan2(erroU[1], erroU[0]) # ângulo entre o x universal e o vetor para o goal

        erroCos = np.cos(theta2-self.theta) # cosseno do ângulo entre o sistema do robô e o vetor para o goal
        erroSen = np.sin(theta2-self.theta) # seno do ângulo entre o sistema do robô e o vetor para o goal
        distanciaEuclidiana = np.sqrt(erroU[0]**2 + erroU[1]**2)
        erro = (distanciaEuclidiana*erroCos, distanciaEuclidiana*erroSen, theta2-self.theta) # erro em referência ao sistema do robo

        pode_mapear = True
        if self.girando: # estado de navegação
            # ----------[ GIRA PARA FICAR EM DIREÇÃO AO GOAL ]---------- #
            # acerta o ângulo
            self.cmd_vel.angular.z = kp*erro[2]
            self.cmd_vel.linear.x = 0
            self.cmd_vel.linear.y = 0
            if erro[2] < 0.01: # se chegou ao ângulo certo, próximo estado
                self.girando = False
                if self.retaParaGoal[0] == None: # se for o início da execução do programa, salva a reta para o goal
                    if (self.goal.x-self.pose.position.x) == 0:
                        self.chegou_goal = True # evita divisão por 0
                    else:
                        self.retaParaGoal[0] = (self.goal.y-self.pose.position.y) / (self.goal.x-self.pose.position.x)
                        self.retaParaGoal[1] = self.goal.y - (self.retaParaGoal[0]*self.goal.x)
                        self.distanciaParaGoal = distanciaEuclidiana

        else: # movimento normal
            self.cmd_vel.angular.z = 0
            self.cmd_vel.linear.x = kp*erro[0]
            self.cmd_vel.linear.y = kp*erro[1]
            print erro[1]

            pode_mapear = True

        self.v_pub.publish(self.cmd_vel)

        if erro[0] < 0.1 and erro[0] < 0.1:
            self.chegou_goal = True
        return pode_mapear


    # Exige um laser de 360 graus
    def do_navigation(self): # navega, retornando se o robô está pronto para se mexer (True), se o mapeamento pode ser feito (True) e se o mapeamento acabou (True)
        c_goal = None
        if self.odom_msg == None or self.laser_msg == None:
            return False, False, False
        elif not self.chegou_goal:
            pode_mapear = self.do_movement()
            return True, pode_mapear, False
        else:
            self.theta = self.current_theta()
            self.cur_node, new_node = self.T.add_node(Point(self.pose.position.x, self.pose.position.y), self.srt_obtem_raios_maximos(), self.cur_node)
            # print Point(self.pose.position.x, self.pose.position.y), self.srt_obtem_raios_maximos(), self.cur_node
            i = 1
            while True:
                theta_rand = randint(0, 359) + modulo360(np.rad2deg(self.theta))
                theta_rand_rad = np.deg2rad(theta_rand)
                r=random()
                alpha = r-0.2 if r > 0.8 else r
                raio = new_node.data.get_radius_from_degree(theta_rand) * alpha
                c_goal = Point(self.pose.position.x + raio*np.cos(theta_rand_rad), self.pose.position.y + raio*np.sin(theta_rand_rad))
                if raio > DMIN and not self.T.is_inside_tree(c_goal, self.cur_node):
                    break
                elif i < 15000: # Imax
                    i += 1
                    continue
                else:
                    c_goal = self.T.get_parent(self.cur_node)
                    if c_goal is None:
                        return True, True, True
                    else:
                        c_goal = c_goal.data.centro
                        break
            self.set_goal(c_goal.x, c_goal.y)
        return True, True, False

    def srt_obtem_raios_maximos(self):
        menores_raios = []
        angulos_por_setor = 360/12
        menores_raios = np.full(12, self.laser_msg.range_max)
        for idx, distancia in enumerate(self.laser_msg.ranges, start=0):
            angulo = modulo360(np.rad2deg(self.laser_msg.angle_increment * idx + self.laser_msg.angle_min + self.theta))
            menores_raios[int(angulo/angulos_por_setor)%12] = min(menores_raios[int(angulo/angulos_por_setor)%12], distancia)
        return menores_raios
    
    def do_mapping(self, grid):
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
            ctrl = np.array([np.cos(self.theta+angulo), np.sin(self.theta+angulo)])
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
        if not self.navegando:
            self.start_navigation()

    def start_navigation (self):
        self.navegando = True

    def stop_navigation (self):
        self.navegando = False

    def current_theta (self):
        return quaternion_to_theta(self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w)


def quaternion_to_theta (x, y, z, w):
    # conversão de quatérnio para rollPitchYaw
    quaternion = (x,y,z,w)
    rollPitchYaw = euler_from_quaternion(quaternion)
    return rollPitchYaw[2] # Yaw

def modulo360 (angulo):
    base = 360
    return ((angulo % base) + base) % base