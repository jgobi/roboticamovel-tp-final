#!/usr/bin/env python
# coding=latin1

import numpy as np
from srt import SRT
from util import Pose, Point, modulo360, quaternion_to_theta
from random import random, randint
from time import time

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

TAMANHO_ROBO = .35
SRT_SECTOR_QTD = 12

class Snapshot:
    def __init__(self, pose=None, laser_msg=None):
        self.pose = pose
        self.laser_msg = laser_msg


# ============[ CLASSE ]=============
class Robot:
    def __init__ (self, cmd_vel_topic, base_scan_topic, odom_topic, kp):

        self.KP = kp
        self.DMIN = 0 # TODO: atualizar para metade do range do laser

        self.sensor_snapshot = Snapshot()

        self.goal = Point(0, 0) # TODO: atualizar para metade do range do laser

        self.laser_msg = None
        self.odom_msg = None
        self.pose = Pose()

        self.v_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        rospy.Subscriber(base_scan_topic, LaserScan, self.laser_callback)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback)

        self.cmd_vel = Twist()


        # ============[ navegacao ]============ #
        self.girando = True    # variável de estado
        self.chegou_goal = True

        self.T = SRT(SRT_SECTOR_QTD)
        self.cur_node = None


    # ===============[ loop principal ]===============
    def do_movement(self): # navega até um goal, definindo se chegou nele ou não
        # ===========[ NAVEGAÇÃO ]=========== #
        erroU = (self.goal.x-self.pose.x, self.goal.y-self.pose.y) # erro em referência ao sistema universal
        theta2 = np.arctan2(erroU[1], erroU[0]) # ângulo entre o x universal e o vetor para o goal

        erroCos = np.cos(theta2-self.pose.theta) # cosseno do ângulo entre o sistema do robô e o vetor para o goal
        erroSen = np.sin(theta2-self.pose.theta) # seno do ângulo entre o sistema do robô e o vetor para o goal
        distanciaEuclidiana = np.sqrt(erroU[0]**2 + erroU[1]**2)
        erro = (distanciaEuclidiana*erroCos, distanciaEuclidiana*erroSen, theta2-self.pose.theta) # erro em referência ao sistema do robo

        pode_mapear = True
        if self.girando: # estado de navegação
            self.cmd_vel.angular.z = self.KP*erro[2]
            self.cmd_vel.linear.x = 0
            self.cmd_vel.linear.y = 0
            if erro[2] < 0.01: # se chegou ao ângulo certo, próximo estado
                self.girando = False

        else: # movimento normal
            self.cmd_vel.angular.z = 0
            self.cmd_vel.linear.x = self.KP*distanciaEuclidiana # self.KP*erro[0]
            self.cmd_vel.linear.y = 0 # self.KP*erro[1]

            pode_mapear = True

        self.v_pub.publish(self.cmd_vel)

        if distanciaEuclidiana < 0.1: #erro[0] < 0.1 and erro[0] < 0.1:
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
            self.cur_node, new_node = self.T.add_node(Point(self.pose.x, self.pose.y), self.srt_obtem_raios_maximos(), self.cur_node)
            
            goal_valid = False
            for i in range(150):
                theta_rand = randint(0, 359) + modulo360(np.rad2deg(self.pose.theta))
                theta_rand_rad = np.deg2rad(theta_rand)
                alpha = random()
                raio = new_node.data.get_radius_from_degree(theta_rand) * alpha
                c_goal = Point(self.pose.x + raio*np.cos(theta_rand_rad), self.pose.y + raio*np.sin(theta_rand_rad))
                if raio > self.DMIN and not self.T.is_inside_tree(c_goal, self.cur_node):
                    goal_valid = True
                    break
            
            if not goal_valid:
                c_goal = self.T.get_parent(self.cur_node)
                if c_goal is None:
                    return True, True, True
                else:
                    c_goal = c_goal.data.centro
            
            self.set_goal(c_goal.x, c_goal.y)
            return True, True, False

    def srt_obtem_raios_maximos(self):
        menores_raios = []
        angulos_por_setor = 360/SRT_SECTOR_QTD
        menores_raios = np.full(SRT_SECTOR_QTD, self.laser_msg.range_max)
        for idx, distancia in enumerate(self.laser_msg.ranges, start=0):
            angulo = modulo360(np.rad2deg(self.laser_msg.angle_increment * idx + self.laser_msg.angle_min + self.pose.theta))
            menores_raios[int(angulo/angulos_por_setor)%SRT_SECTOR_QTD] = min(menores_raios[int(angulo/angulos_por_setor)%SRT_SECTOR_QTD], distancia-TAMANHO_ROBO)
        return menores_raios
    
    def laser_callback (self, msg):
        self.laser_msg = msg

    def odom_callback (self, msg):
        self.odom_msg = msg
        self.pose = Pose(odom_msg=self.odom_msg)

    def take_snapshot(self):
        self.sensor_snapshot.pose = self.pose
        self.sensor_snapshot.laser_msg = self.laser_msg
    
    def set_goal (self, x, y):
        # define goal
        self.goal = Point(x,y)
        # reinicia estados de navegação
        self.girando = True
        self.chegou_goal = False