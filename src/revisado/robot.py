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

TAMANHO_ROBO = .33
FOLGA_ROBO = TAMANHO_ROBO + 0.14 # 0.14 é uma margem de erro escolhida experimentalmente
SRT_SECTOR_QTD = 12

class Snapshot:
    def __init__(self, pose=None, laser_msg=None):
        self.pose = pose
        self.laser_msg = laser_msg


# ============[ CLASSE ]=============
class Robot:
    def __init__ (self, cmd_vel_topic, base_scan_topic, odom_topic, kp):

        self.KP = kp
        self.DMIN = 0 # este valor é atualizado quando a informação do laser fica disponível

        self.goal = Point(0, 0) # este valor é atualizado quando a informação do laser fica disponível

        self.laser_msg = None
        self.odom_msg = None
        self.pose = Pose()

        self.v_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        rospy.Subscriber(base_scan_topic, LaserScan, self.laser_callback)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback)

        self.cmd_vel = Twist()

        self.sensors_ready = np.array([False, False]) # [Odom, Laser]

        # ============[ navegacao ]============ #
        self.chegou_goal = True
        self.retornar_pai = False
        self.done = False

        self.T = SRT(SRT_SECTOR_QTD)
        self.cur_node = None


    # ===============[ loop principal ]===============
    def _do_movement(self): # navega até um goal, definindo se chegou nele ou não
        # ===========[ NAVEGAÇÃO ]=========== #
        erroU = (self.goal.x-self.pose.x, self.goal.y-self.pose.y) # erro em referência ao sistema universal
        theta2 = np.arctan2(erroU[1], erroU[0]) # ângulo entre o x universal e o vetor para o goal

        erroTheta = theta2-self.pose.theta
        erroCos = np.cos(erroTheta) # cosseno do ângulo entre o sistema do robô e o vetor para o goal
        erroSen = np.sin(erroTheta) # seno do ângulo entre o sistema do robô e o vetor para o goal
        distanciaEuclidiana = np.sqrt(erroU[0]**2 + erroU[1]**2)
        erro = (distanciaEuclidiana*erroCos, distanciaEuclidiana*erroSen) # erro em referência ao sistema do robo

        obstaculo_proximo = np.min(self.laser_msg.ranges) < FOLGA_ROBO/2.0

        self.cmd_vel.angular.z = 0
        self.cmd_vel.linear.x = self.KP*erro[0]
        self.cmd_vel.linear.y = self.KP*erro[1]

        self.v_pub.publish(self.cmd_vel)

        if distanciaEuclidiana < 0.1:
            self.chegou_goal = True
        elif obstaculo_proximo:
            self.retornar_pai = True


    def _get_cur_node_parent_location(self):
        goal = self.T.get_parent(self.cur_node)
        if goal is not None:
            goal = goal.data.centro
        return goal

    def _choose_goal(self):
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
            c_goal = self._get_cur_node_parent_location()
        
        return c_goal

    # Exige um laser de 360 graus
    def do_navigation(self): # navega
        if self.odom_msg == None or self.laser_msg == None:
            return
        else:
            self.DMIN = self.laser_msg.range_max / 2
            if self.retornar_pai:
                self.retornar_pai = False
                parent = self._get_cur_node_parent_location()
                self.set_goal(parent)
            elif not self.chegou_goal:
                self._do_movement()
            else:
                goal = self._choose_goal()
                self.set_goal(goal)

    def srt_obtem_raios_maximos(self):
        menores_raios = []
        angulos_por_setor = 360/SRT_SECTOR_QTD
        menores_raios = np.full(SRT_SECTOR_QTD, self.laser_msg.range_max)
        for idx, distancia in enumerate(self.laser_msg.ranges, start=0):
            angulo = modulo360(np.rad2deg(self.laser_msg.angle_increment * idx + self.laser_msg.angle_min + self.pose.theta))
            menores_raios[int(angulo/angulos_por_setor)%SRT_SECTOR_QTD] = min(menores_raios[int(angulo/angulos_por_setor)%SRT_SECTOR_QTD], distancia-FOLGA_ROBO)
        return menores_raios
    
    def laser_callback (self, msg):
        self.sensors_ready[1] = True
        self.laser_msg = msg

    def odom_callback (self, msg):
        self.sensors_ready[0] = True
        self.odom_msg = msg
        self.pose = Pose(odom_msg=self.odom_msg)

    def take_sensor_snapshot (self):
        return Snapshot(self.pose, self.laser_msg)

    def set_goal (self, goal):
        if goal is None:
            self.done = True
        else:
            # define goal
            self.goal = goal
            # reinicia estados de navegação
            self.chegou_goal = False

    def is_ready (self):
        return self.sensors_ready.all() == True
    
    def is_done (self):
        return self.done