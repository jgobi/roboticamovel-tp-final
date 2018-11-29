#!/usr/bin/env python
# coding=latin1

import numpy as np
from bresenham import bresenham

from util import quaternion_to_theta, bound, Pose

import rospy
from nav_msgs.msg import OccupancyGrid

class Map:
    def __init__(self, side, initial_value, log_odds_free, log_odds_occ, resolution_multiplier):
        self.grid = np.full((side, side), initial_value)

        self.MAP_WIDTH = side # largura do mapa
        self.MAP_HEIGHT = self.MAP_WIDTH # altura do mapa
        self.MAP_BL_POSITION = [-self.MAP_WIDTH/2, -self.MAP_HEIGHT/2] # posição do canto inferior esquerdo do mapa
        self.MAP_TR_POSITION = [self.MAP_WIDTH/2, self.MAP_HEIGHT/2] # posição do canto inferior esquerdo do mapa

        self.GRID_RESOLUTION_MULTIPLIER = resolution_multiplier # nível de detalhe do mapa. quanto maior, mais subdividido o grid

        self.INITIAL_CELL_VALUE = initial_value
        self.LOG_ODDS_FREE = log_odds_free # constante l_{free} - l_0 = 40 - 35
        self.LOG_ODDS_OCC  = log_odds_occ # constante l_{occ} - l_0 = 60 - 35

        self.MAP_SIDE = int(np.ceil(max(self.MAP_WIDTH, self.MAP_HEIGHT))) # o mapa precisa ser quadrado para o algoritmo funcionar bem
        self.GRID_SIZE = (self.MAP_SIDE*self.GRID_RESOLUTION_MULTIPLIER, self.MAP_SIDE*self.GRID_RESOLUTION_MULTIPLIER, 1.0/self.GRID_RESOLUTION_MULTIPLIER) # The last one is the resolution

        self.og_pub = rospy.Publisher('map', OccupancyGrid, queue_size=10)
        self.occupancy_grid = OccupancyGrid()
        self.occupancy_grid.header.frame_id = "map"
        self.occupancy_grid.header.stamp = rospy.Time.now()
        self.occupancy_grid.info.width = int(self.GRID_SIZE[0])
        self.occupancy_grid.info.height = int(self.GRID_SIZE[1])
        self.occupancy_grid.info.resolution = self.GRID_SIZE[2]


    def do_mapping(self, pose, laser_msg):
        # self.pose do robo na grid
        pose_robo_grid_i = bound(int(np.floor((pose.y-self.MAP_BL_POSITION[1])/self.GRID_SIZE[2])), 0, self.GRID_SIZE[1])
        pose_robo_grid_j = bound(int(np.floor((pose.x-self.MAP_BL_POSITION[0])/self.GRID_SIZE[2])), 0, self.GRID_SIZE[0])

        # itera nos ranges do laser
        for idx, distancia in enumerate(laser_msg.ranges, start=0):

            angulo = laser_msg.angle_increment * idx + laser_msg.angle_min

            # calculo da posição do obstáculo
            ctrl = np.array([np.cos(pose.theta+angulo), np.sin(pose.theta+angulo)])
            pose_obs = pose.get_xy_array() + distancia * ctrl

            # calculo da posição do obstáculo na grid
            pose_obs_grid_i = bound(int(np.floor((pose_obs[1]-self.MAP_BL_POSITION[1])/self.GRID_SIZE[2])), 0, self.GRID_SIZE[1])
            pose_obs_grid_j = bound(int(np.floor((pose_obs[0]-self.MAP_BL_POSITION[0])/self.GRID_SIZE[2])), 0, self.GRID_SIZE[0])

            """
            Para agilizar o mapeamento, mesmo se o sensor não detectar um obstáculo, suas informações são usadas
            para formar um mapa. Nesse caso, considera-se que se ele não encontrou um obstáculo no seu range máximo,
            então todas as células estão livres.
            Porém, para evitar que isso atrapalhe em alguma célula já dada como ocupada, o acima só é feito caso
            a célula dada como "obstáculo" (ou seja, a célula coincidente com a posição máxima alcançada pelo laser)
            não esteja com probabilidade alta de ser ocupada. Aqui, foi usado o número máximo 61.
            """

            if distancia < laser_msg.range_max or self.grid[pose_obs_grid_i, pose_obs_grid_j] < 61:
                # para cada célula da linha de bresenham que liga a self.pose do robô no grid à self.pose obstáculo no grid
                # decremente log_odds_free
                for cell in list(bresenham(pose_robo_grid_i, pose_robo_grid_j, pose_obs_grid_i, pose_obs_grid_j))[:-1]:
                    if self.grid[cell[0],cell[1]] >= self.LOG_ODDS_FREE: # evita que o número fique negativo
                        self.grid[cell[0],cell[1]] -= self.LOG_ODDS_FREE
                    else:
                        self.grid[cell[0],cell[1]] = 0

            # se aplicável, incremente log_odds_occ na célula do grid equivalente à self.pose do obstáculo
            if distancia < laser_msg.range_max:
                if self.grid[pose_obs_grid_i, pose_obs_grid_j] <= 100-self.LOG_ODDS_OCC: # evita que o número passe de 100
                    self.grid[pose_obs_grid_i, pose_obs_grid_j] += self.LOG_ODDS_OCC
                else:
                    self.grid[pose_obs_grid_i, pose_obs_grid_j] = 100

    def flatten(self):
        return self.grid.flatten()

    def publish(self):
        self.occupancy_grid.header.stamp = rospy.Time.now()
        self.occupancy_grid.data = self.flatten()
        self.og_pub.publish(self.occupancy_grid)
