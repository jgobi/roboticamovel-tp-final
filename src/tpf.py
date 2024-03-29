#!/usr/bin/env python
# coding=latin1

import rospy
from nav_msgs.msg import OccupancyGrid


import numpy as np
from bresenham import bresenham

from time import time
from random import randint

import signal
import sys
import subprocess

from utils import Ponto, bound

from robot import Robot

# Obtenção do path para salvar as imagens de saída do mapa
if len(sys.argv) >= 2:
    pgm_path = sys.argv[1]
    if pgm_path[-1] != '/':
        pgm_path += '/'
else:
    pgm_path = './'


from definitions import NUMERO_ROBOS, MAP_WIDTH, MAP_HEIGHT, MAP_BL_POSITION, MAP_TR_POSITION, GRID_RESOLUTION_MULTIPLIER, LOG_ODDS_FREE, LOG_ODDS_OCC, MAP_SIDE, GRID_SIZE

# ===========[ SALVAR IMAGEM FINAL AO CTRL+C (SIGINT) ]=========== #
def interrupt_ctrl_c (sig, frame):
    global grid, occupancy_grid
    for bot in robots:
        bot.stop_navigation()

    print "\n\n===============[ AGUARDE ]=============="
    print "Salvando imagens finais e saindo...\n\n"
    # roda map_saver
    p=subprocess.Popen(['rosrun', 'map_server', 'map_saver', '-f', pgm_path+'mapa'])
    # continua enviando mensagens enquanto o map_saver estiver ouvindo
    while p.poll() == None:
        occupancy_grid.data = grid.flatten()
        og_pub.publish(occupancy_grid)
        rate.sleep()
    
    # roda map_saver para imagem binária (valor célula <= 15 = livre, do contrário, ocupada)
    p=subprocess.Popen(['rosrun', 'map_server', 'map_saver', '--occ', '16', '--free', '15', '-f', pgm_path+'mapa_threshold'])
    # continua enviando mensagens enquanto o map_saver estiver ouvindo
    while p.poll() == None:
        occupancy_grid.data = grid.flatten()
        og_pub.publish(occupancy_grid)
        rate.sleep()
    
    # encerra a execução do programa
    sys.exit()


# ============[ INICIALIZAÇÕES 2 ]============ #
rospy.init_node('tpf')
og_pub = rospy.Publisher('map', OccupancyGrid, queue_size=10)
occupancy_grid = OccupancyGrid()
occupancy_grid.header.frame_id = "map"
occupancy_grid.header.stamp = rospy.Time.now()
occupancy_grid.info.width = int(GRID_SIZE[0])
occupancy_grid.info.height = int(GRID_SIZE[1])
occupancy_grid.info.resolution = GRID_SIZE[2]

grid = np.full((GRID_SIZE[0],GRID_SIZE[1]), 50)

print 'Pressione CTRL+C para salvar o mapa de ocupação final com e sem threshold e depois sair'

rate = rospy.Rate(5)

robots = []

# ============[ FUNÇÃO DE LOOP ]============ #
def run ():
    global goal, grid, occupancy_grid, robots

    # registra Event Listener para o SIGINT
    signal.signal(signal.SIGINT, interrupt_ctrl_c)
    salvamento_mapa = time()

    for i in range(NUMERO_ROBOS):
        print "robot_"+str(i)+"/cmd_vel", "robot_"+str(i)+"/base_scan", "robot_"+str(i)+"/base_pose_ground_truth"
        bot = Robot("robot_"+str(i)+"/cmd_vel", "robot_"+str(i)+"/base_scan", "robot_"+str(i)+"/base_pose_ground_truth")
        goal = escolhe_goal()
        bot.set_goal(goal.x, goal.y)
        bot.start_navigation()
        robots.append(bot)

    while True:
        ready = True
        print "Quaaase"
        for bot in robots:
            if not bot.step(grid):
                ready = False
            break
        if ready:
            break
        rate.sleep()
        

    print "FOI!"


    while not rospy.is_shutdown():
        print "LOOOOOP"
        for i,bot in enumerate(robots):
            if time() - bot.inicio_jornada > 150 or bot.chegou_goal:
                goal = escolhe_goal()
                bot.set_goal(goal.x, goal.y)
                bot.inicio_jornada = time()
                print "Uma nova jornada para o robô "+str(i)+" começou!", goal.x, goal.y
            bot.step(grid)
        
        if time() - salvamento_mapa > 15:
            salva_mapa()
            salvamento_mapa = time()

        # ===========[ PUBLICAÇÂO DE MENSAGENS ]=========== #
        occupancy_grid.data = grid.flatten()
        og_pub.publish(occupancy_grid)

        rate.sleep()


def escolhe_goal ():
    # tenta aleatoriamente, no máximo 20 vezes, encontrar um goal na parte não varrida do mapa
    goal = None
    for i in range(20):
        goal = Ponto(randint(MAP_BL_POSITION[0]+1, MAP_TR_POSITION[0]-1), randint(MAP_BL_POSITION[1]+1, MAP_TR_POSITION[1]-1))
        pose_goal_grid_i = bound(int(np.floor((goal.y-MAP_BL_POSITION[1])/GRID_SIZE[2])), 0, GRID_SIZE[1])
        pose_goal_grid_j = bound(int(np.floor((goal.x-MAP_BL_POSITION[0])/GRID_SIZE[2])), 0, GRID_SIZE[0])
        if grid[pose_goal_grid_i, pose_goal_grid_j] == 50: # se encontrar, quebra o loop
            break
    return goal

# salva mapa
def salva_mapa ():
    print "Salvando PGM parcial"
    subprocess.Popen(['rosrun', 'map_server', 'map_saver', '-f', pgm_path+'mapa'])

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
