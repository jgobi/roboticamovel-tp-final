#!/usr/bin/env python
# coding=latin1

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid

from tf.transformations import euler_from_quaternion, quaternion_from_euler

import numpy as np
from bresenham import bresenham

from time import time
from random import randint

import signal
import sys
import subprocess

if len(sys.argv) >= 2:
    pgm_path = sys.argv[1]
    if pgm_path[-1] != '/':
        pgm_path += '/'
else:
    pgm_path = './'


laserMsg = None
odomMsg = None

MAP_WIDTH = 16
MAP_HEIGHT = 16
MAP_BL_POSITION = [-MAP_WIDTH/2, -MAP_HEIGHT/2] # posição do canto inferior esquerdo do mapa
MAP_TR_POSITION = [MAP_WIDTH/2, MAP_HEIGHT/2] # posição do canto inferior esquerdo do mapa

MAP_SIDE = int(np.ceil(max(MAP_WIDTH, MAP_HEIGHT)))
GRID_RESOLUTION_MULTIPLIER = 10 # nível de detalhe

# --- INICIO NÃO EDITAR

LOG_0 = 35
LOG_ODDS_FREE = 40
LOG_ODDS_OCC  = 60

GRID_SIZE = (MAP_SIDE*GRID_RESOLUTION_MULTIPLIER, MAP_SIDE*GRID_RESOLUTION_MULTIPLIER, 1.0/GRID_RESOLUTION_MULTIPLIER) # The last one is the resolution
# --- FIM NÃO EDITAR

laserMsg = None
odomMsg = None
pose = None

# ============[ CONSTANTES DO CONTROLE P ]============ #
kp = 0.9

def run ():
    global laserMsg, odomMsg, pose, kp, MAP_BL_POSITION, MAP_TR_POSITION, GRID_SIZE

    rospy.init_node('move_example', anonymous=True)
    v_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    og_pub = rospy.Publisher('map', OccupancyGrid, queue_size=10)
    rospy.Subscriber('base_scan', LaserScan, LaserCallback)
    rospy.Subscriber('base_pose_ground_truth', Odometry, OdomCallback)
    rate = rospy.Rate(5)
    occupancy_grid = OccupancyGrid()
    occupancy_grid.header.frame_id = "map"
    occupancy_grid.header.stamp = rospy.Time.now()
    occupancy_grid.info.width = int(GRID_SIZE[0])
    occupancy_grid.info.height = int(GRID_SIZE[1])
    occupancy_grid.info.resolution = GRID_SIZE[2]

    cmd_vel = Twist()

    grid = np.full((GRID_SIZE[0],GRID_SIZE[1]), 50)
    inicio_jornada = time()
    inicio_desviando = time()

    goal = [None]

    # ===========[ DESVIO DE OBSTÁCULOS 1 ]=========== #
    desviando = False
    girando = True
    retaParaGoal = [None, None] # [a, b]
    distanciaParaGoal = None

    # ===========[ INTERRUPÇÃO DE CTRL+C ]=========== #
    def interrupt_ctrl_c(sig, frame):
        print "\n\n===============[ AGUARDE ]=============="
        print "Salvando imagens finais e saindo...\n\n"
        p=subprocess.Popen(['rosrun', 'map_server', 'map_saver', '-f', pgm_path+'mapa'])
        while p.poll() == None:
            occupancy_grid.data = grid.flatten()
            og_pub.publish(occupancy_grid)
            rate.sleep()
        p=subprocess.Popen(['rosrun', 'map_server', 'map_saver', '--occ', '16', '--free', '15', '-f', pgm_path+'mapa_threshold'])
        while p.poll() == None:
            occupancy_grid.data = grid.flatten()
            og_pub.publish(occupancy_grid)
            rate.sleep()
        sys.exit()
    signal.signal(signal.SIGINT, interrupt_ctrl_c)
    
    # ===============[ loop principal ]===============
    while not rospy.is_shutdown():
        if odomMsg == None or laserMsg == None:
            rate.sleep()
            continue
        elif goal[0] == None:
            goal = np.array(MAP_TR_POSITION) - laserMsg.range_max + 3
            
        
        # conversão de quatérnio para rollPitchYaw
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
        rollPitchYaw = euler_from_quaternion(quaternion)
        theta = rollPitchYaw[2] # Yaw

        erroU = (goal[0]-pose.position.x, goal[1]-pose.position.y) # erro em referência ao sistema universal
        theta2 = np.arctan2(erroU[1], erroU[0]) # ângulo entre o x universal e o vetor para o goal

        erroCos = np.cos(theta2-theta) # cosseno do ângulo entre o sistema do robô e o vetor para o goal
        erroSen = np.sin(theta2-theta) # seno do ângulo entre o sistema do robô e o vetor para o goal
        distanciaEuclidiana = np.sqrt(erroU[0]**2 + erroU[1]**2)
        erro = (distanciaEuclidiana*erroCos, distanciaEuclidiana*erroSen, theta2-theta) # erro em referência ao sistema do robo


        # ===========[ DESVIO DE OBSTÁCULOS 2 ]=========== #
        if desviando:
            if min(laserMsg.ranges[110:250]) <= 0.7: # se obstáculo a frente, gira pra esquerda
                if time() - inicio_desviando > 5:
                    print 'parei de girar'
                    desviando = False
                    girando = True
                cmd_vel.angular.z = 3.0
                cmd_vel.linear.x = 0
                cmd_vel.linear.y = 0
            elif min(laserMsg.ranges[0:120]) <= 0.7: # se obstáculo a direita, segue em frente
                inicio_desviando = time()
                cmd_vel.angular.z = 0
                cmd_vel.linear.x = 0.575
                cmd_vel.linear.y = 0
            else: # do contrário, vira pra direita
                if time() - inicio_desviando > 5:
                    print 'parei de girar'
                    desviando = False
                    girando = True
                cmd_vel.angular.z = -3.0
                cmd_vel.linear.x = 0.05
                cmd_vel.linear.y = 0
            
            # verificação de leave point
            y = retaParaGoal[0]*pose.position.x + retaParaGoal[1]

            if distanciaEuclidiana < distanciaParaGoal and abs(y - pose.position.y) < 0.2:
                cmd_vel.angular.z = 0
                cmd_vel.linear.x = 0
                cmd_vel.linear.y = 0
                desviando = False
                girando = True

        elif girando:
            # acerta o ângulo
            cmd_vel.angular.z = kp*erro[2]
            cmd_vel.linear.x = 0
            cmd_vel.linear.y = 0
            if erro[2] < 0.01: # se chegou ao ângulo certo, próximo estado
                girando = False
                if retaParaGoal[0] == None: # se for o início da execução do programa, salva a reta para o goal
                    retaParaGoal[0] = (goal[1]-pose.position.y) / (goal[0]-pose.position.x)
                    retaParaGoal[1] = goal[1] - (retaParaGoal[0]*goal[0])
                    distanciaParaGoal = distanciaEuclidiana

        elif min(laserMsg.ranges[110:250]) <= 0.7 and distanciaEuclidiana > 0.35: # obstáculo à frente
            # salva hit point
            distanciaParaGoal = distanciaEuclidiana
            # começa a desviar do obstáculo
            desviando = True
            inicio_desviando = time()

        else: # movimento normal
            cmd_vel.angular.z = kp*erro[2]
            cmd_vel.linear.x = kp*erro[0]
            cmd_vel.linear.y = kp*erro[1]


        # ===========[ MAPEAMENTO ]=========== #

        pose_robo = np.array([pose.position.x, pose.position.y])

        # se o robô chegar no goal, ou se ele não tiver chegado em no máximo 150 segundos, seleciona outro goal
        if np.all(np.abs(pose_robo-goal) < 0.1) or time()-inicio_jornada > 150:

            # salva mapa
            print "Salvando PGM parcial"
            subprocess.Popen(['rosrun', 'map_server', 'map_saver', '-f', pgm_path+'mapa'])

            # reinicia máquina de estados de navegação
            desviando = False
            girando = True
            retaParaGoal = [None, None] # [a, b]
            distanciaParaGoal = None

            # tenta aleatoriamente, no máximo 50 vezes, encontrar um goal na parte não varrida do mapa
            for i in range(50):
                goal = np.array([randint(MAP_BL_POSITION[0]+1, MAP_TR_POSITION[0]-1), randint(MAP_BL_POSITION[1]+1, MAP_TR_POSITION[1]-1)])
                pose_goal_grid_i = bound(int(np.floor((goal[1]-MAP_BL_POSITION[1])/GRID_SIZE[2])), 0, GRID_SIZE[1])
                pose_goal_grid_j = bound(int(np.floor((goal[0]-MAP_BL_POSITION[0])/GRID_SIZE[2])), 0, GRID_SIZE[0])
                if grid[pose_goal_grid_i, pose_goal_grid_j] == 50: # se encontrar, pronto, quebra o loop
                    break

            # reinicia contagem do tempo e printa a nova jornada
            inicio_jornada = time()
            print 'nova jornada', goal


        pose_robo_grid_i = bound(int(np.floor((pose_robo[1]-MAP_BL_POSITION[1])/GRID_SIZE[2])), 0, GRID_SIZE[1])
        pose_robo_grid_j = bound(int(np.floor((pose_robo[0]-MAP_BL_POSITION[0])/GRID_SIZE[2])), 0, GRID_SIZE[0])

        for idx, distancia in enumerate(laserMsg.ranges, start=0):

            angulo = laserMsg.angle_increment * idx + laserMsg.angle_min

            ctrl = np.array([np.cos(theta+angulo), np.sin(theta+angulo)])
            pose_obs = pose_robo + distancia * ctrl

            pose_obs_grid_i = bound(int(np.floor((pose_obs[1]-MAP_BL_POSITION[1])/GRID_SIZE[2])), 0, GRID_SIZE[1])
            pose_obs_grid_j = bound(int(np.floor((pose_obs[0]-MAP_BL_POSITION[0])/GRID_SIZE[2])), 0, GRID_SIZE[0])

            if distancia < laserMsg.range_max or grid[pose_obs_grid_i, pose_obs_grid_j] < 61:
                for cell in list(bresenham(pose_robo_grid_i, pose_robo_grid_j, pose_obs_grid_i, pose_obs_grid_j))[:-1]:
                    if grid[cell[0],cell[1]] >= (LOG_ODDS_FREE - LOG_0):
                        grid[cell[0],cell[1]] -= (LOG_ODDS_FREE - LOG_0)
                    else:
                        grid[cell[0],cell[1]] = 0

            if distancia < laserMsg.range_max:
                if grid[pose_obs_grid_i, pose_obs_grid_j] <= 100-(LOG_ODDS_FREE - LOG_0):
                    grid[pose_obs_grid_i, pose_obs_grid_j] += (LOG_ODDS_FREE - LOG_0)
                else:
                    grid[pose_obs_grid_i, pose_obs_grid_j] = 100
            
        
        # ===========[ PUBLICAÇÂO DE MENSAGENS ]=========== #
        v_pub.publish(cmd_vel)
        occupancy_grid.data = grid.flatten()
        og_pub.publish(occupancy_grid)
        rate.sleep()

def LaserCallback(msg):
    global laserMsg
    laserMsg = msg

def OdomCallback(msg):
    global odomMsg, pose
    odomMsg = msg
    pose = odomMsg.pose.pose

def bound(v, min, max):
    if v < min:
        return min
    elif v > max-1:
        return max-1
    else:
        return v


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
