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

# Obtenção do path para salvar as imagens de saída do mapa
if len(sys.argv) >= 2:
    pgm_path = sys.argv[1]
    if pgm_path[-1] != '/':
        pgm_path += '/'
else:
    pgm_path = './'


# ============[ DEFINIÇÕES DO USUÁRIO - NAVEGAÇÃO ]============ #
kp = 0.9 # gain proporcional


# ============[ DEFINIÇÕES DO USUÁRIO - MEPEAMENTO ]============ #
MAP_WIDTH = 16 # largura do mapa
MAP_HEIGHT = 16 # altura do mapa
MAP_BL_POSITION = [-MAP_WIDTH/2, -MAP_HEIGHT/2] # posição do canto inferior esquerdo do mapa
MAP_TR_POSITION = [MAP_WIDTH/2, MAP_HEIGHT/2] # posição do canto inferior esquerdo do mapa

GRID_RESOLUTION_MULTIPLIER = 10 # nível de detalhe do mapa. quanto maior, mais subdividido o grid


LOG_ODDS_FREE = 5 # constante l_{free} - l_0 = 40 - 35
LOG_ODDS_OCC  = 25 # constante l_{occ} - l_0 = 60 - 35


# ============[ INICIALIZAÇÕES 1 ]============ #
MAP_SIDE = int(np.ceil(max(MAP_WIDTH, MAP_HEIGHT))) # o mapa precisa ser quadrado para o algoritmo funcionar bem
GRID_SIZE = (MAP_SIDE*GRID_RESOLUTION_MULTIPLIER, MAP_SIDE*GRID_RESOLUTION_MULTIPLIER, 1.0/GRID_RESOLUTION_MULTIPLIER) # The last one is the resolution

laserMsg = None
odomMsg = None
pose = None



# ============[ INICIO FUNÇÕES AUXILIARES ]============ #

def LaserCallback(msg):
    global laserMsg
    laserMsg = msg

def OdomCallback(msg):
    global odomMsg, pose
    odomMsg = msg
    pose = odomMsg.pose.pose

"""
Recebe um valor e um limite inferior e superior e retorna o próprio valor
ou um dos limites, caso o valor não esteja entre eles.
O limite é inclusivo no mínimo e exclusivo no máximo.
"""
def bound(v, min, max):
    if v < min:
        return min
    elif v > max-1:
        return max-1
    else:
        return v

# ===========[ SALVAR IMAGEM FINAL AO CTRL+C (SIGINT) ]=========== #
def interrupt_ctrl_c (sig, frame):
    global grid, occupancy_grid
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



# ============[ FIM FUNÇÕES AUXILIARES ]============ #


# ============[ INICIALIZAÇÕES 2 ]============ #
rospy.init_node('tp2', anonymous=True)
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

goal = np.array(MAP_TR_POSITION)
print 'Pressione CTRL+C para salvar o mapa de ocupação final com e sem threshold e depois sair'



# ============[ FUNÇÃO DE LOOP ]============ #
def run ():
    global goal, grid, cmd_vel, occupancy_grid


    # Event listener para o SIGINT
    signal.signal(signal.SIGINT, interrupt_ctrl_c)

    # ============[ mapeamento ]============ #
    inicio_jornada = time()


    # ============[ navegacao ]============ #
    inicio_desviando = time()
    desviando = False # variável de estado
    girando = True    # variável de estado
    retaParaGoal = [None, None] # [a, b]
    distanciaParaGoal = None


    # ===============[ loop principal ]===============
    while not rospy.is_shutdown():
        if odomMsg == None or laserMsg == None:
            rate.sleep()
            continue            
        
        # conversão de quatérnio para rollPitchYaw
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
        rollPitchYaw = euler_from_quaternion(quaternion)
        theta = rollPitchYaw[2] # Yaw


        # ===========[ NAVEGAÇÃO ]=========== #
        erroU = (goal[0]-pose.position.x, goal[1]-pose.position.y) # erro em referência ao sistema universal
        theta2 = np.arctan2(erroU[1], erroU[0]) # ângulo entre o x universal e o vetor para o goal

        erroCos = np.cos(theta2-theta) # cosseno do ângulo entre o sistema do robô e o vetor para o goal
        erroSen = np.sin(theta2-theta) # seno do ângulo entre o sistema do robô e o vetor para o goal
        distanciaEuclidiana = np.sqrt(erroU[0]**2 + erroU[1]**2)
        erro = (distanciaEuclidiana*erroCos, distanciaEuclidiana*erroSen, theta2-theta) # erro em referência ao sistema do robo

        if desviando: # estado de navegação
            # ----------[ DESVIO DE OBSTÁCULOS ]---------- #
            if min(laserMsg.ranges[110:250]) <= 0.7: # se obstáculo a frente, gira pra esquerda
                if time() - inicio_desviando > 5: # se robô está girando sem controle a mais de 5 segundos
                    # reseta a máquina de estados
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
                if time() - inicio_desviando > 5: # se robô está girando sem controle a mais de 5 segundos
                    # reseta a máquina de estados
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

        elif girando: # estado de navegação
            # ----------[ GIRA PARA FICAR EM DIREÇÃO AO GOAL ]---------- #
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
            # ----------[ SALVAMENTO DE MAPA E MUDANÇA DE GOAL ]---------- #
            # salva mapa
            print "Salvando PGM parcial"
            subprocess.Popen(['rosrun', 'map_server', 'map_saver', '-f', pgm_path+'mapa'])

            # reinicia estados de navegação
            desviando = False
            girando = True
            retaParaGoal = [None, None] # [a, b]
            distanciaParaGoal = None

            # tenta aleatoriamente, no máximo 50 vezes, encontrar um goal na parte não varrida do mapa
            for i in range(50):
                goal = np.array([randint(MAP_BL_POSITION[0]+1, MAP_TR_POSITION[0]-1), randint(MAP_BL_POSITION[1]+1, MAP_TR_POSITION[1]-1)])
                pose_goal_grid_i = bound(int(np.floor((goal[1]-MAP_BL_POSITION[1])/GRID_SIZE[2])), 0, GRID_SIZE[1])
                pose_goal_grid_j = bound(int(np.floor((goal[0]-MAP_BL_POSITION[0])/GRID_SIZE[2])), 0, GRID_SIZE[0])
                if grid[pose_goal_grid_i, pose_goal_grid_j] == 50: # se encontrar, quebra o loop
                    break

            # reinicia contagem do tempo e printa a nova jornada
            inicio_jornada = time()
            print 'nova jornada', goal


        # pose do robo na grid
        pose_robo_grid_i = bound(int(np.floor((pose_robo[1]-MAP_BL_POSITION[1])/GRID_SIZE[2])), 0, GRID_SIZE[1])
        pose_robo_grid_j = bound(int(np.floor((pose_robo[0]-MAP_BL_POSITION[0])/GRID_SIZE[2])), 0, GRID_SIZE[0])

        # itera nos ranges do laser
        for idx, distancia in enumerate(laserMsg.ranges, start=0):

            angulo = laserMsg.angle_increment * idx + laserMsg.angle_min

            # calculo da posição do obstáculo
            ctrl = np.array([np.cos(theta+angulo), np.sin(theta+angulo)])
            pose_obs = pose_robo + distancia * ctrl

            # calculo da posição do obstáculo na grid
            pose_obs_grid_i = bound(int(np.floor((pose_obs[1]-MAP_BL_POSITION[1])/GRID_SIZE[2])), 0, GRID_SIZE[1])
            pose_obs_grid_j = bound(int(np.floor((pose_obs[0]-MAP_BL_POSITION[0])/GRID_SIZE[2])), 0, GRID_SIZE[0])

            """
            Para agilizar o mapeamento, mesmo se o sensor não detectar um obstáculo, suas informações são usadas
            para formar um mapa. Nesse caso, considera-se que se ele não encontrou um obstáculo no seu range máximo,
            então todas as células estão livres.
            Porém, para evitar que isso atrapalhe em alguma célula já dada como ocupada, o acima só é feito caso
            a célula dada como "obstáculo" (ou seja, a célula coincidente com a posição máxima alcançada pelo laser)
            não esteja com probabilidade alta de ser ocupada. Aqui, foi usado o número máximo 61.
            """

            if distancia < laserMsg.range_max or grid[pose_obs_grid_i, pose_obs_grid_j] < 61:
                # para cada célula da linha de bresenham que liga a pose do robô no grid à pose obstáculo no grid
                # decremente log_odds_free
                for cell in list(bresenham(pose_robo_grid_i, pose_robo_grid_j, pose_obs_grid_i, pose_obs_grid_j))[:-1]:
                    if grid[cell[0],cell[1]] >= LOG_ODDS_FREE: # evita que o número fique negativo
                        grid[cell[0],cell[1]] -= LOG_ODDS_FREE
                    else:
                        grid[cell[0],cell[1]] = 0

            # se aplicável, incremente log_odds_occ na célula do grid equivalente à pose do obstáculo
            if distancia < laserMsg.range_max:
                if grid[pose_obs_grid_i, pose_obs_grid_j] <= 100-LOG_ODDS_OCC: # evita que o número passe de 100
                    grid[pose_obs_grid_i, pose_obs_grid_j] += LOG_ODDS_OCC
                else:
                    grid[pose_obs_grid_i, pose_obs_grid_j] = 100
            
        
        # ===========[ PUBLICAÇÂO DE MENSAGENS ]=========== #
        v_pub.publish(cmd_vel)
        occupancy_grid.data = grid.flatten()
        og_pub.publish(occupancy_grid)
        rate.sleep()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass