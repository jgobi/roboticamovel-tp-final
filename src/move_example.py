#!/usr/bin/env python
# coding=latin1

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid

from tf.transformations import euler_from_quaternion

import numpy as np
from bresenham import bresenham

laserMsg = None
odomMsg = None

XMIN = -16 # x mínimo do mapa (usualmente (-1)*largura/2)
YMIN = -16 # y mínimo do mapa (usualmente (-1)*altura/2)
MAP_RESOLUTION = 1.0 # resolucao do mapa

GRID_RESOLUTION_MULTIPLIER = 10 # nível de detalhe

# --- INICIO NÃO EDITAR
GRID_SIZE = (np.abs(XMIN*2*GRID_RESOLUTION_MULTIPLIER), np.abs(YMIN*2*GRID_RESOLUTION_MULTIPLIER), MAP_RESOLUTION/GRID_RESOLUTION_MULTIPLIER) # The last one is the resolution
# --- FIM NÃO EDITAR

laserMsg = None
odomMsg = None
pose = None

# ============[ CONSTANTES DO CONTROLE P ]============ #
kp = 0.9

# ======================[ GOAL ]====================== #
# if len(sys.argv) >= 3:
#     goal = (int(sys.argv[1]), int(sys.argv[2]))
# else:
goal = (7, 7)


def run ():
    global laserMsg, odomMsg, pose, kp, XMIN, YMIN, GRID_SIZE

    rospy.init_node('move_example', anonymous=True)
    v_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    og_pub = rospy.Publisher('occupancy_grid', OccupancyGrid, queue_size=10)
    rospy.Subscriber('base_scan', LaserScan, LaserCallback)
    rospy.Subscriber('base_pose_ground_truth', Odometry, OdomCallback)
    rate = rospy.Rate(5)
    occupancy_grid = OccupancyGrid()
    occupancy_grid.info.width = int(GRID_SIZE[0])
    occupancy_grid.info.height = int(GRID_SIZE[1])
    occupancy_grid.info.resolution = GRID_SIZE[2]
    occupancy_grid.data = np.full(GRID_SIZE[0]*GRID_SIZE[1], 50)
    cmd_vel = Twist()

    grid = np.full((GRID_SIZE[0],GRID_SIZE[1]), 50)

    # ===========[ DESVIO DE OBSTÁCULOS 1 ]=========== #
    desviando = False
    girando = True
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

        erroU = (goal[0]-pose.position.x, goal[1]-pose.position.y) # erro em referência ao sistema universal
        theta2 = np.arctan2(erroU[1], erroU[0]) # ângulo entre o x universal e o vetor para o goal

        erroCos = np.cos(theta2-theta) # cosseno do ângulo entre o sistema do robô e o vetor para o goal
        erroSen = np.sin(theta2-theta) # seno do ângulo entre o sistema do robô e o vetor para o goal
        distanciaEuclidiana = np.sqrt(erroU[0]**2 + erroU[1]**2)
        erro = (distanciaEuclidiana*erroCos, distanciaEuclidiana*erroSen, theta2-theta) # erro em referência ao sistema do robo


        # ===========[ DESVIO DE OBSTÁCULOS 2 ]=========== #

        if desviando:
            if min(laserMsg.ranges[110:250]) <= 0.53: # se obstáculo a frente, gira pra esquerda
                cmd_vel.angular.z = 3.0
                cmd_vel.linear.x = 0
                cmd_vel.linear.y = 0
            elif min(laserMsg.ranges[0:120]) <= 0.53: # se obstáculo a direita, segue em frente
                cmd_vel.angular.z = 0
                cmd_vel.linear.x = 0.45
                cmd_vel.linear.y = 0
            else: # do contrário, vira pra direita
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

        elif min(laserMsg.ranges[110:250]) <= 0.53 and distanciaEuclidiana > 0.3: # obstáculo à frente
            # salva hit point
            distanciaParaGoal = distanciaEuclidiana
            # começa a desviar do obstáculo
            desviando = True

        else: # movimento normal
            cmd_vel.angular.z = kp*erro[2]
            cmd_vel.linear.x = kp*erro[0]
            cmd_vel.linear.y = kp*erro[1]


        # ===========[ MAPEAMENTO ]=========== #
        
        rot_z = np.array([[np.cos(theta) , np.sin(theta), 0],
                           [-np.sin(theta), np.cos(theta), 0],
                           [0             , 0            , 1]])
            
        pose_odom = np.array([pose.position.x, pose.position.y, 0])

        # angle = 0
        # for i in range(5):
        #     break
        #     print angle, laserMsg.ranges[angle]
        #     distancia_obs = np.array([laserMsg.ranges[angle*2]*np.cos(np.deg2rad(angle)), laserMsg.ranges[angle*2]*np.sin(np.deg2rad(angle)), 0])
        #     print rot_z.dot(distancia_obs.T) + pose_odom
        #     angle+=45
        #     print "\n"

        posicaoGrid_robo_i = int(np.ceil((pose_odom[1]-YMIN)/GRID_SIZE[2]))
        posicaoGrid_robo_j = int(np.ceil((pose_odom[0]-XMIN)/GRID_SIZE[2]))
        for idx, distancia in enumerate(laserMsg.ranges, start=0):
            if distancia >= laserMsg.range_max:
                continue

            angulo = laserMsg.angle_increment * idx

            distancia_decomposta = np.array([distancia*np.cos(angulo), distancia*np.sin(angulo), 0])
            posicaoObstaculo = rot_z.dot(distancia_decomposta) + pose_odom

            posicaoGrid_obstaculo_i = int(np.ceil((posicaoObstaculo[1]-YMIN)/GRID_SIZE[2]))
            posicaoGrid_obstaculo_j = int(np.ceil((posicaoObstaculo[0]-XMIN)/GRID_SIZE[2]))

            if posicaoGrid_obstaculo_i < 0 or posicaoGrid_obstaculo_j < 0 or posicaoGrid_obstaculo_i >= GRID_SIZE[0] or posicaoGrid_obstaculo_j >= GRID_SIZE[1]:
                continue
            for cell in list(bresenham(posicaoGrid_robo_i, posicaoGrid_robo_j, posicaoGrid_obstaculo_i, posicaoGrid_obstaculo_j))[:-1]:
                # print cell
                # print posicaoGrid_robo[0], posicaoGrid_robo[1], posicaoGrid_obstaculo[0], posicaoGrid_obstaculo[1]
                grid[cell[0]-1,cell[1]-1] -= 1
                if grid[cell[0]-1,cell[1]-1] < 0:
                    grid[cell[0]-1,cell[1]-1] = 0
                    
            grid[posicaoGrid_obstaculo_i-1, posicaoGrid_obstaculo_j-1] += 2
            if grid[posicaoGrid_obstaculo_i-1, posicaoGrid_obstaculo_j-1] > 100:
                grid[posicaoGrid_obstaculo_i-1, posicaoGrid_obstaculo_j-1] = 100
            #print angulo, posicaoGrid
        # print occupancy_grid
            
        print "\n"
        v_pub.publish(cmd_vel)
        occupancy_grid.data = grid.reshape(GRID_SIZE[0]*GRID_SIZE[1])
        og_pub.publish(occupancy_grid)
        rate.sleep()

def LaserCallback(msg):
    global laserMsg
    laserMsg = msg

def OdomCallback(msg):
    global odomMsg, pose
    odomMsg = msg
    pose = odomMsg.pose.pose

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass

