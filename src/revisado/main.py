#!/usr/bin/env python
# coding=latin1

import numpy as np
import sys
import signal
import subprocess
from time import time

import rospy

from robot import Robot
from map import Map

# ============[ DEFINIÇÕES DO USUÁRIO ]============ #
MAP_SIDE = 16 # largura do mapa
MAP_GRID_RESOLUTION_MULTIPLIER = 10 # nível de detalhe do mapa. quanto maior, mais subdividido o grid
MAP_INITIAL_CELL_VALUE = 50
MAP_LOG_ODDS_FREE = 5 # constante l_{free} - l_0 = 40 - 35
MAP_LOG_ODDS_OCC  = 25 # constante l_{occ} - l_0 = 60 - 35

MAP_SAVE_INTERVAL = 20

BOTS_NUMBER = 1

BOT_KP = 0.4

rospy.init_node('tpf')
rate = rospy.Rate(5)


# Obtenção do path para salvar as imagens de saída do mapa
if len(sys.argv) >= 2:
    pgm_path = sys.argv[1]
    if pgm_path[-1] != '/':
        pgm_path += '/'
else:
    pgm_path = './'


def main ():
    global mapa, robos

    mapa = Map(MAP_SIDE, MAP_INITIAL_CELL_VALUE, MAP_LOG_ODDS_FREE, MAP_LOG_ODDS_OCC, MAP_GRID_RESOLUTION_MULTIPLIER)
    robos = []
    if BOTS_NUMBER == 0:
        print("O número de robôs é zero")
        sys.exit(1)
    elif BOTS_NUMBER == 1:
        print "cmd_vel", "base_scan", "base_pose_ground_truth"
        bot = Robot("cmd_vel", "base_scan", "base_pose_ground_truth", BOT_KP)
        robos.append(bot)
    else:
        for i in range(BOTS_NUMBER):
            bot = Robot("robot_"+str(i)+"/cmd_vel", "robot_"+str(i)+"/base_scan", "robot_"+str(i)+"/base_pose_ground_truth", BOT_KP)
            robos.append(bot)

    print 'Pressione CTRL+C para salvar o mapa de ocupação final com e sem threshold e depois sair'

    # aguarda robôs ficarem prontos
    while True:
        ready = True
        print "Quaaase"
        for bot in robos:
            ready = bot.is_ready()
            if not ready:
                rate.sleep()
                break
        if ready:
            break

    print "FOI!"

    # registra Event Listener para o SIGINT
    signal.signal(signal.SIGINT, interrupt_ctrl_c)

    last_map_save = time()

    bots_done = np.zeros(BOTS_NUMBER, dtype=bool)
    while not rospy.is_shutdown():
        # print "LOOOOOP"
        for i,bot in enumerate(robos):
            bot.T.T.show()
            pode_mapear = bot.do_navigation()
            if pode_mapear:
                snap = bot.take_sensor_snapshot()
                mapa.do_mapping(snap.pose, snap.laser_msg)
                bots_done[i] = bot.is_done()
        mapa.publish()
        if time()-last_map_save > MAP_SAVE_INTERVAL:
            save_map()
            last_map_save = time()
        if bots_done.all() == True:
            print "ACABO"
            interrupt_ctrl_c(1,2)
        rate.sleep()

def save_map ():
    # roda map_saver
    subprocess.Popen(['rosrun', 'map_server', 'map_saver', '-f', pgm_path+'mapa'])

# ===========[ SALVAR IMAGEM FINAL AO CTRL+C (SIGINT) ]=========== #
def interrupt_ctrl_c (sig, frame):
    print "\n\n===============[ AGUARDE ]=============="
    print "Salvando imagens finais e saindo...\n\n"
    # roda map_saver
    p=subprocess.Popen(['rosrun', 'map_server', 'map_saver', '-f', pgm_path+'mapa'])
    # continua enviando mensagens enquanto o map_saver estiver ouvindo
    while p.poll() == None:
        mapa.publish()
        rate.sleep()
    
    # roda map_saver para imagem binária (valor célula <= 15 = livre, do contrário, ocupada)
    p=subprocess.Popen(['rosrun', 'map_server', 'map_saver', '--occ', '16', '--free', '15', '-f', pgm_path+'mapa_threshold'])
    # continua enviando mensagens enquanto o map_saver estiver ouvindo
    while p.poll() == None:
        mapa.publish()
        rate.sleep()
    
    # encerra a execução do programa
    sys.exit()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
