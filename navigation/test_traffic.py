import pickle
import numpy as np
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET
import copy
import os, sys
import numba

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import sumolib
from sumolib import checkBinary
import traci
from traci.exceptions import FatalTraCIError

dirname = os.path.dirname(__file__)
SUMOCFG_DIR = dirname + "/Map4_Tsinghua_Intersection/configuration.sumocfg"
SUMO_BINARY = checkBinary('sumo-gui')
# SUMO_BINARY = checkBinary('sumo')
STEP_TIME = 0.1

class Traffic(object):

    def __init__(self):
        self.step_time = str(STEP_TIME)
        # seed = 6
        seed = 1000
        # seed = 2000
        # seed = 3000
        # seed = 5000
        import time
        start = time.time()
        port = sumolib.miscutils.getFreeSocketPort()
        try:
            traci.start(
                [SUMO_BINARY, "-c", SUMOCFG_DIR,
                 "--step-length", self.step_time,
                 "--lateral-resolution", "3.75",
                 #"--random",
                 # "--start",
                 # "--quit-on-end",
                 "--no-warnings",
                 "--no-step-log",
                 #"--collision.action", "remove"
                 '--seed', str(int(seed))
                 ], port=port, numRetries=5)  # '--seed', str(int(seed))
        except FatalTraCIError:
            print('Retry by other port')
            port = sumolib.miscutils.getFreeSocketPort()
            traci.start(
                [SUMO_BINARY, "-c", SUMOCFG_DIR,
                 "--step-length", self.step_time,
                 "--lateral-resolution", "1.25",
                 # "--random",
                 # "--start",
                 # "--quit-on-end",
                 "--no-warnings",
                 "--no-step-log",
                 '--seed', str(int(seed))
                 ], port=port, numRetries=5)  # '--seed', str(int(seed))

        # print("Sumo startup time: ", end - start)
        # 先让交通流运行一段时间
        traci.vehicle.add(vehID='egoID', routeID='self_route', typeID="car_4")
        while traci.simulation.getTime() < 50 + 8000:   #这里的时间单位是秒
            # traci.vehicle.moveToXY(egoID, edgeID, lane, ego_x_in_sumo, ego_y_in_sumo, ego_a_in_sumo*180/np.pi, keepRoute=1)
            # traci.vehicle.setLength(egoID, ego_dict['l'])
            # traci.vehicle.setWidth(egoID, ego_dict['w'])
            # traci.vehicle.setSpeed(egoID, ego_v_x)
            traci.simulationStep()


    def close(self):
        traci.close()

traffic = Traffic()