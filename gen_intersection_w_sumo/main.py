from __future__ import absolute_import
from __future__ import print_function


import os
import sys

sys.path.append('/usr/share/sumo/tools/')
# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import optparse
import random
import numpy
import time
import numpy as np

from sumolib import checkBinary
import traci
import traceback

import config as cfg
import csv
from gen_route import generate_routefile


###################

car_list = []

def run():
    """execute the TraCI control loop"""
    simu_step = 0

    try:
        while traci.simulation.getMinExpectedNumber() > 0:

            traci.simulationStep()
            all_c = traci.vehicle.getIDList()
            # Update the position of each car
            for car_id in all_c:
                if car_id not in car_list:
                    traci.vehicle.setMinGap(car_id, cfg.HEADWAY)
                    traci.vehicle.setSpeedMode(car_id, 0)
                    traci.vehicle.setLaneChangeMode(car_id, 0)
                    traci.vehicle.setSpeed(car_id, cfg.MAX_SPEED)

                    traci.vehicle.moveTo(car_id, traci.vehicle.getLaneID(car_id), 180)
                    car_list.append(car_id)

            for car_id in car_list:
                traci.vehicle.getLaneID(car_id)

            simu_step += cfg.TIME_STEP
    except Exception as e:
        traceback.print_exc()
        return False

    traci.close()
    return True


###########################
# Main function
if __name__ == "__main__":
    print("Usage: python code.py")

    sumoBinary = checkBinary('sumo')

    for sublane_1 in range(cfg.LANE_NUM_PER_DIRECTION):
        for sublane_2 in range(cfg.LANE_NUM_PER_DIRECTION):
            for dir_2 in range(4):
                for turn_1 in ['S', 'R', 'L']:
                    for turn_2 in ['S', 'R', 'L']:
                        if dir_2 == 0 and sublane_1==sublane_2 and turn_1 == turn_2:
                            continue

                        lane_2 = dir_2*cfg.LANE_NUM_PER_DIRECTION + sublane_2
                        for time_gap in np.arange(-15.0, 15.0, 0.1):

                            #time_gap = -7.5
                            generate_routefile(time_gap, sublane_1, turn_1, sublane_2, turn_2, dir_2)

                            # 3. This is the normal way of using traci. sumo is started as a subprocess and then the python script connects and runs
                            traci.start([sumoBinary, "-c", "data/icacc+.sumocfg",
                                                     "--tripinfo-output", "tripinfo.xml","--step-length", str(cfg.TIME_STEP),
                                                     "--collision.mingap-factor", "0",
                                                     "--collision.action", "remove"])

                            # 4. Start running SUMO
                            result = run()

                            if not result:
                                # Collision happened
                                print("ERRRRRR")
                                None
                            else:
                                # no collision happens
                                None
