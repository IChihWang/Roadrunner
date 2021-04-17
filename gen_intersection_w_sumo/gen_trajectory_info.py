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
from gen_route import generate_routefile, generate_one_car_routefile
import json


###################

car_list = []

def run():
    """execute the TraCI control loop"""
    simu_step = 0

    car_0_enter_time = None
    car_0_leave_time = None

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
                    lane = traci.vehicle.getLaneID(car_id)

                    traci.vehicle.moveTo(car_id, lane, 180)

                    car_list.append(car_id)

            if len(car_list) > 0:
                car_id = car_list[0]
                if car_id in all_c:
                    now_lane = traci.vehicle.getLaneID(car_id)
                    lane_idx = now_lane.split('_')
                    if now_lane[0] == '-' and car_0_leave_time == None:
                        car_0_leave_time = simu_step
                    elif now_lane[0] == ':' and car_0_enter_time == None:
                        car_0_enter_time = simu_step

            if car_0_enter_time != None and car_0_leave_time == None:
                in_intersection_time_stamp = simu_step-car_0_enter_time
                in_intersection_distance = in_intersection_time_stamp*cfg.MAX_SPEED
                print(traci.vehicle.getPosition(car_id)[0], traci.vehicle.getPosition(car_id)[1])


            simu_step += cfg.TIME_STEP

        traci.close()
    except Exception as e:
        traceback.print_exc()

    return


###########################
# Main function
if __name__ == "__main__":
    print("Usage: python code.py")

    sumoBinary = checkBinary('sumo-gui')

    data_dict = dict()
    in_intersection_travel_time_dict = dict()

    for lane_1 in range(1):
        for turn_1 in ['S']:

            key_str = str(lane_1) + turn_1

            in_intersection_travel_time = None
            tau_S1_S2 = None
            time_gap_1_search = 0

            print(key_str, "==================")

            #time_gap = -7.5
            generate_one_car_routefile(lane_1, turn_1)

            # 3. This is the normal way of using traci. sumo is started as a subprocess and then the python script connects and runs
            try:
                traci.start([sumoBinary, "-c", "data/icacc+.sumocfg",
                                         "--tripinfo-output", "tripinfo.xml","--step-length", str(cfg.TIME_STEP),
                                         "--collision.mingap-factor", "0"])

                # 4. Start running SUMO
                run()

                traci.close()
            except Exception as e:
                traceback.print_exc()
                None


            '''
            # Write tau if gap is necessary
            if tau_S1_S2 != None or tau_S2_S1 != None:
                if tau_S1_S2 == None:
                    tau_S1_S2 = 0
                if tau_S2_S1 == None:
                    tau_S2_S1 = 0
                data_dict[key_str] = {'tau_S1_S2':tau_S1_S2+cfg.HEADWAY/cfg.MAX_SPEED, 'tau_S2_S1':tau_S2_S1+cfg.HEADWAY/cfg.MAX_SPEED}
                with open("../inter_info/sumo_lane"+str(cfg.LANE_NUM_PER_DIRECTION)+".json", 'w') as file:
                    file.write(json.dumps(data_dict))

            if in_intersection_travel_time != None:
                in_intersection_travel_time_dict[str(sublane_1) + turn_1] = in_intersection_travel_time
                with open("../inter_length_info/sumo_lane"+str(cfg.LANE_NUM_PER_DIRECTION)+".json", 'w') as file:
                    file.write(json.dumps(in_intersection_travel_time_dict))
            '''
