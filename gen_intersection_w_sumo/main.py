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
from gen_route import generate_routefile
import json


###################


def run():
    """execute the TraCI control loop"""
    simu_step = 0
    car_list = []

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

            if traci.simulation.getStartingTeleportNumber() > 0:
                traci.close()
                return (False, None)

            simu_step += cfg.TIME_STEP
    except Exception as e:
        traci.close()
        traceback.print_exc()
        return (False, None)

    traci.close()
    return (True, car_0_leave_time-car_0_enter_time)


###########################
# Main function
if __name__ == "__main__":
    print("Usage: python code.py")

    sumoBinary = checkBinary('sumo-gui')

    data_dict = dict()
    in_intersection_travel_time_dict = dict()

    '''
    for sublane_1 in range(cfg.LANE_NUM_PER_DIRECTION):
        for dir_2 in range(4):
            for sublane_2 in range(cfg.LANE_NUM_PER_DIRECTION):
                for turn_1 in ['S', 'L', 'R']:
                    for turn_2 in ['S','L', 'R']:
                        lane_2 = dir_2*cfg.LANE_NUM_PER_DIRECTION + sublane_2

                        if sublane_1 >= lane_2:
                            continue

                        # Tricky part to skip some search for roundabout
                        if dir_2 == 0:
                            if turn_2 == 'R':
                                continue
                            elif turn_2 == 'S' and (turn_1 == 'S' or turn_1 == 'L'):
                                continue
                            elif turn_2 == 'L' and (turn_1 == 'L'):
                                continue

                        elif turn_1 == 'R':
                            continue
                        elif turn_1 == 'S':
                            if dir_2 == 2 or dir_2 == 3:
                                continue
                            elif turn_2 == 'R':
                                if sublane_1 < sublane_2:
                                    continue
                        elif turn_1 == 'L':
                            if dir_2 == 3:
                                continue
                            elif dir_2 == 1:
                                if turn_2 == 'R' or turn_2 == 'S':
                                    if sublane_1 < sublane_2:
                                        continue
                            elif dir_2 == 2:
                                if turn_2 == 'R':
                                    if sublane_1 < sublane_2:
                                        continue

                        key_str = str(sublane_1) + turn_1 + str(lane_2) + turn_2

                        in_intersection_travel_time = None
                        tau_S1_S2 = None
                        time_gap_1_search = 0
                        for time_gap in np.arange(900, -900, -5):
                            time_gap_1_search = time_gap
                            time_gap /= 100.0

                            print(key_str, time_gap, "==================")

                            #time_gap = -7.5
                            generate_routefile(time_gap, sublane_1, turn_1, sublane_2, turn_2, dir_2)

                            # 3. This is the normal way of using traci. sumo is started as a subprocess and then the python script connects and runs
                            try:
                                traci.start([sumoBinary, "-c", "data/icacc+.sumocfg",
                                                         "--tripinfo-output", "tripinfo.xml","--step-length", str(cfg.TIME_STEP),
                                                         "--collision.mingap-factor", "0"])

                                # 4. Start running SUMO
                                result = run()

                                if not result[0]:
                                    # Collision happened
                                    if time_gap >= 0:
                                        # car 1 comes first
                                        if tau_S1_S2 == None:
                                            tau_S1_S2 = time_gap
                                            break
                                else:
                                    # no collision happens
                                    in_intersection_travel_time = result[1]

                                traci.close()
                            except Exception as e:
                                traceback.print_exc()
                                None


                        tau_S2_S1 = None
                        for time_gap in np.arange(-900, time_gap_1_search, 5):
                            time_gap /= 100.0

                            print(key_str, time_gap, "==================")

                            #time_gap = -7.5
                            generate_routefile(time_gap, sublane_1, turn_1, sublane_2, turn_2, dir_2)

                            # 3. This is the normal way of using traci. sumo is started as a subprocess and then the python script connects and runs
                            try:
                                traci.start([sumoBinary, "-c", "data/icacc+.sumocfg",
                                                         "--tripinfo-output", "tripinfo.xml","--step-length", str(cfg.TIME_STEP),
                                                         "--collision.mingap-factor", "0",
                                                         "--collision.action", "remove"])

                                # 4. Start running SUMO
                                result = run()

                                if not result:
                                    # Collision happened
                                    if time_gap >= 0:
                                        # car 1 comes first
                                        if tau_S2_S1 == None:
                                            tau_S2_S1 = -time_gap
                                            break
                                else:
                                    # no collision happens
                                    in_intersection_travel_time = result[1]

                                traci.close()
                            except Exception as e:
                                traceback.print_exc()
                                None

                        print("Done-----------", key_str)

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
    time_gap = 50
    sublane_1 = 0
    dir_2 = 0
    sublane_2 = 1
    turn_1 = 'R'
    turn_2 = 'S'
    lane_2 = dir_2*cfg.LANE_NUM_PER_DIRECTION + sublane_2
    key_str = str(sublane_1) + turn_1 + str(lane_2) + turn_2

    in_intersection_travel_time = None
    tau_S1_S2 = None
    tau_S2_S1 = None
    time_gap_1_search = 0

    time_gap_1_search = time_gap
    time_gap /= 100.0

    print(key_str, time_gap, "==================")

    #time_gap = -7.5
    generate_routefile(time_gap, sublane_1, turn_1, sublane_2, turn_2, dir_2)

    # 3. This is the normal way of using traci. sumo is started as a subprocess and then the python script connects and runs
    try:
        traci.start([sumoBinary, "-c", "data/icacc+.sumocfg",
                                 "--tripinfo-output", "tripinfo.xml","--step-length", str(cfg.TIME_STEP),
                                 "--collision.mingap-factor", "0"])

        # 4. Start running SUMO
        result = run()

        if not result[0]:
            # Collision happened
            if time_gap >= 0:
                # car 1 comes first
                if tau_S1_S2 == None:
                    tau_S1_S2 = time_gap
        else:
            # no collision happens
            in_intersection_travel_time = result[1]

        traci.close()
    except Exception as e:
        traceback.print_exc()
        None


    print("Done-----------", key_str)

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
