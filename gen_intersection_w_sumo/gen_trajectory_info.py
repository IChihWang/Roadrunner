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

sys.path.append('..')
import config as cfg
from gen_route import generate_routefile, generate_one_car_routefile
import json
import math


###################
resolution = 2

def run():
    """execute the TraCI control loop"""
    simu_step = 0
    car_list = []

    car_0_enter_time = None
    car_0_leave_time = None
    trajectory_list = []
    advising_info = []
    XY_record = []

    min_coord_x = 99999
    min_coord_y = 99999

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
                coord = traci.vehicle.getPosition(car_id)
                angle = traci.vehicle.getAngle(car_id)
                trajectory_list.append({'distance':in_intersection_distance, 'X':coord[0], 'Y':coord[1]})

                x0 = (coord[0] + 50) * resolution / cfg.LANE_WIDTH
                y0 = (coord[1] + 50) * resolution / cfg.LANE_WIDTH


                x1 = (coord[0] - (cfg.LANE_WIDTH/4)*math.cos(angle*math.pi/180) + 50) * resolution / cfg.LANE_WIDTH
                y1 = (coord[1] + (cfg.LANE_WIDTH/4)*math.sin(angle*math.pi/180) + 50) * resolution / cfg.LANE_WIDTH
                x2 = (coord[0] + (cfg.LANE_WIDTH/4)*math.cos(angle*math.pi/180) + 50) * resolution / cfg.LANE_WIDTH
                y2 = (coord[1] - (cfg.LANE_WIDTH/4)*math.sin(angle*math.pi/180) + 50) * resolution / cfg.LANE_WIDTH

                x0 = int(x0)
                y0 = int(y0)
                id_0 = str(x0)+str(y0)
                x1 = int(x1)
                y1 = int(y1)
                id_1 = str(x1)+str(y1)
                x2 = int(x2)
                y2 = int(y2)
                id_2 = str(x2)+str(y2)

                if id_0 not in XY_record:
                    XY_record.append(id_0)
                    advising_info.append({'distance':in_intersection_distance, 'X':x0, 'Y':y0})
                if id_1 not in XY_record:
                    XY_record.append(id_1)
                    advising_info.append({'distance':in_intersection_distance, 'X':x1, 'Y':y1})
                if id_2 not in XY_record:
                    XY_record.append(id_2)
                    advising_info.append({'distance':in_intersection_distance, 'X':x2, 'Y':y2})
                min_coord_x = min(min_coord_x, x0)
                min_coord_x = min(min_coord_x, x1)
                min_coord_x = min(min_coord_x, x2)
                min_coord_y = min(min_coord_y, y0)
                min_coord_y = min(min_coord_y, y1)
                min_coord_y = min(min_coord_y, y2)

            simu_step += cfg.TIME_STEP

    except Exception as e:
        traceback.print_exc()

    return trajectory_list, advising_info, car_0_leave_time-car_0_enter_time, min_coord_x, min_coord_y


###########################
# Main function
if __name__ == "__main__":
    print("Usage: python code.py")

    if sys.platform == "win32":
        os.system("rmdir data /s/q")
        os.system("xcopy ..\data data /E/Q/I")
    else:
        os.system("rm -r data")
        os.system("cp -r ../data .")
    sumoBinary = checkBinary('sumo')

    data_dict = dict()
    in_intersection_travel_time_dict = dict()
    trajectory_list_dict = dict()
    advise_list_dict = dict()

    min_min_coord_x = 99999
    min_min_coord_y = 99999

    for lane_1 in range(4*cfg.LANE_NUM_PER_DIRECTION):
    #for lane_1 in [0]:
        for turn_1 in ['S', 'L', 'R']:
        #for turn_1 in ['S', 'L']:
            key_str = str(lane_1) + turn_1

            in_intersection_travel_time = None

            print(key_str, "==================")

            #time_gap = -7.5
            generate_one_car_routefile(lane_1, turn_1)

            # 3. This is the normal way of using traci. sumo is started as a subprocess and then the python script connects and runs
            try:
                traci.start([sumoBinary, "-c", "data/icacc+.sumocfg",
                                         "--tripinfo-output", "tripinfo.xml","--step-length", str(cfg.TIME_STEP),
                                         "--collision.mingap-factor", "0"])

                # 4. Start running SUMO
                trajectory_list, advising_info, in_intersection_travel_time, min_coord_x, min_coord_y = run()
                min_min_coord_x = min(min_min_coord_x, min_coord_x)
                min_min_coord_y = min(min_min_coord_y, min_coord_y)

                in_intersection_travel_time_dict[key_str] = in_intersection_travel_time
                trajectory_list_dict[key_str] = trajectory_list
                advise_list_dict[key_str] = advising_info


            except Exception as e:
                traceback.print_exc()
                None

            try:
                traci.close()
            except Exception as e:
                None


    with open("inter_data/sumo_inter_info"+str(cfg.LANE_NUM_PER_DIRECTION)+".json", 'w') as file:
        file.write(json.dumps(trajectory_list_dict))

    for id, data_list in advise_list_dict.items():
        for data in data_list:
            data['X'] -= min_min_coord_x
            data['Y'] -= min_min_coord_y

    with open("../advise_info/sumo_lane_adv"+str(cfg.LANE_NUM_PER_DIRECTION)+".json", 'w') as file:
        file.write(json.dumps(advise_list_dict))

    with open("../inter_length_info/sumo_lane"+str(cfg.LANE_NUM_PER_DIRECTION)+".json", 'w') as file:
                file.write(json.dumps(in_intersection_travel_time_dict))

    if sys.platform == "win32":
        os.system("rmdir data /s/q")
    else:
        os.system("rm -r data")
