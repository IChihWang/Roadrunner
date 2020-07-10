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
import threading
import time
import numpy as np

from ortools.linear_solver import pywraplp
from sumolib import checkBinary
import traci
import traceback

import config as cfg

from gen_route import generate_routefile
import socket
import json

from IntersectionManager import IntersectionManager




src_dst_dict = None     # Load from the file (car_id, (src_idx, dst_idx))
path_table = None


arrival_rate = sys.argv[1]
seed = int(sys.argv[2])
sumoBinary = checkBinary('sumo-gui')
###################


def run(path_dict):

    """execute the TraCI control loop"""
    simu_step = 0


    car_dst_dict = dict()
    car_enter_time = dict()
    car_intersection_record = dict()
    car_path_idx_record = dict()

    travel_time_list = []

    # Create a list with intersection managers
    intersection_manager_list = []
    for idx in range(1, cfg.INTER_SIZE+1):
        for jdx in range(1, cfg.INTER_SIZE+1):
            intersection_manager_id = "%3.3o"%(idx) + "_" + "%3.3o"%(jdx)
            intersection_manager = IntersectionManager(intersection_manager_id)
            intersection_manager_list.append(intersection_manager)



    try:
        while traci.simulation.getMinExpectedNumber() > 0:

            if (simu_step*10)//1/10.0 >= 600:
                break

            traci.simulationStep()
            all_c = traci.vehicle.getIDList()
            # Update the position of each car
            for car_id in all_c:
                car_path = path_dict[car_id]

                # Generate source/destination
                if car_id not in car_dst_dict:

                    # Get source & destination

                    src_node_idx, dst_node_idx = src_dst_dict[car_id]
                    car_dst_dict[car_id] = dst_node_idx


                    # Record entering time
                    car_enter_time[car_id] = simu_step
                    car_path_idx_record[car_id] = -1
                    car_intersection_record[car_id] = None


                lane_id = traci.vehicle.getLaneID(car_id)

                is_handled = False
                for intersection_manager in intersection_manager_list:
                    if intersection_manager.check_in_my_region(lane_id):
                        if intersection_manager != car_intersection_record[car_id]:
                            car_intersection_record[car_id] = intersection_manager
                            car_path_idx_record[car_id] += 1

                        is_handled = True
                        car_turn = car_path[car_path_idx_record[car_id]]
                        data = intersection_manager.update_car(car_id, lane_id, simu_step, car_turn)


                        break

                if not is_handled:
                    traci.vehicle.setSpeed(car_id, cfg.MAX_SPEED)

            del_car_id_list = []
            for car_id in car_dst_dict:
                if car_id not in all_c:
                    # The car exits the system
                    del_car_id_list.append(car_id)
            for car_id in del_car_id_list:
                travel_time_list.append(simu_step-car_enter_time[car_id])
                del car_dst_dict[car_id]
                del car_enter_time[car_id]


            for intersection_manager in intersection_manager_list:
                intersection_manager.run(simu_step)


            simu_step += cfg.TIME_STEP



    except Exception as e:
        traceback.print_exc()


    #print("avg_fuel = ",intersection_manager.total_fuel_consumption/intersection_manager.fuel_consumption_count)
    avg_travel_time = sum(travel_time_list)/len(travel_time_list)
    car_num = len(travel_time_list)
    print("avg_delay = ", avg_travel_time)
    print("car_num = ", len(travel_time_list))
    print("arrival_rate = ", len(travel_time_list)/600.0)
    sys.stdout.flush()

    traci.close()

    return (car_num, avg_travel_time)


def recursive_run(car_id_list, car_id_list_idx, path_dict):
    # Termination
    if car_id_list_idx == len(car_id_list):
        net_name = "lane%iby%i.net.xml" % (cfg.INTER_SIZE, cfg.INTER_SIZE)
        route_name = "%i_%s_%i.rou.xml" % (cfg.INTER_SIZE, arrival_rate, seed)
        traci.start([sumoBinary, "-c", "data/UDTA.sumocfg",
                                 "--step-length", str(cfg.TIME_STEP),
                                 "--collision.mingap-factor", "0",
                                 "-n", "data/net/" + net_name,
                                 "-r", "data/routes/" + route_name])
        print("Running simulation...")
        car_num, avg_travel_time = run(path_dict)

        return (car_num, avg_travel_time)

    car_id = car_id_list[car_id_list_idx]
    path_list = path_table[str(tuple(src_dst_dict[car_id]))]

    optimal_travel_time = 99999999
    optimal_car_num = None
    for path in path_list:
        path_dict[car_id] = path
        car_num, avg_travel_time = recursive_run(car_id_list, car_id_list_idx+1, path_dict)
        if avg_travel_time < optimal_travel_time:
            optimal_travel_time = avg_travel_time
            optimal_car_num = car_num

    return (optimal_car_num, optimal_travel_time)




###########################
# Main function
if __name__ == "__main__":
    print("Usage: python code.py <arrival_rate (0~1.0)> <seed>")


    random.seed(seed)  # make tests reproducible
    numpy.random.seed(seed)


    # Load from the file
    src_dst_file_name = "%i_%s_%i_src_dst.json" % (cfg.INTER_SIZE, arrival_rate, seed)
    with open('data/routes/'+src_dst_file_name) as json_file:
        src_dst_dict = json.load(json_file)

    car_id_list = list(src_dst_dict.keys())
    path_table = json.load(open('data/path/%iX%i.json'%(cfg.INTER_SIZE, cfg.INTER_SIZE)))

    path_dict = dict()
    car_id_list_idx = 0
    optimal_car_num, optimal_travel_time = recursive_run(car_id_list, car_id_list_idx, path_dict)

    print("Optimal travel time: %f" % optimal_travel_time)
    print("Car number: %i" % optimal_car_num)
    print("Arrival rate: %f" % optimal_car_num/600.0)
