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
import matplotlib.pyplot as plt
import numpy as np

from ortools.linear_solver import pywraplp
from sumolib import checkBinary
import traci
import traceback

import config as cfg

from gen_route import generate_routefile_with_src_dst
import socket
import json
import csv



# For debug
#from playsound import playsound

from IntersectionManager import IntersectionManager
#from myGraphic import Gui
#import myGraphic


#myGraphic.gui = Gui()

car_path_dict = dict()
car_intersection_id_dict = dict()


###################

simulation_time = 3600

def run():
    car_enter_time = dict()
    car_travel_time = []
    car_delay_time = []


    """execute the TraCI control loop"""
    simu_step = 0
    total_car_num = 0

    # Create a list with intersection managers
    intersection_manager_list = []
    intersection_manager_dict = dict()
    for idx in range(1, cfg.INTER_SIZE+1):
        for jdx in range(1, cfg.INTER_SIZE+1):
            intersection_manager_id = "%3.3o"%(idx) + "_" + "%3.3o"%(jdx)
            intersection_manager = IntersectionManager(intersection_manager_id)
            intersection_manager_list.append(intersection_manager)
            intersection_manager_dict[(idx, jdx)] = intersection_manager

    for idx in range(1, cfg.INTER_SIZE+1):
        for jdx in range(1, cfg.INTER_SIZE+1):
            if idx <= cfg.INTER_SIZE-1:
                intersection_manager_dict[(idx, jdx)].connect(1, intersection_manager_dict[(idx+1, jdx)], 3)

            if jdx <= cfg.INTER_SIZE-1:
                intersection_manager_dict[(idx, jdx)].connect(2, intersection_manager_dict[(idx, jdx+1)], 0)


    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            #if (simu_step*10)//1/10.0 >= simulation_time:
                #break

            traci.simulationStep()
            all_c = traci.vehicle.getIDList()
            server_send_str = ""
            # Update the position of each car
            for car_id in all_c:

                # Generate source/destination
                if car_id not in car_path_dict:
                    # Record entering time
                    car_enter_time[car_id] = simu_step

                    car_info = car_id.split("_")
                    car_path_dict[car_id] = car_info[2]
                    total_car_num += 1

                lane_id = traci.vehicle.getLaneID(car_id)

                is_handled = False
                for intersection_manager in intersection_manager_list:
                    if intersection_manager.check_in_my_region(lane_id):
                        is_handled = True
                        car_turn = car_path_dict[car_id][0]

                        #if car_id == 'car_10845_LS':
                            #print(car_id, car_path_dict[car_id])
                        data = intersection_manager.update_car(car_id, lane_id, simu_step, car_turn)
                        if data != None:
                            intersection_id = data
                            if (car_id in car_intersection_id_dict) and (intersection_id != car_intersection_id_dict[car_id]):
                                car_path_dict[car_id] = car_path_dict[car_id][1:]
                            car_intersection_id_dict[car_id] = intersection_id
                        break

                if not is_handled:
                    traci.vehicle.setSpeed(car_id, cfg.MAX_SPEED)

            del_car_id_list = []
            for car_id in car_path_dict:
                if car_id not in all_c:
                    del car_intersection_id_dict[car_id]
                    del_car_id_list.append(car_id)
            for car_id in del_car_id_list:
                del car_path_dict[car_id]

            for car_id in del_car_id_list:
                travel_time = simu_step-car_enter_time[car_id]
                car_travel_time.append(travel_time)

            for intersection_manager in intersection_manager_list:
                intersection_manager.run(simu_step)


            simu_step += cfg.TIME_STEP

    except Exception as e:
        traceback.print_exc()

    car_travel_time = car_travel_time	# Skip first 500 cars
    car_delay_time = car_delay_time

    avg_travel_time = (sum(car_travel_time)/len(car_travel_time))
    avg_delay_time = (sum(car_delay_time)/len(car_delay_time))
    served_car_num = (len(car_travel_time))
    actual_departure_rate = (float(served_car_num)/simu_step)
    actual_arrival_rate = (float(total_car_num)/simu_step)
    #print("Average delay: %f" % avg_travel_time)
    #print("Car number: %i" % (len(car_travel_time)))
    #print("Arrival rate: %f" % (len(car_travel_time)/600))
    sys.stdout.flush()
    sock.sendall("END@")

    traci.close()

    with open('../result/traveling_time.csv', 'a') as csvfile:
        file_writer = csv.writer(csvfile, lineterminator='\n')
        to_write_list = [sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6], sys.argv[7]]
        to_write_list.append(avg_travel_time)
        to_write_list.append(avg_delay_time)
        to_write_list.append(total_car_num)
        to_write_list.append(actual_arrival_rate)
        to_write_list.append(served_car_num)
        to_write_list.append(actual_departure_rate)
        file_writer.writerow(to_write_list)







###########################
# Main function
if __name__ == "__main__":
    print("  === Usage: python3 main.py <arrival_rate (0~1.0)> <seed> <grid size>")
    print("      Example: python3 main.py 0.2 0 2")
    sys.argv[3]

    cfg.INTER_SIZE = int(sys.argv[3])

    #HOST, PORT = "128.238.147.124", 9909

    seed = int(sys.argv[2])
    random.seed(seed)  # make tests reproducible
    numpy.random.seed(seed)

    # this script has been called from the command line. It will start sumo as a server, then connect and run
    sumoBinary = checkBinary('sumo-gui')

    # 1. Generate the route file for this simulation
    arrival_rate = sys.argv[1]

    generate_routefile_with_src_dst(cfg.INTER_SIZE, float(arrival_rate), seed, simulation_time)

    # Load from the file
    '''
    src_dst_file_name = "%i_%s_%i_src_dst.json" % (cfg.INTER_SIZE, arrival_rate, seed)
    with open('data/routes/'+src_dst_file_name) as json_file:
        src_dst_dict = json.load(json_file)
    '''


    try:
        # 3. This is the normal way of using traci. sumo is started as a subprocess and then the python script connects and runs

        net_name = "lane%iby%ione_lane.net.xml" % (cfg.INTER_SIZE, cfg.INTER_SIZE)
        route_name = "%i_%s_%i.rou.xml" % (cfg.INTER_SIZE, arrival_rate, seed)
        traci.start([sumoBinary, "-c", "data/UDTA.sumocfg",
                                 "--tripinfo-output", "tripinfo.xml","--step-length", str(cfg.TIME_STEP),
                                 "--collision.mingap-factor", "0",
                                 "-n", "data/net/" + net_name,
                                 "-r", "data/routes/" + route_name])

        run()
    except Exception as e:
        traceback.print_exc()
