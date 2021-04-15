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
import csv

from gen_route import generate_routefile


# For debug
#from playsound import playsound

from IntersectionManager import IntersectionManager
#from myGraphic import Gui
#import myGraphic


#myGraphic.gui = Gui()

###################

vehNr = 0

def run():
    """execute the TraCI control loop"""
    simu_step = 0

    intersections = [IntersectionManager("00%i"%(i) + '_' + "001") for i in range(1, 4)]

    if sys.argv[4] == "T":
        intersections[0].connect(1, intersections[1], 3)
        intersections[1].connect(1, intersections[2], 3)
    else:
        pass

    turning_track_dict = dict()

    file_name = "utility_time_"
    file_name += sys.argv[1] + "_" + sys.argv[2] + "_" + sys.argv[3] + "_" + sys.argv[4]
    file_name += '.csv'
    #csvfile = open('./result/'+file_name, 'a')


    try:
        # Record car info
        car_info = dict()

        while traci.simulation.getMinExpectedNumber() > 0:

            if (simu_step*10)//1/10.0 == cfg.N_TIME_STEP:
                break
            #'''
            #if 'L_1383' in intersection_manager.car_list:
            #if (simu_step*10)//1/10.0 == 900:
                '''
                car = intersections[1].car_list['LR_808']
                print(car.ID, car.zone, car.zone_state, car.CC_state)

                car = intersections[1].car_list['SR_742']
                print(car.ID, car.zone, car.zone_state, car.CC_state)
                '''

                #raw_input()
            #'''
            '''
            if (simu_step*10)//1/10.0 == 105:
                print("check RR_311 RS_317  ?   (If not work, change back to max)")
                input('Enter enter:')
            #'''

            '''
            if simu_step > 1534:
                car = intersections[1].car_list['LS_2608']
                print(car.ID, car.zone, car.zone_state, car.CC_state, car.CC_get_front_speed())

                car = intersections[1].car_list['LS_2575']
                print(car.ID, car.zone, car.zone_state, car.CC_state, car.CC_get_front_speed())

            #'''

            traci.simulationStep()
            all_c = traci.vehicle.getIDList()
            # Update the position of each car
            for car_id in all_c:

                lane_id = traci.vehicle.getLaneID(car_id)

                # Dummy: Generate routes (turnings)
                if not car_id in car_info:
                    car_info[car_id] = dict()
                    car_info[car_id]["route"] = []
                    path = car_id.split('_')[0]
                    for turn in path:
                        car_info[car_id]["route"].append(turn)
                    car_info[car_id]["route"] += ["X", "X"]


                is_handled = False
                for intersection_manager in intersections:
                    if (intersection_manager.check_in_my_region(lane_id) == "On my lane"):

                        current_turn = car_info[car_id]["route"][0]
                        next_turn = car_info[car_id]["route"][1]

                        intersection_manager.update_car(car_id, lane_id, simu_step, current_turn, next_turn)
                        is_handled = True
                        car_info[car_id]["inter_status"] = "On my lane"

                    elif (intersection_manager.check_in_my_region(lane_id) == "In my intersection"):

                        # Check if the car enter the intersection (by changing state from "On my lane" to "in intersection")

                        if (car_info[car_id]["inter_status"] == "On my lane"):
                            car_info[car_id]["route"].pop(0)
                        current_turn = car_info[car_id]["route"][0]
                        next_turn = car_info[car_id]["route"][1]

                        intersection_manager.update_car(car_id, lane_id, simu_step, current_turn, next_turn)
                        is_handled = True
                        car_info[car_id]["inter_status"] = "In my intersection"

                    else:   # The intersection doesn't have the car
                        intersection_manager.delete_car(car_id)

                if not is_handled:
                    # Leaving intersections
                    traci.vehicle.setSpeed(car_id, cfg.MAX_SPEED)
                    car_info[car_id]["inter_status"] = "None"


            # Remove cars
            car_to_delete = []
            for car_id in car_info:
                if not car_id in all_c:
                    car_to_delete.append(car_id)
            for car_id in car_to_delete:
                del car_info[car_id]


            for intersection_manager in intersections:
                intersection_manager.run(simu_step)

            #lane_utility[0].append((simu_step, cfg.TOTAL_LEN-intersections[0].my_road_info[1]['avail_len']))
            #lane_utility[1].append((simu_step, cfg.TOTAL_LEN-intersections[1].my_road_info[3]['avail_len']))

            '''
            file_writer = csv.writer(csvfile, lineterminator='\n')

            to_write_list = [simu_step]
            for lane_idx in range(cfg.LANE_NUM_PER_DIRECTION):
                my_lane = 1*cfg.LANE_NUM_PER_DIRECTION+lane_idx
                utility = (cfg.TOTAL_LEN-intersections[0].my_road_info[my_lane]['avail_len'])/cfg.TOTAL_LEN
                to_write_list.append(utility)

                my_lane = 3*cfg.LANE_NUM_PER_DIRECTION+lane_idx
                utility = (cfg.TOTAL_LEN-intersections[1].my_road_info[my_lane]['avail_len'])/cfg.TOTAL_LEN
                to_write_list.append(utility)

            file_writer.writerow(to_write_list)
            '''
            simu_step += cfg.TIME_STEP



    except Exception as e:
        traceback.print_exc()


    #debug_t = threading.Thread(target=debug_ring)
    #debug_t.start()
    print(sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), sys.argv[4])

    # Print out the measurements
    #print("Average total delay: ", total_delays/car_num)
    #print("Average delay by scheduling: ", total_delays_by_sche/car_num)
    print(intersection_manager.total_delays/intersection_manager.car_num, intersection_manager.total_delays_by_sche/intersection_manager.car_num, intersection_manager.car_num)

    print("avg_fuel = ",intersection_manager.total_fuel_consumption/intersection_manager.fuel_consumption_count)


    file_name2 = 'result/result.csv'

    with open(file_name2, 'a', newline='') as csvfile2:
        writer2 = csv.writer(csvfile2, dialect='excel-tab', quoting=csv.QUOTE_MINIMAL, delimiter = ',')
        to_write2 = [sys.argv[1], sys.argv[2], sys.argv[3],
                    sys.argv[4], "_", simu_step,
                    intersections[0].car_num,
                    intersections[1].car_num,
                    vehNr,
                    vehNr/6
                    ]
        writer2.writerow(to_write2)

    sys.stdout.flush()

    traci.close()





##########################
# Setup running options for sumo
def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


###########################
# Main function
if __name__ == "__main__":
    print("Usage: python code.py <arrival_rate (0~1.0)> <seed> <schedular> <spillback T/F>")
    sys.argv[4]

    seed = int(sys.argv[2])
    random.seed(seed)  # make tests reproducible
    numpy.random.seed(seed)

    sumoBinary = checkBinary('sumo-gui')

    # 0. Generate the intersection information files
    os.system("bash gen_intersection/gen_data.sh " + str(cfg.LANE_NUM_PER_DIRECTION))

    # 1. Generate the route file for this simulation
    arrival_rate = float(sys.argv[1])
    vehNr = generate_routefile(arrival_rate)





    try:
        # 3. This is the normal way of using traci. sumo is started as a subprocess and then the python script connects and runs
        traci.start([sumoBinary, "-c", "data/icacc+.sumocfg",
                                 "--tripinfo-output", "tripinfo.xml","--step-length", str(cfg.TIME_STEP),
                                 "--collision.mingap-factor", "0",
                                 "--default.speeddev", "1",
                                 "--log", "error_log.txt",
                                 "--threads", "8"], numRetries=100)

        # 4. Start running SUMO
        run()
    except Exception as e:
        traceback.print_exc()
