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
import argparse

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

total_car_num = 0
traci_connection = None

def run():
    """execute the TraCI control loop"""
    simu_step = 0

    intersection_manager = IntersectionManager(scheduler)


    try:
        while traci_connection.simulation.getMinExpectedNumber() > 0:

            if (simu_step*10)//1/10.0 == cfg.N_TIME_STEP:
                break

            traci_connection.simulationStep()
            all_c = traci_connection.vehicle.getIDList()
            # Update the position of each car
            for car_id in all_c:
                lane_id = traci_connection.vehicle.getLaneID(car_id)
                intersection_manager.update_car(car_id, lane_id, simu_step)

            is_slowdown_control = False
            if slow_down == 'T':
                is_slowdown_control = True

            intersection_manager.run(simu_step, is_slowdown_control)
            simu_step += cfg.TIME_STEP
    except Exception as e:
        traceback.print_exc()


    #debug_t = threading.Thread(target=debug_ring)
    #debug_t.start()
    print(arrival_rate, int(seed), int(scheduler), slow_down)

    # Print out the measurements
    #print("Average total delay: ", total_delays/car_num)
    #print("Average delay by scheduling: ", total_delays_by_sche/car_num)
    print(intersection_manager.total_delays/intersection_manager.car_num, intersection_manager.total_delays_by_sche/intersection_manager.car_num, intersection_manager.car_num)

    print("avg_fuel = ",intersection_manager.total_fuel_consumption/intersection_manager.car_num)

    file_name = 'result/result.csv'
    with open(file_name, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile, dialect='excel-tab', quoting=csv.QUOTE_MINIMAL, delimiter = ',')
        to_write = [arrival_rate, seed, scheduler,
                    slow_down, comm_delay_handle, "_", simu_step, total_car_num, intersection_manager.car_num,
                    intersection_manager.total_delays/intersection_manager.car_num,
                    intersection_manager.total_delays_by_sche/intersection_manager.car_num,
                    intersection_manager.total_fuel_consumption/intersection_manager.car_num,
                    sum(intersection_manager.schedule_time)/len(intersection_manager.schedule_time),
                    sum(intersection_manager.advice_time)/len(intersection_manager.advice_time),
                    sum(intersection_manager.CControl_time)/len(intersection_manager.CControl_time)]
        writer.writerow(to_write)

    sys.stdout.flush()

    traci_connection.close()





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
    # Define parameter parser
    parser = argparse.ArgumentParser(prog='PROG')
    parser.add_argument('--rate', nargs='?', help='arrival_rate (0~1.0)', default=0.1)
    parser.add_argument('--seed', nargs='?', help='seed', default=0)
    parser.add_argument('--scheduler', nargs='?', help='0: Roadrunner, 1: ICCID, 2: FCFS, 3: FCFT', default=0)
    parser.add_argument('--slow_down', nargs='?', help='is_slowdown_control <T/F>', default='T')
    parser.add_argument('--comm_delay_handle', nargs='?', help='communication delay fed into the scheduler', default=0)
    parser.add_argument('--gui', nargs='?', help='Enable GUI for simulation <T/F>', default='F')
    args = parser.parse_args()

    global seed
    global arrival_rate
    global scheduler
    global slow_down
    global comm_delay_handle
    global gui

    seed = int(args.seed)
    arrival_rate = float(args.rate)
    scheduler = int(args.scheduler)
    slow_down = args.slow_down
    comm_delay_handle = int(args.comm_delay_handle)
    gui = args.gui


    random.seed(seed)  # make tests reproducible
    numpy.random.seed(seed)
    cfg.SCHEDULE_LOSS_PROBABILITY = 0
    cfg.CONTROL_DELAY_PROBABILITY = 0
    cfg.COMM_DELAY_STEPS = 0
    cfg.COMM_DELAY_S =  0.001*int(comm_delay_handle)
    cfg.COMM_DELAY_DIS = cfg.COMM_DELAY_S*cfg.MAX_SPEED
    cfg.HEADWAY += cfg.COMM_DELAY_DIS
    print("Headway: ", cfg.HEADWAY)

    # this script has been called from the command line. It will start sumo as a server, then connect and run
    if gui == 'F':
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # 0. Generate the intersection information files
    os.system("bash gen_intersection/gen_data.sh " + str(cfg.LANE_NUM_PER_DIRECTION))

    # 1. Generate the route file for this simulation
    total_car_num = len(generate_routefile(arrival_rate))


    try:
        # 3. This is the normal way of using traci. sumo is started as a subprocess and then the python script connects and runs
        traci.start([sumoBinary, "-c", "data/icacc+.sumocfg",
                                 "--tripinfo-output", "tripinfo.xml","--step-length", str(cfg.TIME_STEP),
                                 "--collision.mingap-factor", "0"], port=9091, label="vehicle_control")
        traci_connection = traci.getConnection("vehicle_control")
        traci_connection.setOrder(2)

        # 4. Start running SUMO
        run()
    except:
        None
