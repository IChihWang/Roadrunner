from __future__ import absolute_import
from __future__ import print_function


import os
import sys

#sys.path.append('/usr/share/sumo/tools/')
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
from milp import Icacc, IcaccPlus, Fcfs, FixedSignal
from gen_route import generate_routefile
import csv




from Cars import Car



in_lanes = []
out_lanes = []

CCZ_detecter = []
BZ_detecter = []
GZ_detecter = []
PZ_detecter = []
AZ_detecter = []


###################



def run():
    """execute the TraCI control loop"""
    simu_step = 0

    update_count = 0    # Counter to see how often we should trigger the scheduling
    scheduling_thread = None

    car_list = dict()   # Cars that needs to be handled
    cc_list = dict()    # Cars under Cruse Control in CCZ
    leaving_cars = dict()   # Cars just entered the intersection (leave the CC zone)
    az_list = dict()

    # Statistics
    total_delays = 0
    total_delays_by_sche = 0
    car_num = 0
    total_fuel_consumption = 0

    # For front car
    CC_last_cars_on_lanes = dict()
    for idx in range(4*cfg.LANE_NUM_PER_DIRECTION):
        CC_last_cars_on_lanes[idx] = None

    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            if (simu_step*10)//1/10.0 == cfg.N_TIME_STEP:
                break
            # if (simu_step*10)//1/10.0 == 1000:
                #debug_t = threading.Thread(target=debug_ring)
                #debug_t.start()
                # wait = raw_input("PRESS ENTER TO CONTINUE.")


            traci.simulationStep()
            all_c = traci.vehicle.getIDList()

            # Update the time OT
            for car_key in car_list:
                # Update when the car is scheduled
                if car_list[car_key].OT != None:
                    car_list[car_key].OT -= cfg.TIME_STEP


            # Update the position of each car
            for car_id in all_c:
                lane_id = traci.vehicle.getLaneID(car_id)
                total_fuel_consumption += traci.vehicle.getFuelConsumption(car_id)*cfg.TIME_STEP

                # Only checking at the cars in in-coming lanes (Name in SUMO simulator)
                if lane_id in in_lanes:

                    # Add car if the car is not in the list yet
                    if car_id not in car_list:
                        # Gather the information of the new car
                        #traci.vehicle.setSpeed(car_id, cfg.MAX_SPEED)
                        length = traci.vehicle.getLength(car_id)
                        lane = ((4-int(lane_id[0]))*cfg.LANE_NUM_PER_DIRECTION) + (cfg.LANE_NUM_PER_DIRECTION-int(lane_id[2])-1)
                        turning = car_id[0]

                        new_car = Car(car_id, length, lane, turning)
                        new_car.Enter_T = simu_step - (traci.vehicle.getLanePosition(car_id))/cfg.MAX_SPEED
                        car_list[car_id] = new_car


                    # Set the position of each cars
                    position = cfg.AZ_LEN + cfg.PZ_LEN + cfg.GZ_LEN+ cfg.BZ_LEN + cfg.CCZ_LEN - traci.vehicle.getLanePosition(car_id)
                    car_list[car_id].setPosition(position)


            # Entering the intersection (Record the cars)
            for car_id, car in car_list.copy().items():
                lane_id = traci.vehicle.getLaneID(car_id)
                if lane_id not in in_lanes:

                    leaving_cars[car_id] = car_list[car_id]
                    car_list[car_id].Leave_T = simu_step
                    total_delays += (car.Leave_T - car.Enter_T) - ((cfg.CCZ_LEN+cfg.GZ_LEN+cfg.BZ_LEN+cfg.PZ_LEN+cfg.AZ_LEN)/cfg.MAX_SPEED)# Measurement
                    car_num += 1
                    car_list.pop(car_id)

            # Leaving the intersection (Reset the speed to V_max)
            for car_id, car in leaving_cars.copy().items():
                lane_id = traci.vehicle.getLaneID(car_id)
                if lane_id in out_lanes:
                    traci.vehicle.setSpeed(car_id, cfg.MAX_SPEED)
                    del leaving_cars[car_id]


            simu_step += cfg.TIME_STEP



            ################################################
            # Change lane in AZ
            for lane in AZ_detecter:
                if traci.inductionloop.getLastStepVehicleNumber(lane) > 0:
                    Veh = traci.inductionloop.getLastStepVehicleIDs(lane)
                    car_id = Veh[0]
                    az_list[car_id] = car_list[car_id]

                    traci.vehicle.setSpeed(car_id, cfg.MAX_SPEED)

                    traci.vehicle.setMinGap(car_id, 1)
                    #traci.vehicle.setLaneChangeMode(car_id, 1557)




    except Exception as e:
        traceback.print_exc()
    #JC
    print("Average total delay: ", total_delays/car_num)
    print("Number of car: ", car_num)

    with open(file_name, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile, dialect='excel-tab', quoting=csv.QUOTE_MINIMAL, delimiter = ',')
        to_write = [sys.argv[1], sys.argv[2], "_", car_num,
                    total_delays/car_num,
                    total_fuel_consumption/car_num]
        writer.writerow(to_write)

    traci.close()
    sys.stdout.flush()

    return (simu_step*10)//1/10.0





###########################
# Main function
if __name__ == "__main__":

    seed = int(sys.argv[2])
    random.seed(seed)  # make tests reproducible
    numpy.random.seed(seed)

    sumoBinary = checkBinary('sumo')

    # 0. Generate the intersection information files
    os.system("bash gen_intersection/gen_data.sh " + str(cfg.LANE_NUM_PER_DIRECTION))

    arrival_rate = sys.argv[1]
    # 1. Generate the route file for this simulation
    generate_routefile(str(arrival_rate))

    # 2. Setup lane ID
    for idx in range(1,5):
        for jdx in range(cfg.LANE_NUM_PER_DIRECTION):
            idx_str = str(idx)+'_'+str(jdx)
            in_lanes.append(idx_str)
            out_lanes.append('-'+idx_str)



    try:
        # 3. This is the normal way of using traci. sumo is started as a subprocess and then the python script connects and runs
        traci.start([sumoBinary, "-c", "data/icacc+.sumocfg",
                                 "--tripinfo-output", "tripinfo.xml","--step-length", str(cfg.TIME_STEP),
                                 "--collision.mingap-factor", "0"])

        # 4. Start running SUMO
        testtime = run()
    except:
        None
