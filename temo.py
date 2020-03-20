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
import threading
import time
import matplotlib.pyplot as plt
import numpy as np

from ortools.linear_solver import pywraplp
from sumolib import checkBinary
import traci

import config as cfg
from milp import Icacc, IcaccPlus, Fcfs, FixedSignal
from gen_route import generate_routefile


# For debug
from playsound import playsound


from Cars import Car
from LaneAdviser import LaneAdviser, PreAssignedLaneAdvise, GreedyAdvise, HybridAdvise_min, HybridAdvise, HybridAdvise_avg




in_lanes = []
out_lanes = []

CCZ_detecter = []
BZ_detecter = []
GZ_detecter = []
PZ_detecter = []
AZ_detecter = []



###################


def debug_ring():
    playsound('ring.mp3')


def run():
    """execute the TraCI control loop"""
    simu_step = 0

    update_count = 0    # Counter to see how often we should trigger the scheduling
    scheduling_thread = None

    car_list = dict()   # Cars that needs to be handled
    cc_list = dict()    # Cars under Cruse Control in CCZ
    leaving_cars = dict()   # Cars just entered the intersection (leave the CC zone)

    lane_advisor = LaneAdviser()


    # Statistics
    total_delays = 0
    car_num = 0


    while traci.simulation.getMinExpectedNumber() > 0:

        if (simu_step*10)//1/10.0 == 4000:
            #debug_t = threading.Thread(target=debug_ring)
            #debug_t.start()
            wait = raw_input("PRESS ENTER TO CONTINUE.")


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

            # Only checking at the cars in in-coming lanes (Name in SUMO simulator)
            if lane_id in in_lanes:

                # Add car if the car is not in the list yet
                if car_id not in car_list:
                    # Gather the information of the new car
                    traci.vehicle.setSpeed(car_id, cfg.MAX_SPEED)
                    length = traci.vehicle.getLength(car_id)
                    lane = ((4-int(lane_id[0]))*cfg.LANE_NUM_PER_DIRECTION) + (cfg.LANE_NUM_PER_DIRECTION-int(lane_id[2])-1)
                    turning = car_id[0]
                    new_car = Car(car_id, length, lane, turning)
                    car_list[car_id] = new_car


                # Set the position of each cars
                position = cfg.AZ_LEN + cfg.PZ_LEN + cfg.GZ_LEN+ cfg.BZ_LEN + cfg.CCZ_LEN - traci.vehicle.getLanePosition(car_id)
                car_list[car_id].setPosition(position)



        # Entering the intersection (Record the cars)
        for car_id, car in car_list.items():
            lane_id = traci.vehicle.getLaneID(car_id)
            if lane_id not in in_lanes:
                traci.vehicle.setSpeed(car_id, car.speed_in_int)


                # DEBUG
                if car.D+car.OT <= -0.2 or car.D+car.OT >= 0.2:
                    print("DEBUG: ", car_id, car.D+car.OT, car.D)

                cc_list.pop(car_id)


                leaving_cars[car_id] = car_list[car_id]
                car_list.pop(car_id)

        # Leaving the intersection (Reset the speed to V_max)
        for car_id, car in leaving_cars.items():
            lane_id = traci.vehicle.getLaneID(car_id)
            if lane_id in out_lanes:
                traci.vehicle.setSpeed(car_id, cfg.MAX_SPEED)
                del leaving_cars[car_id]




        ##########################################
        # Cruse Control

        # Start to let cars control itself once it enters the CCZ
        for lane in CCZ_detecter:
            if traci.inductionloop.getLastStepVehicleNumber(lane) > 0:
                Veh = traci.inductionloop.getLastStepVehicleIDs(lane)
                car_id = Veh[0]
                car = car_list[car_id]

                if car.CC_stage == None:
                    # Wait if the scheduling hasn't finished yet (To sync with the SUMO simulation)
                    if type(car.D) != float:
                        scheduling_thread.join()

                    # Getting ready for CC
                    car.CC_stage = '0 Not Handled Yet'

                    # Put car into the CC zontrol
                    if not cc_list.has_key(car_id):
                        cc_list[car_id] = car


                    # Measurement
                    total_delays += car.D
                    car_num += 1


        # Each car perform their own Cruise Control behavior
        for car_id, car in cc_list.items():
            # Cars perform their own CC
            car.handle_CC_behavior(car_list)
        #######################################################





        simu_step += cfg.TIME_STEP



        ##############################################
        # Grouping the cars and schedule
        # Put here due to the thread handling
        update_count += 1
        if update_count > (1/cfg.TIME_STEP)*cfg.GZ_LEN/cfg.MAX_SPEED-1:
            if scheduling_thread == None or (not scheduling_thread.isAlive()):

                # Classify the cars for scheduler
                sched_car = []
                n_sched_car = []
                for car_id, car in car_list.items():
                    if car.position <= cfg.CCZ_LEN + cfg.BZ_LEN + cfg.GZ_LEN:
                        if car.D is None:
                            n_sched_car.append(car_list[car_id])
                        else:
                            sched_car.append(car_list[car_id])


                for c_idx in range(len(n_sched_car)):
                    traci.vehicle.setColor(n_sched_car[c_idx].ID, (100,250,92))
                    n_sched_car[c_idx].D = None

                scheduling_thread = threading.Thread(target = Scheduling, args = (lane_advisor, sched_car, n_sched_car,))
                scheduling_thread.start()


                update_count = 0

            else:
                print("Warning: the update period does not sync with the length of GZ")



        ################################################
        # Set Max Speed in PZ
        for lane in PZ_detecter:
            if traci.inductionloop.getLastStepVehicleNumber(lane) > 0:
                Veh = traci.inductionloop.getLastStepVehicleIDs(lane)
                car_id = Veh[0]

                # Take over the speed control from the car
                traci.vehicle.setSpeedMode(car_id, 0)
                traci.vehicle.setSpeed(car_id, cfg.MAX_SPEED)

                lane_id = traci.vehicle.getLaneID(car_id)
                lane = ((4-int(lane_id[0]))*cfg.LANE_NUM_PER_DIRECTION) + (cfg.LANE_NUM_PER_DIRECTION-int(lane_id[2])-1)
                car_list[car_id].lane = lane

                # Stay on its lane
                traci.vehicle.changeLane(car_id, int(lane_id[2]), 1)
        ################################################
        # Change lane in AZ
        for lane in AZ_detecter:
            if traci.inductionloop.getLastStepVehicleNumber(lane) > 0:
                Veh = traci.inductionloop.getLastStepVehicleIDs(lane)
                car_id = Veh[0]

                traci.vehicle.setLaneChangeMode(car_id, 0)
                # 256 (collision avoidance) or 512 (collision avoidance and safety-gap enforcement)


                time_in_AZ = cfg.AZ_LEN/cfg.MAX_SPEED * 2 # Set up the maximum time for lane changing

                #advised_lane = GreedyAdvise(car_list[car_id], lane_advisor)
                #advised_lane = PreAssignedLaneAdvise(car_list[car_id])
                #advised_lane = HybridAdvise_min(car_list[car_id], lane_advisor)
                #traci.vehicle.changeLane(car_id, advised_lane, time_in_AZ)


    # Print out the measurements
    print("Average delay: ", total_delays/car_num)

    traci.close()


    sys.stdout.flush()



##########################
# Scheduling thread that handles scheduling and update the table for lane advising
def Scheduling(lane_advisor, sched_car, n_sched_car):
    IcaccPlus(sched_car, n_sched_car)
    lane_advisor.update_table(n_sched_car)
##########################




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

    random.seed(42)  # make tests reproducible

    options = get_options()

    # this script has been called from the command line. It will start sumo as a server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # 0. Generate the intersection information files
    os.system("bash gen_intersection/gen_data.sh " + str(cfg.LANE_NUM_PER_DIRECTION))

    # 1. Generate the route file for this simulation
    generate_routefile()

    # 2. Setup lane ID
    for idx in range(1,5):
        for jdx in range(cfg.LANE_NUM_PER_DIRECTION):
            idx_str = str(idx)+'_'+str(jdx)
            in_lanes.append(idx_str)
            out_lanes.append('-'+idx_str)

            CCZ_detecter.append('d' + idx_str)
            BZ_detecter.append('od' + idx_str)
            GZ_detecter.append('ood' + idx_str)
            PZ_detecter.append('oood' + idx_str)
            AZ_detecter.append('ooood' + idx_str)


    # 3. This is the normal way of using traci. sumo is started as a subprocess and then the python script connects and runs

    traci.start([sumoBinary, "-c", "data/icacc+.sumocfg",
                             "--tripinfo-output", "tripinfo.xml","--step-length", str(cfg.TIME_STEP),
                             "--collision.mingap-factor", "0"])

    # 4. Start running SUMO
    run()
