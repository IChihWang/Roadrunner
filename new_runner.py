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
from milp import Icacc, IcaccPlus, Fcfs, FixedSignal
from gen_route import generate_routefile


# For debug
from playsound import playsound


from Cars import Car
from LaneAdviser import LaneAdviser
#from myGraphic import Gui
#import myGraphic




in_lanes = []
out_lanes = []

CCZ_detecter = []
BZ_detecter = []
GZ_detecter = []
PZ_detecter = []
AZ_detecter = []

#myGraphic.gui = Gui()

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
    az_list = dict()

    lane_advisor = LaneAdviser()

    # Statistics
    total_delays = 0
    total_delays_by_sche = 0
    car_num = 0

    # For front car
    CC_last_cars_on_lanes = dict()
    for idx in range(4*cfg.LANE_NUM_PER_DIRECTION):
        CC_last_cars_on_lanes[idx] = None

    try:
        while traci.simulation.getMinExpectedNumber() > 0:

            if (simu_step*10)//1/10.0 == 600:
                break


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
            for car_id, car in car_list.items():
                lane_id = traci.vehicle.getLaneID(car_id)
                if lane_id not in in_lanes:
                    traci.vehicle.setSpeed(car_id, car.speed_in_int)

                    # DEBUG
                    '''
                    if car.D+car.OT <= -0.4 or car.D+car.OT >= 0.4:
                        print("DEBUG: Car didn't arrive at the intersection at right time.")

                        print("ID", car.ID)
                        print("OT+D", car.D+car.OT)
                        print("lane", car.lane)
                        print("D", car.D)
                        print("OT", car.OT)
                        print("CC_slow_speed", car.CC_slow_speed)
                        print("CC_affected_by_front", car.CC_affected_by_front)
                        print("CC_shift", car.CC_shift)
                        print("CC_shift_end", car.CC_shift_end)
                        print("CC_following", car.CC_following)
                        print("CC_stop_n_go", car.CC_stop_n_go)
                        print("CC_auto", car.CC_auto )
                        print("CC_auto_slow_down_speed", car.CC_auto_slow_down_speed)
                        print("=======")
                        print("-----------------")
                        #wait = raw_input("PRESS ENTER TO CONTINUE.")
                    #'''

                    cc_list.pop(car_id)


                    leaving_cars[car_id] = car_list[car_id]
                    car_list[car_id].Leave_T = simu_step
                    total_delays += (car.Leave_T - car.Enter_T) - ((cfg.CCZ_LEN+cfg.GZ_LEN+cfg.BZ_LEN+cfg.PZ_LEN+cfg.AZ_LEN)/cfg.MAX_SPEED)

                    # Measurement
                    total_delays_by_sche += car.D
                    car_num += 1

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

                    if car.CC_done_scheduling == True:
                        # Wait if the scheduling hasn't finished yet (To sync with the SUMO simulation)
                        '''
                        if type(car.D) != float:
                            scheduling_thread.join()
                        '''

                        '''
                        # Getting ready for CC
                        car.CC_stage = '0 Not Handled Yet'

                        # Put car into the CC zontrol
                        if not cc_list.has_key(car_id):
                            cc_list[car_id] = car
                        '''




            # Each car perform their own Cruise Control behavior
            for car_id, car in cc_list.items():
                # Cars perform their own CC
                car.handle_CC_behavior(car_list)



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
                    advised_n_sched_car = []
                    for car_id, car in car_list.items():
                        if car.position <= cfg.CCZ_LEN + cfg.BZ_LEN + cfg.GZ_LEN:
                            if car.D is None:
                                n_sched_car.append(car_list[car_id])
                            else:
                                sched_car.append(car_list[car_id])
                        elif car.position <= cfg.CCZ_LEN + cfg.BZ_LEN + cfg.GZ_LEN + cfg.PZ_LEN + cfg.AZ_LEN:
                            advised_n_sched_car.append(car_list[car_id])


                    for c_idx in range(len(n_sched_car)):
                        traci.vehicle.setColor(n_sched_car[c_idx].ID, (100,250,92))
                        n_sched_car[c_idx].D = None

                    scheduling_thread = threading.Thread(target = Scheduling, args = (lane_advisor, sched_car, n_sched_car, advised_n_sched_car, cc_list, car_list))
                    scheduling_thread.start()


                    update_count = 0

                else:
                    print("Warning: the update period does not sync with the length of GZ")



            ############################
            cars_need_handle = dict()
            available_space = dict()
            is_congested = dict()
            for idx in range(4*cfg.LANE_NUM_PER_DIRECTION):
                cars_need_handle[idx] = []
                available_space[idx] = cfg.CCZ_LEN
                is_congested[idx] = False

            for car_id, car in car_list.items():
                if car.CC_shift != None:
                    if available_space[car.lane] > max(car.CC_shift, cfg.CCZ_LEN-car.position):
                        available_space[car.lane] = max(car.CC_shift, cfg.CCZ_LEN-car.position)
                    if car.CC_shift < 0:
                        is_congested[car.lane] = True
                elif car.position > cfg.CCZ_LEN+cfg.BZ_LEN+cfg.GZ_LEN+cfg.PZ_LEN:
                    cars_need_handle[car.lane].append(car)

                ''' Debug
                if traci.vehicle.getSpeed(car_id) < 0.01 and car.position > cfg.CCZ_LEN+cfg.BZ_LEN+cfg.GZ_LEN+cfg.PZ_LEN and car.position < cfg.CCZ_LEN+cfg.BZ_LEN+cfg.GZ_LEN+cfg.PZ_LEN + 20:
                    print('-->  ', car_id, car.lane)
                    if car.CC_front_car != None:
                        print("      ", car.CC_front_car.ID)
                if car_id == 'L_294':
                    print('        -->    ', car_id, car.lane)
                    if car.CC_front_car != None:
                        print("            ", car.CC_front_car.ID)
                #'''

            for car_id, car in car_list.items():
                if car.CC_shift == None and car.position < cfg.CCZ_LEN+cfg.BZ_LEN+cfg.GZ_LEN+cfg.PZ_LEN:
                    available_space[car.lane] -= car.length+cfg.HEADWAY

            for idx in range(4*cfg.LANE_NUM_PER_DIRECTION):
                sorted_car_list = sorted(cars_need_handle[idx], key=lambda car: car.position)

                #print(idx, is_congested[idx], available_space[idx])
                for car in sorted_car_list:
                    if is_congested[idx] == True:
                        car.setHalting(True)
                    else:
                        if available_space[idx] > car.length:
                            car.setHalting(False)
                        else:
                            car.setHalting(True)

                        available_space[idx] -= (car.length + cfg.HEADWAY)




            ################################################
            # Set Max Speed in PZ

            for lane in PZ_detecter:
                if traci.inductionloop.getLastStepVehicleNumber(lane) > 0:
                    Veh = traci.inductionloop.getLastStepVehicleIDs(lane)
                    car_id = Veh[0]
                    if car_id in az_list:
                        del az_list[car_id]

                    # Take over the speed control from the car
                    traci.vehicle.setSpeedMode(car_id, 0)
                    traci.vehicle.setSpeed(car_id, cfg.MAX_SPEED)

                    # Cancel the auto gap
                    traci.vehicle.setLaneChangeMode(car_id, 0)

                    lane_id = traci.vehicle.getLaneID(car_id)
                    lane = ((4-int(lane_id[0]))*cfg.LANE_NUM_PER_DIRECTION) + (cfg.LANE_NUM_PER_DIRECTION-int(lane_id[2])-1)
                    car_list[car_id].lane = lane

                    # Stay on its lane
                    traci.vehicle.changeLane(car_id, int(lane_id[2]), 10.0)
                    '''
                    if car_list[car_id].lane != car_list[car_id].desired_lane:
                        traci.vehicle.setColor(car_id, (255,0,0))
                        print("ID", car_list[car_id].ID)
                        print("turning", car_list[car_id].turning)
                        print("desired_lane", car_list[car_id].desired_lane)
                        print("lane", car_list[car_id].lane)

                    #'''
                    '''
                    if car_list[car_id].desired_lane != car_list[car_id].lane:
                        print("ID", car_list[car_id].ID)
                        print("turning", car_list[car_id].turning)
                        print("desired_lane", car_list[car_id].desired_lane)
                        print("lane", car_list[car_id].lane)
                    #'''

                    if car_list[car_id].CC_front_car == None and CC_last_cars_on_lanes[lane] != None:
                        if CC_last_cars_on_lanes[lane].ID != car_list[car_id].ID:
                            car_list[car_id].CC_front_car = CC_last_cars_on_lanes[lane]

                    CC_last_cars_on_lanes[lane] = car_list[car_id]


            ################################################
            # Change lane in AZ
            for lane in AZ_detecter:
                if traci.inductionloop.getLastStepVehicleNumber(lane) > 0:
                    Veh = traci.inductionloop.getLastStepVehicleIDs(lane)
                    car_id = Veh[0]
                    az_list[car_id] = car_list[car_id]

                    traci.vehicle.setMinGap(car_id, 1)
                    traci.vehicle.setLaneChangeMode(car_id, 256)
                    # 256 (collision avoidance) or 512 (collision avoidance and safety-gap enforcement)

                    time_in_AZ = cfg.AZ_LEN/cfg.MAX_SPEED *3


                    advised_lane = lane_advisor.adviseLaneShortestTrajectory(car_list[car_id])
                    #advised_lane = lane_advisor.adviseLane(car_list[car_id])
                    #advised_lane = lane_advisor.adviseLane_v2(car_list[car_id])
                    #advised_lane = random.randrange(0, cfg.LANE_NUM_PER_DIRECTION)

                    traci.vehicle.changeLane(car_id, advised_lane, time_in_AZ)
                    car_list[car_id].desired_lane = (cfg.LANE_NUM_PER_DIRECTION-advised_lane-1)+(car_list[car_id].lane//cfg.LANE_NUM_PER_DIRECTION)*cfg.LANE_NUM_PER_DIRECTION
                    #car_list[car_id].desired_lane = car.lane
                    '''
                    if car_list[car_id].lane != car_list[car_id].desired_lane:
                        traci.vehicle.setColor(n_sched_car[c_idx].ID, (100,250,92))
                        print("lane", car_list[car_id].lane)
                        print("D", car_list[car_id].D)
                        print("OT", car_list[car_id].OT)
                        print("CC_slow_speed", car_list[car_id].CC_slow_speed)
                        print("CC_affected_by_front", car_list[car_id].CC_affected_by_front)
                        print("CC_stop_n_go", car_list[car_id].CC_stop_n_go)
                        print("CC_shift", car_list[car_id].CC_shift)
                        print("CC_shift_end", car_list[car_id].CC_shift_end)
                        print("=======")
                        '''


                    #myGraphic.gui.updateGraph()


            ###########################################################
            # Handle halting in AZ
            # Must be after PZ_len, because its function overwite some commands
            for id, car in az_list.items():
                car.handleHalting()


                lane_id = traci.vehicle.getLaneID(id)
                lane = ((4-int(lane_id[0]))*cfg.LANE_NUM_PER_DIRECTION) + (cfg.LANE_NUM_PER_DIRECTION-int(lane_id[2])-1)
                car_list[id].lane = lane


            ############################################
            ## DEBUG:
            '''
            debug_car_id = 'S_789'
            if debug_car_id in car_list:
                print("ID", car_list[debug_car_id].ID)
                print("lane", car_list[debug_car_id].lane)
                print("D", car_list[debug_car_id].D)
                print("OT", car_list[debug_car_id].OT)
                print("CC_slow_speed", car_list[debug_car_id].CC_slow_speed)
                print("CC_affected_by_front", car_list[debug_car_id].CC_affected_by_front)
                print("CC_stop_n_go", car_list[debug_car_id].CC_stop_n_go)
                print("CC_shift", car_list[debug_car_id].CC_shift)
                print("CC_shift_end", car_list[debug_car_id].CC_shift_end)
                print("=======")
            #'''
            ############################################





    except Exception as e:
        traceback.print_exc()


    #debug_t = threading.Thread(target=debug_ring)
    #debug_t.start()
    print(sys.argv[1], int(sys.argv[2]), int(sys.argv[3]))

    # Print out the measurements
    #print("Average total delay: ", total_delays/car_num)
    #print("Average delay by scheduling: ", total_delays_by_sche/car_num)
    print(total_delays/car_num, total_delays_by_sche/car_num, car_num)

    sys.stdout.flush()

    traci.close()



##########################
# Scheduling thread that handles scheduling and update the table for lane advising
def Scheduling(lane_advisor, sched_car, n_sched_car, advised_n_sched_car, cc_list, car_list):
    if int(sys.argv[3]) == 0:
        IcaccPlus(sched_car, n_sched_car)
    elif int(sys.argv[3]) == 1:
        Icacc(sched_car, n_sched_car)
    elif int(sys.argv[3]) == 2:
        Fcfs(sched_car, n_sched_car)


    lane_advisor.updateTableFromCars(n_sched_car, advised_n_sched_car)

    # Handle the speed control computation
    n_cars_on_lanes = dict()
    for idx in range(4*cfg.LANE_NUM_PER_DIRECTION):
        n_cars_on_lanes[idx] = []

    # Handle car behavior and detect congestion
    for car in n_sched_car:
        # Getting ready for CC
        car.CC_stage = '0 Not Handled Yet'
        n_cars_on_lanes[car.lane].append(car)

        # Put car into the CC zontrol
        if not cc_list.has_key(car.ID):
            cc_list[car.ID] = car

    for idx in range(4*cfg.LANE_NUM_PER_DIRECTION):
        sorted_car_list = sorted(n_cars_on_lanes[idx], key=lambda car: car.position)

        for car in sorted_car_list:
            car.handle_CC_behavior(car_list)
            car.CC_done_scheduling = True














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
    print("Usage: python code.py <arrival_rate (0~1.0)> <seed>")

    seed = int(sys.argv[2])
    random.seed(seed)  # make tests reproducible
    numpy.random.seed(seed)

    options = get_options()

    # this script has been called from the command line. It will start sumo as a server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # 0. Generate the intersection information files
    os.system("bash gen_intersection/gen_data.sh " + str(cfg.LANE_NUM_PER_DIRECTION))

    # 1. Generate the route file for this simulation
    arrival_rate = float(sys.argv[1])
    generate_routefile(arrival_rate)

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



    try:
        # 3. This is the normal way of using traci. sumo is started as a subprocess and then the python script connects and runs
        traci.start([sumoBinary, "-c", "data/icacc+.sumocfg",
                                 "--tripinfo-output", "tripinfo.xml","--step-length", str(cfg.TIME_STEP),
                                 "--collision.mingap-factor", "0"])

        # 4. Start running SUMO
        run()
    except:
        None


    #myGraphic.gui.stopShowing()
