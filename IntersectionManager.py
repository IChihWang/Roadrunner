
import sys
import config as cfg
import traci
import threading
import time
import random

from Cars import Car
from milp import Icacc, IcaccPlus, Fcfs, FixedSignal, Fcfs_not_reservation
from LaneAdviser import LaneAdviser
from get_inter_length_info import Data

inter_length_data = Data()

class IntersectionManager:
    def __init__(self):
        self.az_list = dict()
        self.pz_list = dict()
        self.ccz_list = dict()

        self.car_list = dict()   # Cars that needs to be handled
        self.cc_list = dict()    # Cars under Cruse Control in CCZ
        self.leaving_cars = dict()   # Cars just entered the intersection (leave the CC zone)

        self.schedule_period_count = 0
        self.lane_advisor = LaneAdviser()
        self.scheduling_thread = None
        self.in_lanes = []
        self.out_lanes = []

        # For front car
        self.CC_last_cars_on_lanes = dict()
        for idx in range(4*cfg.LANE_NUM_PER_DIRECTION):
            self.CC_last_cars_on_lanes[idx] = None


        # Statistics
        self.total_delays = 0
        self.total_delays_by_sche = 0
        self.car_num = 0

        self.total_fuel_consumption = 0
        self.fuel_consumption_count = 0

        # Pedestrian control
        self.is_pedestrian_list = [False]*4         # Whether there is a pedestrian request
        self.pedestrian_time_mark_list = [None]*4      # Planned pedestrian time (In case some cars insterted and interrupt the pedestiran time)

        self.schedule_time = []
        self.advice_time = []
        self.CControl_time = []


        self.set_round_lane()


    def set_round_lane(self):
        for idx in range(1,5):
            for jdx in range(cfg.LANE_NUM_PER_DIRECTION):
                idx_str = str(idx)+'_'+str(jdx)
                self.in_lanes.append(idx_str)
                self.out_lanes.append('-'+idx_str)


    def update_car(self, car_id, lane_id, simu_step):
        if lane_id in self.in_lanes:
            lane = ((4-int(lane_id[0]))*cfg.LANE_NUM_PER_DIRECTION) + (cfg.LANE_NUM_PER_DIRECTION-int(lane_id[2])-1)

            # Add car if the car is not in the list yet
            if car_id not in self.car_list:
                # Gather the information of the new car
                #traci.vehicle.setSpeed(car_id, cfg.MAX_SPEED)
                length = traci.vehicle.getLength(car_id)
                turning = car_id[0]

                new_car = Car(car_id, length, lane, turning)
                new_car.Enter_T = simu_step - (traci.vehicle.getLanePosition(car_id))/cfg.MAX_SPEED
                self.car_list[car_id] = new_car

            # Set the position of each cars
            position = cfg.AZ_LEN + cfg.PZ_LEN + cfg.GZ_LEN+ cfg.BZ_LEN + cfg.CCZ_LEN - traci.vehicle.getLanePosition(car_id)
            self.car_list[car_id].setPosition(position)


            if self.car_list[car_id].zone == None:
                self.car_list[car_id].zone = "AZ"
                self.car_list[car_id].zone_state = "AZ_not_advised"

            elif (self.car_list[car_id].zone == "AZ") and (position <= cfg.PZ_LEN + cfg.GZ_LEN + cfg.BZ_LEN + cfg.CCZ_LEN):
                self.car_list[car_id].zone = "PZ"
                self.car_list[car_id].zone_state = "PZ_not_set"

            elif (self.car_list[car_id].zone == "PZ") and (position <= cfg.GZ_LEN + cfg.BZ_LEN + cfg.CCZ_LEN):
                self.car_list[car_id].zone = "GZ"
                self.car_list[car_id].zone_state = "not_scheduled"

            elif (self.car_list[car_id].zone == "GZ") and (position <= cfg.BZ_LEN + cfg.CCZ_LEN):
                self.car_list[car_id].zone = "BZ"

            elif (self.car_list[car_id].zone == "BZ") and (position <= cfg.CCZ_LEN):
                self.car_list[car_id].zone = "CCZ"


    def run(self, simu_step, is_slowdown_control):

        # ===== Update the time OT =====
        for car_key in self.car_list:
            # Update when the car is scheduled
            if self.car_list[car_key].OT != None:
                self.car_list[car_key].OT -= cfg.TIME_STEP

            self.total_fuel_consumption += traci.vehicle.getFuelConsumption(car_key)*cfg.TIME_STEP
            self.fuel_consumption_count += 1

        # ===== Entering the intersection (Record the cars) =====
        for car_id, car in self.car_list.items():
            lane_id = traci.vehicle.getLaneID(car_id)
            if lane_id not in self.in_lanes:
                traci.vehicle.setSpeed(car_id, car.speed_in_intersection)

                self.leaving_cars[car_id] = self.car_list[car_id]
                if self.car_list[car_id].Leave_T == None:
                    self.car_list[car_id].Leave_T = simu_step

                    if car.D+car.OT <= -0.4 or car.D+car.OT >= 0.4:
                        print("DEBUG: Car didn't arrive at the intersection at right time.")

                        print("ID", car.ID)
                        print("OT+D", car.D+car.OT)
                        print("lane", car.lane)
                        print("D", car.D)
                        print("OT", car.OT)
                        print("=======")
                        print("-----------------")


        # ===== Leaving the intersection (Reset the speed to V_max) =====
        to_be_deleted = []
        for car_id, car in self.leaving_cars.items():
            lane_id = traci.vehicle.getLaneID(car_id)
            if lane_id in self.out_lanes:
                traci.vehicle.setSpeed(car_id, cfg.MAX_SPEED)
                to_be_deleted.append(car_id)

        for car_id in to_be_deleted:
            del self.leaving_cars[car_id]
            del self.ccz_list[car_id]
            self.car_list.pop(car_id)

            self.total_delays += (car.Leave_T - car.Enter_T) - ((cfg.CCZ_LEN+cfg.GZ_LEN+cfg.BZ_LEN+cfg.PZ_LEN+cfg.AZ_LEN)/cfg.MAX_SPEED)
            # Measurement
            self.total_delays_by_sche += car.D
            self.car_num += 1



        # ===== Starting Cruise control
        to_be_deleted = []
        for car_id, car in self.pz_list.items():
            if car.position <= cfg.CCZ_LEN and isinstance(car.D, float):

                self.ccz_list[car_id] = car
                to_be_deleted.append(car_id)

                if is_slowdown_control == True:
                    if (car.CC_state == "Preseting_done"):
                        car.CC_state = "CruiseControl_ready"

        for car_id in to_be_deleted:
            del self.pz_list[car_id]




        ##############################################
        # Grouping the cars and schedule
        # Put here due to the thread handling
        self.schedule_period_count += cfg.TIME_STEP
        if self.schedule_period_count > cfg.GZ_LEN/cfg.MAX_SPEED -1:
            if self.scheduling_thread == None or (not self.scheduling_thread.is_alive()):

                # Classify the cars for scheduler
                sched_car = []
                n_sched_car = []
                advised_n_sched_car = []
                for car_id, car in self.car_list.items():
                    if car.zone == "GZ" or car.zone == "BZ" or car.zone == "CCZ":
                        if isinstance(car.D, float) and (not car.need_reschedule):
                            sched_car.append(car)
                        else:
                            n_sched_car.append(car)
                    elif car.zone == "PZ" or car.zone == "AZ":
                        advised_n_sched_car.append(car)


                for car in n_sched_car:
                    car.D = None
                ori_n_sched_car = n_sched_car

                # Setting the pedestrian list
                self.is_pedestrian_list = [True]*4
                for direction in range(4):
                    # Cancel the request if a pedestrian time has been scheduled
                    if self.is_pedestrian_list[direction] == True and self.pedestrian_time_mark_list[direction] != None:
                        self.is_pedestrian_list[direction] = False
                self.pedestrian_time_mark_list = self.get_max_AT_direction(sched_car, self.is_pedestrian_list, self.pedestrian_time_mark_list)
                #print(self.pedestrian_time_mark_list)

                Scheduling(self.lane_advisor,
                        sched_car, n_sched_car,
                        advised_n_sched_car,
                        self.cc_list,
                        self.car_list,
                        self.pedestrian_time_mark_list,
                        self.schedule_period_count,
                        self.schedule_time)

                for car in ori_n_sched_car:
                    if car.is_control_delay:
                        traci.vehicle.setColor(car.ID, (255,128,0))
                    elif car.is_error:
                        traci.vehicle.setColor(car.ID, (255,0,0))
                    elif not car.need_reschedule:
                        traci.vehicle.setColor(car.ID, (100,250,92))
                    else:
                        traci.vehicle.setColor(car.ID, (255,51,255))


                self.schedule_period_count = 0

            else:
                print("Warning: the update period does not sync with the length of GZ")






        ################################################
        # Set Max Speed in PZ
        to_be_deleted = []
        for car_id, car in self.az_list.items():
            if car.zone == "PZ" and car.zone_state == "PZ_not_set":
                traci.vehicle.setMinGap(car_id, cfg.HEADWAY)
                self.pz_list[car_id] = car
                to_be_deleted.append(car_id)

                # Take over the speed control from the car
                traci.vehicle.setSpeedMode(car_id, 0)
                car.CC_state = "Preseting_start"

                car.zone_state = "PZ_set"

        for car_id in to_be_deleted:
            del self.az_list[car_id]




        ##########################################
        # Cruse Control

        # Start to let cars control itself once it enters the CCZ
        # Each car perform their own Cruise Control behavior
        ccontrol_list = self.pz_list.copy()
        ccontrol_list.update(self.ccz_list)
        sorted_ccontrol_list = sorted(ccontrol_list.items(), key=lambda x: x[1].position)
        # SUPER IMPORTANT: sorted to ensure the following car speed
        for car_id, car in sorted_ccontrol_list:
            start = time.time()
            # Cars perform their own CC
            car.handle_CC_behavior(self.car_list)
            end = time.time()
            self.CControl_time.append(end - start)


        ################################################
        # Change lane in AZ
        for car_id, car in self.car_list.items():
            if car.zone == "AZ" and car.zone_state == "AZ_not_advised":
                self.az_list[car_id] = car

                traci.vehicle.setMinGap(car_id, cfg.HEADWAY)
                #traci.vehicle.setLaneChangeMode(car_id, 256)
                traci.vehicle.setLaneChangeMode(car_id, 272)
                # 256 (collision avoidance) or 512 (collision avoidance and safety-gap enforcement)

                time_in_AZ = cfg.AZ_LEN/cfg.MAX_SPEED *3

                start = time.time()
                #advised_lane = self.lane_advisor.adviseLaneShortestTrajectory(car)
                advised_lane = self.lane_advisor.adviseLane(self.car_list[car_id])
                #advised_lane = self.lane_advisor.adviseLane_v2(self.car_list[car_id])
                #advised_lane = random.randrange(0, cfg.LANE_NUM_PER_DIRECTION)
                end = time.time()

                self.advice_time.append(end - start)

                traci.vehicle.changeLane(car_id, advised_lane, time_in_AZ)
                car.desired_lane = (cfg.LANE_NUM_PER_DIRECTION-advised_lane-1)+(car.lane//cfg.LANE_NUM_PER_DIRECTION)*cfg.LANE_NUM_PER_DIRECTION
                #self.car_list[car_id].desired_lane = car.lane

                car.zone_state = "AZ_advised"

            elif car.zone == "AZ" and car.zone_state == "AZ_advised" and car.position <= cfg.PZ_LEN + cfg.GZ_LEN + cfg.BZ_LEN + cfg.CCZ_LEN + cfg.CCZ_ACC_LEN:
                leader_tuple = traci.vehicle.getLeader(car.ID)
                if leader_tuple != None:
                    if leader_tuple[0] in self.car_list.keys():
                        front_car_ID = leader_tuple[0]
                        front_car = self.car_list[front_car_ID]
                        front_distance = leader_tuple[1]

                        my_speed = traci.vehicle.getSpeed(car.ID)
                        front_speed = traci.vehicle.getSpeed(front_car.ID)
                        min_catch_up_time = (my_speed-front_speed)/cfg.MAX_ACC
                        min_distance = (my_speed-front_speed)*min_catch_up_time

                        min_gap = max(cfg.HEADWAY, min_distance+cfg.HEADWAY)
                        traci.vehicle.setMinGap(car_id, min_gap)


                # Cancel the auto gap
                traci.vehicle.setLaneChangeMode(car_id, 512)
                # Keep the car on the same lane
                lane_id = traci.vehicle.getLaneID(car_id)
                lane = int(((4-int(lane_id[0]))*cfg.LANE_NUM_PER_DIRECTION) + (cfg.LANE_NUM_PER_DIRECTION-int(lane_id[2])-1))
                self.car_list[car_id].lane = lane
                # Stay on its lane
                traci.vehicle.changeLane(car_id, int(lane_id[2]), 1.0)



    # Compute teh max AT for pedestrian time
    def get_max_AT_direction(self, sched_car, is_pedestrian_list, pedestrian_time_mark_list):
        max_AT = [0]*4  # Four directions

        for car in sched_car:
            # Compute the direction of entering/exiting for the intersection
            in_dir = car.in_dir
            out_dir = car.out_dir

            # Compute arrival time at the exiting point and entering point
            in_AT = car.OT + car.D + car.length/car.speed_in_intersection
            out_AT = car.OT + car.D + car.length/cfg.MAX_SPEED + inter_length_data.getIntertime(car.lane, car.turning)

            # Find max and update
            if in_AT > max_AT[in_dir]:
                max_AT[in_dir] = in_AT
            if out_AT > max_AT[out_dir]:
                max_AT[out_dir] = out_AT

        for direction in range(4):
            if pedestrian_time_mark_list[direction] != None:
                max_AT[direction] = pedestrian_time_mark_list[direction]
            elif is_pedestrian_list[direction] == False:
                max_AT[direction] = None

        return max_AT




##########################
# Scheduling thread that handles scheduling and update the table for lane advising
def Scheduling(lane_advisor, sched_car, n_sched_car,
                advised_n_sched_car, cc_list, car_list,
                pedestrian_time_mark_list, schedule_period_count,
                schedule_time):

    start = time.time()
    if int(sys.argv[3]) == 0:
        IcaccPlus(sched_car, n_sched_car, pedestrian_time_mark_list)
    elif int(sys.argv[3]) == 1:
        Icacc(sched_car, n_sched_car)
    elif int(sys.argv[3]) == 2:
        Fcfs(sched_car, n_sched_car, pedestrian_time_mark_list)
    elif int(sys.argv[3]) == 3:
        Fcfs_not_reservation(sched_car, n_sched_car)

    lane_advisor.updateTableFromCars(n_sched_car, advised_n_sched_car)

    for car in n_sched_car:
        car.zone_state = "scheduled"

    # Update the pedestrian ime list
    for direction in range(4):
        if pedestrian_time_mark_list[direction] != None:
            pedestrian_time_mark_list[direction] -= schedule_period_count
        if pedestrian_time_mark_list[direction] < -cfg.PEDESTRIAN_TIME_GAP:
            pedestrian_time_mark_list[direction] = None


    end = time.time()
    schedule_time.append(end-start)


    #'''
    to_be_deleted = []
    for car in n_sched_car:
        car.need_reschedule = False

        if car.is_error == False:
            pass
        elif car.is_error == None and car.is_reschedule:
            # Reschedule is called by the Car
            car.is_error = False

        elif car.is_error == None or car.is_error == True:
            # Reschedule is called by the Noise
            if random.uniform(0, 1) < cfg.SCHEDULE_LOSS_PROBABILITY:
                car.is_error = True
                car.is_reschedule = True
                car.need_reschedule = True
                car.D = None
                to_be_deleted.append(car)
            else:
                car.is_error = False
    for car in to_be_deleted:
        n_sched_car.remove(car)
    #'''

    for car in n_sched_car:
        if not car.is_control_delay:
            if random.uniform(0, 1) < cfg.CONTROL_DELAY_PROBABILITY:
                car.is_control_delay = True
                car.original_delay = car.D
                car.D += random.uniform(0, 5)
