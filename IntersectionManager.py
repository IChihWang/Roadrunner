
import sys
import config as cfg
import traci
import threading
import random


from Cars import Car
from milp import Icacc, IcaccPlus, Fcfs, FixedSignal
from LaneAdviser import LaneAdviser


class IntersectionManager:
    def __init__(self, my_id):
        self.ID = my_id
        self.az_list = dict()
        self.pz_list = dict()
        self.ccz_list = dict()

        self.car_list = dict()   # Cars that needs to be handled
        self.cc_list = dict()    # Cars under Cruse Control in CCZ

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


        self.set_round_lane()


    def set_round_lane(self):
        for idx in range(1,5):
            for jdx in range(cfg.LANE_NUM_PER_DIRECTION):
                idx_str = self.ID + '_' + str(idx)+'_'+str(jdx)
                self.in_lanes.append(idx_str)

    def check_in_my_region(self, lane_id):
        if lane_id in self.in_lanes:
            return True
        else:
            return False


    def update_car(self, car_id, lane_id, simu_step):
        if lane_id in self.in_lanes:
            lane = ((4-int(lane_id[8]))*cfg.LANE_NUM_PER_DIRECTION) + (cfg.LANE_NUM_PER_DIRECTION-int(lane_id[10])-1)

            # Add car if the car is not in the list yet
            if car_id not in self.car_list:
                # Gather the information of the new car
                #traci.vehicle.setSpeed(car_id, cfg.MAX_SPEED)
                length = traci.vehicle.getLength(car_id)

                new_car = Car(car_id, length, lane, "R")
                new_car.Enter_T = simu_step - (traci.vehicle.getLanePosition(car_id))/cfg.MAX_SPEED
                new_car.is_given_route = False
                self.car_list[car_id] = new_car





    def run(self, simu_step):

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


                self.car_list[car_id].Leave_T = simu_step
                self.total_delays += (car.Leave_T - car.Enter_T) - ((cfg.CCZ_LEN+cfg.GZ_LEN+cfg.BZ_LEN+cfg.PZ_LEN+cfg.AZ_LEN)/cfg.MAX_SPEED)

                # Measurement
                #self.total_delays_by_sche += car.D
                self.car_num += 1

                self.car_list.pop(car_id)
                car.zone == "Intersection"


                '''
                Debug for now:
                    Randomly assign the directions
                '''

            elif (not car.is_given_route) and (traci.vehicle.getLanePosition(car_id) > car.length):

                car.turning = random.choice(['R', 'S', 'L'])
                car.is_given_route = True

                intersection_dir = int(lane_id[8])
                x_idx = int(self.ID[0:3])
                y_idx = int(self.ID[4:7])

                target_dir = None

                if car.turning == 'R':
                    target_dir = ((intersection_dir-1)+1)%4+1
                elif car.turning == 'S':
                    target_dir = intersection_dir
                elif car.turning == 'L':
                    target_dir = ((intersection_dir-1)-1)%4+1

                if target_dir == 1:
                    x_idx = x_idx + 1
                    y_idx = y_idx
                elif target_dir == 2:
                    x_idx = x_idx
                    y_idx = y_idx - 1
                elif target_dir == 3:
                    x_idx = x_idx - 1
                    y_idx = y_idx
                elif target_dir == 4:
                    x_idx = x_idx
                    y_idx = y_idx + 1

                intersection_manager_id = "%3.3o"%(x_idx) + "_" + "%3.3o"%(y_idx)

                target_edge = intersection_manager_id + "_" + str(target_dir)
                traci.vehicle.changeTarget(car_id, target_edge)
                traci.vehicle.setColor(car_id, (255,255,255))
