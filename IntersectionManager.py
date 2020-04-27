
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
                turning = "R"

                new_car = Car(car_id, length, lane, turning)
                new_car.Enter_T = simu_step - (traci.vehicle.getLanePosition(car_id))/cfg.MAX_SPEED
                self.car_list[car_id] = new_car

                traci.vehicle.setSpeed(car_id, cfg.MAX_SPEED)
                #traci.vehicle.setSpeedMode(car_id, 7)



                '''
                Debug for now:
                    Randomly assign the directions
                '''
                new_car.turning = random.choice(['R', 'S', 'L'])

                intersection_dir = int(lane_id[8])
                x_idx = int(self.ID[0:3])
                y_idx = int(self.ID[4:7])

                target_dir = None

                if new_car.turning == 'R':
                    target_dir = ((intersection_dir-1)+1)%4+1
                elif new_car.turning == 'S':
                    target_dir = intersection_dir
                elif new_car.turning == 'L':
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
                traci.vehicle.setMaxSpeed(car_id, cfg.MAX_SPEED)
                traci.vehicle.setColor(car_id, (255,255,255))





            # Set the position of each cars
            position = cfg.AZ_LEN + cfg.PZ_LEN + cfg.GZ_LEN+ cfg.BZ_LEN + cfg.CCZ_LEN - traci.vehicle.getLanePosition(car_id)
            self.car_list[car_id].setPosition(position)


            if (self.car_list[car_id].zone == None) and (position <= cfg.AZ_LEN + cfg.PZ_LEN + cfg.GZ_LEN + cfg.BZ_LEN + cfg.CCZ_LEN - self.car_list[car_id].length):
                # The minus part is the to prevent cars from changing too early (while in the intersection)
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


    def run(self, cars, index_of_target_car):

        # Cars: list of cars around the intersection
        # index_of_target_car: index of the target car in the list

        # The result is the candidate for turnings
        turning_results = dict()

        # Classify the cars for scheduler
        sched_car = []      # Scheduled
        n_sched_car = []    # Not scheduled

        for car_idx in range(len(cars)):
            car = cars[car_idx]
            if car.AT == None:
                car.OT = None       # Format the data for the scheduler
                car.D = None
                n_sched_car.append(car)

                # Cars given lane advice but not scheduled
                if car_idx != index_of_target_car:
                    advised_n_sched_car.append(car)
            else:
                car.OT = 0          # Format the data for the scheduler
                car.D = car.AT
                sched_car.append(car)


        # Update the table for the lane advising
        lane_advisor.updateTableFromCars(sched_car, advised_n_sched_car)

        for turning_str in ['R', 'S', 'L']:

            # Assign the turning to the car
            car[index_of_target_car].turning = turning_str

            # Line advise
            advised_lane = self.lane_advisor.adviseLane(self.car_list[car_id])

            # Reset the delays
            for c_idx in range(len(n_sched_car)):
                n_sched_car[c_idx].D = None

            # Do the scheduling
            IcaccPlus(sched_car, n_sched_car)


            for car_idx in range(len(cars)):
                if car.AT == None:
                    

            turning_results[turning_str] =
