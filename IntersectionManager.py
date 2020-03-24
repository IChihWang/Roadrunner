
import sys
import config as cfg
import traci
import threading


from Cars import Car
from milp import Icacc, IcaccPlus, Fcfs, FixedSignal
from LaneAdviser import LaneAdviser


class IntersectionManager:
    def __init__(self):
        self.az_list = dict()
        self.pz_list = dict()

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


    def run(self, simu_step):

        # ===== Update the time OT =====
        for car_key in self.car_list:
            # Update when the car is scheduled
            if self.car_list[car_key].OT != None:
                self.car_list[car_key].OT -= cfg.TIME_STEP


        # ===== Entering the intersection (Record the cars) =====
        for car_id, car in self.car_list.items():
            lane_id = traci.vehicle.getLaneID(car_id)
            if lane_id not in self.in_lanes:
                traci.vehicle.setSpeed(car_id, car.speed_in_intersection)

                self.pz_list.pop(car_id)

                self.leaving_cars[car_id] = self.car_list[car_id]
                self.car_list[car_id].Leave_T = simu_step
                self.total_delays += (car.Leave_T - car.Enter_T) - ((cfg.CCZ_LEN+cfg.GZ_LEN+cfg.BZ_LEN+cfg.PZ_LEN+cfg.AZ_LEN)/cfg.MAX_SPEED)

                # Measurement
                self.total_delays_by_sche += car.D
                self.car_num += 1

                self.car_list.pop(car_id)
                car.zone == "Intersection"

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
        for car_id, car in self.leaving_cars.items():
            lane_id = traci.vehicle.getLaneID(car_id)
            if lane_id in self.out_lanes:
                traci.vehicle.setSpeed(car_id, cfg.MAX_SPEED)
                del self.leaving_cars[car_id]




        ##############################################
        # Grouping the cars and schedule
        # Put here due to the thread handling
        self.schedule_period_count += 1
        if self.schedule_period_count > (1/cfg.TIME_STEP)*cfg.GZ_LEN/cfg.MAX_SPEED-1:
            if self.scheduling_thread == None or (not self.scheduling_thread.isAlive()):

                # Classify the cars for scheduler
                sched_car = []
                n_sched_car = []
                advised_n_sched_car = []
                for car_id, car in self.car_list.items():
                    if car.zone == "GZ" or car.zone == "BZ" or car.zone == "CCZ":
                        if car.zone_state == "not_scheduled":
                            n_sched_car.append(car)
                        else:
                            sched_car.append(car)
                    elif car.zone == "PZ" or car.zone == "AZ":
                        advised_n_sched_car.append(car)


                for c_idx in range(len(n_sched_car)):
                    traci.vehicle.setColor(n_sched_car[c_idx].ID, (100,250,92))
                    n_sched_car[c_idx].D = None

                self.scheduling_thread = threading.Thread(target = Scheduling, args = (self.lane_advisor, sched_car, n_sched_car, advised_n_sched_car, self.cc_list, self.car_list))
                self.scheduling_thread.start()


                self.schedule_period_count = 0

            else:
                print("Warning: the update period does not sync with the length of GZ")






        ################################################
        # Set Max Speed in PZ
        for car_id, car in self.az_list.items():
            if car.zone == "PZ" and car.zone_state == "PZ_not_set":

                self.pz_list[car_id] = car
                del self.az_list[car_id]

                # Take over the speed control from the car
                traci.vehicle.setSpeedMode(car_id, 0)

                '''
                traci.vehicle.setSpeed(car_id, cfg.MAX_SPEED)
                '''

                car.CC_state = "Preseting_ready"
                '''
                traci.vehicle.setMaxSpeed(self.ID, cfg.MAX_SPEED)
                dec_time = (max(cfg.MAX_SPEED-my_speed, my_speed-cfg.MAX_SPEED))/cfg.MAX_ACC
                self.CC_slowdown_timer = dec_time
                traci.vehicle.slowDown(self.ID,cfg.MAX_SPEED, dec_time)
                '''



                # Cancel the auto gap
                traci.vehicle.setLaneChangeMode(car_id, 0)

                lane_id = traci.vehicle.getLaneID(car_id)
                lane = ((4-int(lane_id[0]))*cfg.LANE_NUM_PER_DIRECTION) + (cfg.LANE_NUM_PER_DIRECTION-int(lane_id[2])-1)
                self.car_list[car_id].lane = lane

                # Stay on its lane
                traci.vehicle.changeLane(car_id, int(lane_id[2]), 10.0)


                car.zone_state = "PZ_set"


        ##########################################
        # Cruse Control

        # Start to let cars control itself once it enters the CCZ
        # Each car perform their own Cruise Control behavior
        sorted_pz_list = sorted(self.pz_list.items(), key=lambda x: x[1].position)
        # SUPER IMPORTANT: sorted to ensure the following car speed
        for car_id, car in sorted_pz_list:
            # Cars perform their own CC
            car.handle_CC_behavior_general(self.car_list)


        #======================================
        # Handle halting
        '''
        cars_need_handle = dict()
        available_space = dict()
        is_congested = dict()
        for idx in range(4*cfg.LANE_NUM_PER_DIRECTION):
            cars_need_handle[idx] = []
            available_space[idx] = cfg.CCZ_LEN
            is_congested[idx] = False

        for car_id, car in self.car_list.items():
            if car.CC_shift != None:
                if available_space[car.lane] > max(car.CC_shift, cfg.CCZ_LEN-car.position):
                    available_space[car.lane] = max(car.CC_shift, cfg.CCZ_LEN-car.position)
                if car.CC_shift < 0:
                    is_congested[car.lane] = True
            elif car.position > cfg.CCZ_LEN+cfg.BZ_LEN+cfg.GZ_LEN+cfg.PZ_LEN:
                cars_need_handle[car.lane].append(car)


        for car_id, car in self.car_list.items():
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
        '''


        ################################################
        # Change lane in AZ
        for car_id, car in self.car_list.items():
            if car.zone == "AZ" and car.zone_state == "AZ_not_advised":
                self.az_list[car_id] = car

                traci.vehicle.setMinGap(car_id, 1)
                traci.vehicle.setLaneChangeMode(car_id, 256)
                # 256 (collision avoidance) or 512 (collision avoidance and safety-gap enforcement)

                time_in_AZ = cfg.AZ_LEN/cfg.MAX_SPEED *3


                advised_lane = self.lane_advisor.adviseLaneShortestTrajectory(car)
                #advised_lane = self.lane_advisor.adviseLane(self.car_list[car_id])
                #advised_lane = self.lane_advisor.adviseLane_v2(self.car_list[car_id])
                #advised_lane = random.randrange(0, cfg.LANE_NUM_PER_DIRECTION)

                traci.vehicle.changeLane(car_id, advised_lane, time_in_AZ)
                car.desired_lane = (cfg.LANE_NUM_PER_DIRECTION-advised_lane-1)+(car.lane//cfg.LANE_NUM_PER_DIRECTION)*cfg.LANE_NUM_PER_DIRECTION
                #self.car_list[car_id].desired_lane = car.lane

                car.zone_state = "AZ_advised"


        ###########################################################
        # Handle halting in AZ
        # Must be after PZ_len, because its function overwite some commands
        for id, car in self.az_list.items():
            car.handleHalting()


            lane_id = traci.vehicle.getLaneID(id)
            lane = ((4-int(lane_id[0]))*cfg.LANE_NUM_PER_DIRECTION) + (cfg.LANE_NUM_PER_DIRECTION-int(lane_id[2])-1)
            self.car_list[id].lane = lane


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

    for car in n_sched_car:
        car.zone_state = "scheduled"

    # Handle the speed control computation
    n_cars_on_lanes = dict()
    for idx in range(4*cfg.LANE_NUM_PER_DIRECTION):
        n_cars_on_lanes[idx] = []

    # Handle car behavior and detect congestion
    for car in n_sched_car:
        # Getting ready for CC
        car.CC_stage = '0 Not Handled Yet'
        n_cars_on_lanes[car.lane].append(car)



    for idx in range(4*cfg.LANE_NUM_PER_DIRECTION):
        sorted_car_list = sorted(n_cars_on_lanes[idx], key=lambda car: car.position)

        for car in sorted_car_list:
            car.handle_CC_behavior(car_list)
            car.CC_done_scheduling = True
