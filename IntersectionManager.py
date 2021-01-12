
import sys
import config as cfg
import traci
import threading
import random


from Cars import Car
from milp import Icacc, IcaccPlus, Fcfs, FixedSignal
from LaneAdviser import LaneAdviser
from get_inter_length_info import Data

inter_length_data = Data()


class IntersectionManager:
    def __init__(self, my_id):
        self.ID = my_id
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

        # Spill-back info
        self.my_road_info = [{'avail_len':cfg.TOTAL_LEN, 'delay':0} for i in range(4*cfg.LANE_NUM_PER_DIRECTION)] # For updating the available road info
        self.others_road_info = [None]*(4*cfg.LANE_NUM_PER_DIRECTION)   # Reference to read others road info
        self.spillback_delay_record = [0]*(4*cfg.LANE_NUM_PER_DIRECTION)

        self.set_round_lane()

    def connect(self, my_direction, intersection, its_direction):
        '''
                2
            3   i   1
                0
        '''
        for lane_idx in range(cfg.LANE_NUM_PER_DIRECTION):
            my_lane = my_direction*cfg.LANE_NUM_PER_DIRECTION+lane_idx
            its_lane = its_direction*cfg.LANE_NUM_PER_DIRECTION+(cfg.LANE_NUM_PER_DIRECTION-lane_idx-1)

            self.others_road_info[my_lane] = intersection.my_road_info[its_lane]
            intersection.others_road_info[its_lane] = self.my_road_info[my_lane]


    def set_round_lane(self):
        for idx in range(1,5):
            for jdx in range(cfg.LANE_NUM_PER_DIRECTION):
                idx_str = self.ID + '_' + str(idx)+'_'+str(jdx)
                self.in_lanes.append(idx_str)

    def check_in_my_region(self, lane_id):
        if lane_id in self.in_lanes:
            return True
        else:
            lane_data = lane_id.split("_")
            lane_id_short = lane_data[0] + "_" + lane_data[1]
            if lane_id_short == self.ID:
                return True
            else:
                return False

    def change_turning(self, car_id, car_turn, intersection_dir):
        id_data = self.ID.split('_')
        x_idx = int(id_data[0])
        y_idx = int(id_data[1])

        target_dir = None

        if car_turn == 'R':
            target_dir = ((intersection_dir-1)+1)%4+1
        elif car_turn == 'S':
            target_dir = intersection_dir
        elif car_turn == 'L':
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

        intersection_manager_id = "00%i"%(x_idx) + "_" + "00%i"%(y_idx)

        target_edge = intersection_manager_id + "_" + str(target_dir)
        traci.vehicle.changeTarget(car_id, target_edge)
        traci.vehicle.setMaxSpeed(car_id, cfg.MAX_SPEED)
        traci.vehicle.setColor(car_id, (255,255,255))


    def update_car(self, car_id, lane_id, simu_step, car_turn):
        if lane_id in self.in_lanes:
            lane_data = lane_id.split("_")
            lane_direction = int(lane_data[2])
            lane_sub_idx = int(lane_data[3])
            lane = ((4-lane_direction))*cfg.LANE_NUM_PER_DIRECTION + (cfg.LANE_NUM_PER_DIRECTION-lane_sub_idx-1)



            # Add car if the car is not in the list yet
            if car_id not in self.car_list:
                # Gather the information of the new car
                #traci.vehicle.setSpeed(car_id, cfg.MAX_SPEED)
                length = traci.vehicle.getLength(car_id)
                turning = car_turn

                new_car = Car(car_id, length, lane, turning)
                new_car.Enter_T = simu_step - (traci.vehicle.getLanePosition(car_id))/cfg.MAX_SPEED
                self.car_list[car_id] = new_car

                traci.vehicle.setSpeed(car_id, cfg.MAX_SPEED)
                #traci.vehicle.setSpeedMode(car_id, 7)


                '''
                Debug for now:
                    Randomly assign the directions
                '''
                #new_car.turning = random.choice(['R', 'S', 'L'])
                #new_car.turning = car_turn

                self.change_turning(car_id, car_turn, lane_direction)

            # Set the position of each cars
            #position = cfg.AZ_LEN + cfg.PZ_LEN + cfg.GZ_LEN+ cfg.BZ_LEN + cfg.CCZ_LEN - traci.vehicle.getLanePosition(car_id)
            lane_id = traci.vehicle.getLaneID(car_id)
            position = traci.lane.getLength(lane_id) - traci.vehicle.getLanePosition(car_id)
            self.car_list[car_id].setPosition(position)


            if (self.car_list[car_id].zone == None) and (position <= cfg.TOTAL_LEN - self.car_list[car_id].length):
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


            # Not yet enter Roadrunner, still able to change turn
            if position > cfg.TOTAL_LEN:
                self.car_list[car_id].turning = car_turn
                self.change_turning(car_id, car_turn, lane_direction)



            # Decide the time offset
            # On the road
            time_offset = None
            intersection_id = None              # After offset, which intersection/node
            intersection_from_direction = None

            if self.car_list[car_id].zone == None:
                # Not yet enter Roadrunner
                diff_pos = position - cfg.TOTAL_LEN
                time_offset = diff_pos/cfg.MAX_SPEED


                # Start from next intersection
                if time_offset <= cfg.ROUTING_PERIOD:
                    time_offset = None
                else:
                    # Decide the intersection id
                    direction = lane//cfg.LANE_NUM_PER_DIRECTION
                    intersection_from_direction = (direction+2)%4
                    intersection_idx_list = self.ID.split("_")
                    if intersection_from_direction == 2:
                        intersection_idx_list[1] = "00%i"%(int(intersection_idx_list[1])-1)
                    elif intersection_from_direction == 3:
                        intersection_idx_list[0] = "00%i"%(int(intersection_idx_list[0])+1)
                    elif intersection_from_direction == 0:
                        intersection_idx_list[1] = "00%i"%(int(intersection_idx_list[1])+1)
                    elif intersection_from_direction == 1:
                        intersection_idx_list[0] = "00%i"%(int(intersection_idx_list[0])-1)

                    intersection_id = intersection_idx_list[0] + "_" + intersection_idx_list[1]

            if time_offset == None:
                # Next intersection
                if not isinstance(self.car_list[car_id].D, float):
                    # Not computed yet
                    time_offset = position/cfg.MAX_SPEED
                else:
                    time_offset = self.car_list[car_id].OT + self.car_list[car_id].D

                # Add the time in the intersection
                lane = None
                if self.car_list[car_id].zone == "AZ":
                    lane = self.car_list[car_id].desired_lane
                else:
                    lane = self.car_list[car_id].lane

                turning = self.car_list[car_id].turning
                time_in_inter = inter_length_data.getIntertime(lane, turning)

                time_offset += time_in_inter  # time pass intersection

                # Start from next intersection
                if time_offset <= cfg.ROUTING_PERIOD:
                    # Not allowing scheduling for a while
                    return None
                else:
                    direction = lane//cfg.LANE_NUM_PER_DIRECTION
                    if turning == "S":
                        intersection_from_direction = (direction+2)%4
                    elif turning == "L":
                        intersection_from_direction = (direction-1)%4
                    elif turning == "R":
                        intersection_from_direction = (direction+1)%4

                    intersection_id = self.ID

            length = self.car_list[car_id].length

            return (length, time_offset, intersection_id, int(intersection_from_direction))

        elif self.ID in lane_id:
            return None


        else:
            return None

    def run(self, simu_step):

        # ===== Update the time OT =====
        for car_key in self.car_list:
            # Update when the car is scheduled
            if self.car_list[car_key].OT != None:
                self.car_list[car_key].OT -= cfg.TIME_STEP

            self.total_fuel_consumption += traci.vehicle.getFuelConsumption(car_key)*cfg.TIME_STEP
            self.fuel_consumption_count += 1


        # ===== Entering the intersection (Record the cars) =====
        to_be_deleted = []
        for car_id, car in self.car_list.items():
            lane_id = traci.vehicle.getLaneID(car_id)
            if lane_id not in self.in_lanes:
                traci.vehicle.setSpeed(car_id, car.speed_in_intersection)

                to_be_deleted.append(car_id)


                self.car_list[car_id].Leave_T = simu_step
                self.total_delays += (car.Leave_T - car.Enter_T) - ((cfg.CCZ_LEN+cfg.GZ_LEN+cfg.BZ_LEN+cfg.PZ_LEN+cfg.AZ_LEN)/cfg.MAX_SPEED)

                # Measurement
                self.total_delays_by_sche += car.D
                self.car_num += 1

                car.zone = "Intersection"

                if car.D+car.OT <= -0.4 or car.D+car.OT >= 0.4:
                    print("DEBUG: Car didn't arrive at the intersection at right time.")

                    print("ID", car.ID)
                    print("OT+D", car.D+car.OT)
                    print("lane", car.lane)
                    print("D", car.D)
                    print("OT", car.OT)
                    print("=======")
                    print("-----------------")
        for car_id in to_be_deleted:
            del self.ccz_list[car_id]
            self.car_list.pop(car_id)



        # ===== Starting Cruise control
        to_be_deleted = []
        for car_id, car in self.pz_list.items():
            if car.position <= cfg.CCZ_LEN and isinstance(car.D, float):

                self.ccz_list[car_id] = car
                to_be_deleted.append(car_id)

                if (car.CC_state == None):
                    car.CC_state = "CruiseControl_ready"

        for car_id in to_be_deleted:
            del self.pz_list[car_id]




        ##############################################
        # Grouping the cars and schedule
        # Put here due to the thread handling
        self.schedule_period_count += cfg.TIME_STEP
        if self.schedule_period_count > int(cfg.GZ_LEN/cfg.MAX_SPEED) -1:
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

                others_road_info = copy.deepcopy(self.others_road_info)

                self.scheduling_thread = threading.Thread(target = Scheduling, args = (self.lane_advisor, sched_car, n_sched_car, advised_n_sched_car, self.cc_list, self.car_list, self.schedule_period_count, others_road_info, self.spillback_delay_record))
                self.scheduling_thread.start()


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
                car.CC_state = "Preseting_ready"


                # Cancel the auto gap
                traci.vehicle.setLaneChangeMode(car_id, 0)

                lane = car.lane
                car.desired_lane = car.lane
                lane_sub_idx = (cfg.LANE_NUM_PER_DIRECTION-lane%cfg.LANE_NUM_PER_DIRECTION-1)

                # Stay on its lane
                traci.vehicle.changeLane(car_id, lane_sub_idx, 1.0)


                car.zone_state = "PZ_set"
        for car_id in to_be_deleted:
            del self.az_list[car_id]

        # Set Max Speed in PZ
        for car_id, car in self.pz_list.items():
            if car.zone == "PZ" and car.zone_state == "PZ_set":
                lane = car.lane
                car.desired_lane = car.lane

                lane_sub_idx = (cfg.LANE_NUM_PER_DIRECTION-lane%cfg.LANE_NUM_PER_DIRECTION-1)

                TODO: set out lane based on the path
                '''
                out_sub_lane = (cfg.LANE_NUM_PER_DIRECTION-lane%cfg.LANE_NUM_PER_DIRECTION-1)

                if car.ID[1] == 'R':
                    out_sub_lane = 0
                elif car.ID[1] == 'L':
                    out_sub_lane = cfg.LANE_NUM_PER_DIRECTION-1

                car.dst_lane = int(car.out_dir*cfg.LANE_NUM_PER_DIRECTION + out_sub_lane)
                '''

                # Stay on its lane
                traci.vehicle.changeLane(car_id, lane_sub_idx, 1.0)

        ##########################################
        # Cruse Control

        # Start to let cars control itself once it enters the CCZ
        # Each car perform their own Cruise Control behavior
        sorted_ccontrol_list = sorted(self.car_list.items(), key=lambda x: x[1].position)
        # SUPER IMPORTANT: sorted to ensure the following car speed
        for car_id, car in sorted_ccontrol_list:
            # Cars perform their own CC
            if car.zone != None:
                car.handle_CC_behavior(self.car_list)



        ################################################
        # Change lane in AZ

        # Check whether there is a spillback
        accumulate_car_len_lane = [0]*(4*cfg.LANE_NUM_PER_DIRECTION)
        spillback_lane_advise_avoid = [False]*(4*cfg.LANE_NUM_PER_DIRECTION)

        for car_id, car in self.car_list.items():
            lane_idx = car.dst_lane
            #print(lane_idx)
            if self.others_road_info[lane_idx] != None:
                accumulate_car_len_lane[lane_idx] += (car.length + cfg.HEADWAY)
            if car.is_spillback == True:
                spillback_lane_advise_avoid[lane_idx] = True

        for lane_idx in range(4*cfg.LANE_NUM_PER_DIRECTION):
            if self.others_road_info[lane_idx] != None:
                if accumulate_car_len_lane[lane_idx] >= self.others_road_info[lane_idx]['avail_len']:
                    spillback_lane_advise_avoid[lane_idx] = True


        for car_id, car in self.car_list.items():
            if car.zone == "AZ" and car.zone_state == "AZ_not_advised":
                self.az_list[car_id] = car

                traci.vehicle.setMinGap(car_id, cfg.HEADWAY)
                traci.vehicle.setLaneChangeMode(car_id, 784)

                time_in_AZ = 9999.91


                #advised_lane = self.lane_advisor.adviseLaneShortestTrajectory(car)
                advised_lane = self.lane_advisor.adviseLane(self.car_list[car_id], spillback_lane_advise_avoid)
                #advised_lane = self.lane_advisor.adviseLane_v2(self.car_list[car_id])
                #advised_lane = random.randrange(0, cfg.LANE_NUM_PER_DIRECTION)

                traci.vehicle.changeLane(car_id, advised_lane, time_in_AZ)
                car.desired_lane = int((cfg.LANE_NUM_PER_DIRECTION-advised_lane-1)+(car.lane//cfg.LANE_NUM_PER_DIRECTION)*cfg.LANE_NUM_PER_DIRECTION)
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

        ##########################################
        # Update the road info after actions
        car_accumulate_len_lane = [0]*(cfg.LANE_NUM_PER_DIRECTION*4)
        delay_lane = [0]*(cfg.LANE_NUM_PER_DIRECTION*4)
        car_position_with_delay_lane = [0]*(cfg.LANE_NUM_PER_DIRECTION*4)
        for car_id, car in self.car_list.items():
            lane = car.lane
            car_accumulate_len_lane[lane] += car.length + cfg.HEADWAY

            if car.position > car_position_with_delay_lane[lane] and isinstance(car.D, float):
                car_position_with_delay_lane[lane] = car.position
                #delay_lane[lane] = (car.OT+car.D)-(car.position/cfg.MAX_SPEED)
                delay_lane[lane] = car.D

        for lane_idx in range(4*cfg.LANE_NUM_PER_DIRECTION):
            self.my_road_info[lane_idx]['avail_len'] = cfg.TOTAL_LEN - car_accumulate_len_lane[lane_idx]
            self.my_road_info[lane_idx]['delay'] = delay_lane[lane_idx]
        #print(self.ID, self.others_road_info)


##########################
# Scheduling thread that handles scheduling and update the table for lane advising
def Scheduling(lane_advisor, sched_car, n_sched_car, advised_n_sched_car, cc_list, car_list, schedule_period_count, others_road_info, spillback_delay_record):

    IcaccPlus(sched_car, n_sched_car, advised_n_sched_car, others_road_info, spillback_delay_record)

    lane_advisor.updateTableFromCars(n_sched_car, advised_n_sched_car)

    for car in n_sched_car:
        car.zone_state = "scheduled"
