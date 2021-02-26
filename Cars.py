# Define the car's behaviors (mainly the Cruse Control on each cars)
# Created by Michael. I -C Wang, 16 Sep. 2019

from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import math

sys.path.append('/usr/share/sumo/tools/')
# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


import traci




import config as cfg
import numpy as np

class Car:
    def __init__(self, car_id, length, lane, turn, next_turn):
        # ===== Profile of the car (Not change during the simulation) ==========
        self.ID = car_id
        self.length = length
        self.current_turn = turn
        self.next_turn = next_turn

        self.in_direction = lane // cfg.LANE_NUM_PER_DIRECTION
        self.out_direction = None
        if turn == 'S':
            self.out_direction = (self.in_direction+2)%4
        elif turn == 'R':
            self.out_direction = (self.in_direction+1)%4
        elif turn == 'L':
            self.out_direction = (self.in_direction-1)%4


        # Determine the speed in the intersection
        speed_in_intersection = cfg.TURN_SPEED
        if turn == "S":
            speed_in_intersection = cfg.MAX_SPEED
        else:
            speed_in_intersection = cfg.TURN_SPEED
        self.speed_in_intersection = speed_in_intersection

        # ======================================================================



        # ===== Information that might change during the simulation ============
        self.original_lane = lane   # The lane when the car joined the system
        self.lane = lane            # Current car lane
        self.desired_lane = lane    # The lane that the car wants to change
        self.is_spillback = False
        self.is_spillback_strict = False


        out_sub_lane = (cfg.LANE_NUM_PER_DIRECTION-lane%cfg.LANE_NUM_PER_DIRECTION-1)
        self.dst_lane = int(self.out_direction*cfg.LANE_NUM_PER_DIRECTION + out_sub_lane)     # Destination lane before next lane change
        if next_turn == 'R':
            out_sub_lane = 0
        elif next_turn == 'L':
            out_sub_lane = cfg.LANE_NUM_PER_DIRECTION-1
        elif next_turn == 'S':
            out_sub_lane = cfg.LANE_NUM_PER_DIRECTION//2

        self.dst_lane_changed_to = int(self.out_direction*cfg.LANE_NUM_PER_DIRECTION + out_sub_lane)  # Destination lane after next lane change



        # Position: how far between it and the intersection (0 at the entry of intersection)
        self.position = cfg.AZ_LEN + cfg.PZ_LEN + cfg.GZ_LEN+ cfg.BZ_LEN + cfg.CCZ_LEN
        # ======================================================================



        # ===== Variables for Scheduling =======================================
        self.OT = None
        self.D = None
        self.Enter_T = None
        self.Leave_T = None
        # ======================================================================


        # ===== Zone stage =====================================================
        self.zone = None
        self.zone_state = None
        # ======================================================================



        # ===== Variables for Cruse Control ====================================
        self.CC_front_pos_diff = 0

        self.CC_slow_speed = cfg.MAX_SPEED
        self.CC_shift = None
        self.CC_shift_end = 0

        self.CC_state = None
        self.CC_slowdown_timer = 0
        self.CC_front_car = None
        self.CC_is_stop_n_go = False



        # ======================================================================


    def setPosition(self, pos):
        self.position = pos


    def handle_CC_behavior(self, car_list):
        front_car = None
        front_distance = None
        front_speed = None
        self.CC_slowdown_timer -= cfg.TIME_STEP

        leader_tuple = traci.vehicle.getLeader(self.ID)



        if leader_tuple != None:
            if leader_tuple[0] in car_list.keys():
                front_car_ID = leader_tuple[0]
                front_car = car_list[front_car_ID]
                front_distance = leader_tuple[1] + 3    # Because SUMO measre the distance with a given Gap

                if self.CC_front_pos_diff == 0:
                    self.CC_front_pos_diff = self.position - front_car.position


        self.CC_front_car = front_car
        front_speed = self.CC_get_front_speed()

        # 1. Detect if the front car is too close
        if (self.CC_state == None) or (not ("Platoon" in self.CC_state or "Entering" in self.CC_state)):
            if front_car != None:
                my_speed = traci.vehicle.getSpeed(self.ID) + traci.vehicle.getAcceleration(self.ID)
                min_catch_up_time = (my_speed-front_speed)/cfg.MAX_ACC
                min_distance = (my_speed-front_speed)*min_catch_up_time

                if min_catch_up_time > 0 and front_distance < min_distance + cfg.HEADWAY:
                    self.CC_state = "Platoon_catchup"

        # 2. If the car is ready for stopping
        if (self.position < (2*cfg.CCZ_ACC_LEN+cfg.CCZ_DEC2_LEN)) and ((self.CC_state == None) or (not ("Entering" in self.CC_state))):
            self.CC_state = "Entering_decelerate"
            slow_down_speed = 0

            my_speed = traci.vehicle.getSpeed(self.ID)

            if not isinstance(self.D, float):
                slow_down_speed = 0.001

            else:
                # Compute the slowdown speed
                T = self.OT+self.D- ((cfg.CCZ_DEC2_LEN) / ((self.speed_in_intersection+cfg.MAX_SPEED)/2))
                max_total_time = (self.position - (cfg.CCZ_ACC_LEN+cfg.CCZ_DEC2_LEN))/(my_speed/2) + cfg.CCZ_ACC_LEN/(cfg.MAX_SPEED/2)

                if T > max_total_time:
                    self.CC_auto_stop_n_go = True
                    slow_down_speed = 0.001
                elif T < 0:
                    slow_down_speed = cfg.MAX_SPEED
                else:
                    x1 = self.position - (cfg.CCZ_ACC_LEN+cfg.CCZ_DEC2_LEN)
                    x2 = cfg.CCZ_ACC_LEN
                    v1 = my_speed
                    vm = cfg.MAX_SPEED

                    a = T
                    b = (vm*T+v1*T-2*x1-2*x2)
                    c = (vm*v1*T-2*x1*vm-2*x2*v1)

                    slow_down_speed = max( (-b-math.sqrt(b**2-4*a*c))/(2*a), (-b+math.sqrt(b**2-4*a*c))/(2*a))
                    slow_down_speed = min(slow_down_speed, cfg.MAX_SPEED)

            self.CC_slow_speed = slow_down_speed
            if slow_down_speed < 0:
                print(self.ID, T, max_total_time, slow_down_speed)
                print(self.ID, x1, x2, v1, vm)

            traci.vehicle.setMaxSpeed(self.ID, slow_down_speed)
            dec_time = (self.position-(cfg.CCZ_ACC_LEN+cfg.CCZ_DEC2_LEN)) / ((my_speed+slow_down_speed)/2)
            self.CC_slowdown_timer = dec_time
            if (dec_time < 0):
                print(self.ID, slow_down_speed, dec_time, self.position, my_speed+slow_down_speed)
            traci.vehicle.slowDown(self.ID,slow_down_speed, dec_time)



        elif (self.CC_state == "Entering_decelerate" or self.CC_state == "Entering_wait") and (self.CC_slowdown_timer <= 0):
            traci.vehicle.setSpeed(self.ID, self.CC_slow_speed)

            wait_time = 99999   # inf and wait
            if isinstance(self.D, float):
                wait_time = self.OT+self.D - ((self.position - cfg.CCZ_DEC2_LEN) / ((cfg.MAX_SPEED+self.CC_slow_speed)/2)) - ((cfg.CCZ_DEC2_LEN) / ((self.speed_in_intersection+cfg.MAX_SPEED)/2))

            if wait_time > 0:
                self.CC_state = "Entering_wait"
            else:
                self.CC_state = "Entering_accerlerate"

                traci.vehicle.setMaxSpeed(self.ID, cfg.MAX_SPEED)
                dec_time = None
                if self.position>cfg.CCZ_DEC2_LEN:
                    dec_time = (self.position-cfg.CCZ_DEC2_LEN) / ((self.CC_slow_speed+cfg.MAX_SPEED)/2)
                else:
                    dec_time = self.position / ((self.CC_slow_speed+cfg.MAX_SPEED)/2)
                self.CC_slowdown_timer = dec_time
                traci.vehicle.slowDown(self.ID, cfg.MAX_SPEED, dec_time)

        elif (self.CC_state == "Entering_accerlerate") and (self.CC_slowdown_timer <= 0):
            self.CC_state = "Entering_adjusting_speed"

            traci.vehicle.setMaxSpeed(self.ID, self.speed_in_intersection)
            dec_time = self.position / ((self.speed_in_intersection+cfg.MAX_SPEED)/2)
            self.CC_slowdown_timer = dec_time
            traci.vehicle.slowDown(self.ID, self.speed_in_intersection, dec_time)

        elif (self.CC_state == "Entering_adjusting_speed") and (self.CC_slowdown_timer <= 0):
            self.CC_state = "Entering_intersection"
            traci.vehicle.setSpeed(self.ID, self.speed_in_intersection)


        elif (self.CC_state == "Platoon_catchup"):
            if front_car == None:
                my_speed = traci.vehicle.getSpeed(self.ID)
                target_speed = min(cfg.MAX_SPEED, my_speed + cfg.MAX_ACC*cfg.TIME_STEP)
                traci.vehicle.setSpeed(self.ID, target_speed)
                if target_speed == cfg.MAX_SPEED:
                    self.CC_state = "Keep_Max_speed"
            else:
                my_speed = traci.vehicle.getSpeed(self.ID)
                min_catch_up_time = (my_speed-0)/cfg.MAX_ACC
                min_distance = (my_speed-0)*min_catch_up_time

                if front_distance < min_distance:
                    target_speed = max(front_speed, my_speed - cfg.MAX_ACC*cfg.TIME_STEP)

                    if front_distance <= cfg.HEADWAY:
                        self.CC_state = "Platoon_following"
                        traci.vehicle.setSpeed(self.ID, front_speed)
                    else:
                        if front_speed > target_speed:
                            target_speed = min(cfg.MAX_SPEED, my_speed + cfg.MAX_ACC*cfg.TIME_STEP)
                            traci.vehicle.setSpeed(self.ID, target_speed)
                        else:
                            target_speed = max(front_speed + cfg.MAX_ACC*cfg.TIME_STEP, my_speed - cfg.MAX_ACC*cfg.TIME_STEP)
                            traci.vehicle.setSpeed(self.ID, target_speed)
                else:
                    target_speed = min(cfg.MAX_SPEED, my_speed + cfg.MAX_ACC*cfg.TIME_STEP)
                    traci.vehicle.setSpeed(self.ID, target_speed)

        elif (self.CC_state == "Platoon_following"):

            if front_car == None:
                my_speed = traci.vehicle.getSpeed(self.ID)
                target_speed = min(cfg.MAX_SPEED, my_speed + cfg.MAX_ACC*cfg.TIME_STEP)
                traci.vehicle.setSpeed(self.ID, target_speed)
                if target_speed == cfg.MAX_SPEED:
                    self.CC_state = "Keep_Max_speed"
            else:
                if front_distance > cfg.HEADWAY:
                    my_speed = traci.vehicle.getSpeed(self.ID)
                    target_speed = min(cfg.MAX_SPEED, my_speed + cfg.MAX_ACC*cfg.TIME_STEP)
                    traci.vehicle.setSpeed(self.ID, target_speed)
                    self.CC_state = "Platoon_catchup"
                else:
                    traci.vehicle.setSpeed(self.ID, front_speed)

        elif (self.CC_state == "Preseting_start"):
            my_speed = traci.vehicle.getSpeed(self.ID)
            traci.vehicle.setMaxSpeed(self.ID, cfg.MAX_SPEED)
            dec_time = (max(cfg.MAX_SPEED-my_speed, my_speed-cfg.MAX_SPEED))/cfg.MAX_ACC
            self.CC_slowdown_timer = dec_time
            traci.vehicle.slowDown(self.ID,cfg.MAX_SPEED, dec_time)
            self.CC_state = "Preseting_done"


        elif (self.CC_state == "Preseting_done") and (self.CC_slowdown_timer <= 0):
             traci.vehicle.setSpeed(self.ID, cfg.MAX_SPEED)

        elif (self.CC_state == "CruiseControl_ready"):
            if self.CC_front_car != None and self.CC_front_car.CC_shift == None:
                self.CC_front_car = None

            reply = self.CC_get_shifts(car_list)
            self.CC_get_slow_down_speed()
            if self.CC_is_stop_n_go == True:
                # Only stop at very closed to the intersection
                self.CC_state = "Keep_Max_speed"
            else:
                self.CC_state = "CruiseControl_shift_start"

        elif (self.CC_state == "CruiseControl_shift_start") and self.position < (cfg.CCZ_LEN-self.CC_shift):
            self.CC_state = "CruiseControl_decelerate"
            speed = self.CC_slow_speed

            # Delta: some small error that SUMO unsync with ideal case
            delta = (cfg.CCZ_LEN-self.CC_shift)-self.position
            dec_time = (cfg.CCZ_ACC_LEN-delta) / ((cfg.MAX_SPEED+speed)/2)

            if dec_time < 0:
                self.CC_state = "Keep_Max_speed"
            else:
                traci.vehicle.setMaxSpeed(self.ID, speed)
                traci.vehicle.slowDown(self.ID, speed, dec_time)
                self.CC_slowdown_timer = dec_time

        elif (self.CC_state == "CruiseControl_decelerate") and (self.CC_slowdown_timer <= 0):
            traci.vehicle.setSpeed(self.ID, self.CC_slow_speed)
            self.CC_state = "CruiseControl_slowdown_speed"

        elif (self.CC_state == "CruiseControl_slowdown_speed") and (self.position <= (cfg.CCZ_ACC_LEN + self.CC_shift_end)):
            self.CC_state = "CruiseControl_accelerate"
            # Delta: some small error that SUMO unsync with ideal case
            delta = (cfg.CCZ_ACC_LEN + self.CC_shift_end)-self.position
            dec_time = (cfg.CCZ_ACC_LEN-delta) / ((cfg.MAX_SPEED+self.CC_slow_speed)/2)
            traci.vehicle.setMaxSpeed(self.ID, cfg.MAX_SPEED)
            traci.vehicle.slowDown(self.ID, cfg.MAX_SPEED, dec_time)
            self.CC_slowdown_timer = dec_time

        elif (self.CC_state == "CruiseControl_accelerate") and (self.CC_slowdown_timer <= 0):
            self.CC_state == "CruiseControl_max_speed"
            traci.vehicle.setSpeed(self.ID, cfg.MAX_SPEED)




    def CC_get_front_speed(self):
        if self.CC_front_car != None:
            if self.CC_front_car.CC_state != None and "Platoon" in self.CC_front_car.CC_state:
                return self.CC_front_car.CC_get_front_speed()
            else:
                return traci.vehicle.getSpeed(self.CC_front_car.ID) + traci.vehicle.getAcceleration(self.CC_front_car.ID)*cfg.TIME_STEP
        else:
            return traci.vehicle.getSpeed(self.ID) + traci.vehicle.getAcceleration(self.ID)*cfg.TIME_STEP


    # Compute the shifts
    def CC_get_shifts(self, car_list):
        # 1.1 Determine how much to advance the car acceleration (shift_end)

        is_catching_up_front = False

        if self.CC_front_car != None:
            shifting_end = cfg.CCZ_DEC2_LEN
            front_remain_D = (self.CC_front_car.OT+self.CC_front_car.D)-(self.CC_front_car.position/cfg.MAX_SPEED)
            catch_up_distance = (front_remain_D - self.D)*cfg.MAX_SPEED
            diff_distance = self.position - self.CC_front_car.position
            if (diff_distance - catch_up_distance - self.CC_front_car.length) < (cfg.HEADWAY):
                # The car is going to catch up the front car
                shifting_end = self.CC_front_car.CC_shift_end + self.CC_front_car.length + cfg.HEADWAY
                is_catching_up_front = True
            self.CC_shift_end = shifting_end


        # 1.2 Determine the upperbound of the delaying for a car to accelerate
        cc_shift_max = cfg.CCZ_LEN-self.CC_shift_end-2*cfg.CCZ_ACC_LEN
        if is_catching_up_front and self.CC_front_car.CC_slow_speed < cfg.MAX_SPEED:
            # First, assume that the two cars decelerate at the same time (So the car has this shift)
            cc_shift_max = self.CC_front_car.CC_shift - self.CC_front_pos_diff

            # The space between two cars (might < 0, because might smaller than HEADWAY)
            space_between_two = max(self.CC_front_pos_diff - self.CC_front_car.length - cfg.HEADWAY, 0)

            #2 Compute catch up time and reflect to the space
            catch_up_t = space_between_two/(cfg.MAX_SPEED-self.CC_front_car.CC_slow_speed)
            cc_shift_max += catch_up_t*cfg.MAX_SPEED

        cc_shift_max = min(cc_shift_max, cfg.CCZ_LEN-self.CC_shift_end-2*cfg.CCZ_ACC_LEN)


        # 1.3 Determine the delay it desires. Reserving for the following cars
        # Count cars that'll enter CCZ during the delaying
        reserve_shift = self.length + cfg.HEADWAY
        count_distance = self.D * cfg.MAX_SPEED + self.length + cfg.HEADWAY

        count_car_list = []
        for count_car_id, count_car in car_list.items():
            if (count_car.lane == self.lane) and (count_car.position > self.position):
                count_car_list.append(count_car)

        count_car_list.append(self)
        count_car_list = sorted(count_car_list, key = lambda x: x.position)

        temp_D = self.D

        for car_i in range(1, len(count_car_list)):
            if (count_car_list[car_i].position < count_car_list[car_i-1].position+count_distance):
                reserve_shift += count_car_list[car_i].length+cfg.HEADWAY

                # If the car has been scheduled
                if isinstance(count_car_list[car_i].D, float):
                    count_distance = count_car_list[car_i].D * cfg.MAX_SPEED + count_car_list[car_i].length + cfg.HEADWAY
                    temp_D = count_car_list[car_i].D
                else:
                    count_distance = temp_D * cfg.MAX_SPEED + count_car_list[car_i].length + cfg.HEADWAY

            else:
                break

        reserve_shift = min(reserve_shift, cfg.CCZ_LEN-self.CC_shift_end-2*cfg.CCZ_ACC_LEN)

        # 1.4 Decide the final shift
        shifting = min(reserve_shift, cc_shift_max)

        self.CC_shift = shifting

        return {'shifting': shifting, 'shifting_end': self.CC_shift_end}




    # Compute the slow-slow speed
    def CC_get_slow_down_speed(self):
        D = self.D
        AT = D + ((cfg.CCZ_LEN)/cfg.MAX_SPEED) - (self.CC_shift/cfg.MAX_SPEED) - ((self.CC_shift_end-cfg.CCZ_DEC2_LEN)/cfg.MAX_SPEED) - (2*cfg.CCZ_DEC2_LEN/(cfg.MAX_SPEED+self.speed_in_intersection))

        # ----\_________/----
        #     S   S2    S
        #

        S = cfg.CCZ_ACC_LEN
        S2 = cfg.CCZ_LEN - 2*cfg.CCZ_ACC_LEN - self.CC_shift - self.CC_shift_end
        T = ((cfg.CCZ_LEN)/cfg.MAX_SPEED)+D - ((self.CC_shift+self.CC_shift_end-cfg.CCZ_DEC2_LEN)/cfg.MAX_SPEED +2*cfg.CCZ_DEC2_LEN/(cfg.MAX_SPEED+self.speed_in_intersection))

        poly_coeff = [0]*3
        poly_coeff[0] = T
        poly_coeff[1] = -(S2 + 4*S - T*cfg.MAX_SPEED)
        poly_coeff[2] = -S2 * cfg.MAX_SPEED

        ans = np.roots(poly_coeff)
        speed = min(max(ans), cfg.MAX_SPEED)


        # Determine if there's stop and go
        if speed < 1:
            self.CC_is_stop_n_go = True


        self.CC_slow_speed = speed
