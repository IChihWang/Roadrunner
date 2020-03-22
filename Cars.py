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
    def __init__(self, car_id, length, lane, turning):
        # ===== Profile of the car (Not change during the simulation) ==========
        self.ID = car_id
        self.length = length
        self.turning = turning

        # Determine the speed in the intersection
        speed_in_int = cfg.TURN_SPEED
        if turning == "S":
            speed_in_int = cfg.MAX_SPEED
        else:
            speed_in_int = cfg.TURN_SPEED
        self.speed_in_int = speed_in_int

        # ======================================================================



        # ===== Information that might change during the simulation ============
        self.original_lane = lane   # The lane when the car joined the system
        self.lane = lane
        self.desired_lane = lane

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
        self.CC_stage = None
        self.CC_slow_speed = cfg.MAX_SPEED
        self.CC_shift = None
        self.CC_shift_end = 0
        self.CC_affected_by_front = False
        self.CC_stop_n_go = 0            # A car needs to stop and go
        self.CC_stop_n_go_remained = 0   # Count down the stop-and-go timer
        self.CC_front_car = None
        self.CC_back_car = None


        self.CC_front_pos_diff = None    # Original distance between self and the front car
        self.CC_following = None
        self.CC_done_scheduling = False


        self.CC_auto_stop_n_go = False
        self.CC_auto_slow_down_speed = None
        self.CC_auto = False


        self.CC_is_following = False

        # ======================================================================


        # ===== Variables for Halting Control ==================================
        self.H_should_halt = False              # For receiving command
        self.H_is_halting = False               # For resuming the speed
        self.H_gave_halting_command = False     # Making sure it only give command once



    def setHalting(self, should_halt):
        self.H_should_halt = should_halt
    def handleHalting(self):
        if self.H_should_halt == True and self.H_gave_halting_command == False:
            if self.position < cfg.CCZ_LEN+cfg.BZ_LEN+cfg.GZ_LEN+cfg.PZ_LEN+2*cfg.CCZ_ACC_LEN:
                dec_time = cfg.CCZ_ACC_LEN/((cfg.MAX_SPEED+0)/2)
                traci.vehicle.setMaxSpeed(self.ID, 0.0001)
                traci.vehicle.slowDown(self.ID, 0, dec_time)
                self.H_is_halting = True
                self.H_gave_halting_command = True

        elif self.H_is_halting == True and self.H_should_halt == False:
            # Resume the speed
            dec_time = (self.position-(cfg.CCZ_LEN+cfg.BZ_LEN+cfg.GZ_LEN+cfg.PZ_LEN))/((cfg.MAX_SPEED+0)/2)
            traci.vehicle.setMaxSpeed(self.ID, cfg.MAX_SPEED)
            traci.vehicle.slowDown(self.ID, cfg.MAX_SPEED, dec_time)
            self.H_is_halting == False
            self.H_gave_halting_command = False



    def setPosition(self, pos):
        self.position = pos



    def handle_CC_behavior_general(self):

        if "S_9" in self.car_list:
             print(traci.vehicle.getLeader("S_9"))


        # 1. Detect potential crashing
        if self.CC_front_car != None:
        else:
            self.CC_is_following

        # 1. Decelerate for stopping
        if self.position < (2*cfg.CCZ_ACC_LEN+cfg.CCZ_DEC2_LEN):

            None


    def handle_CC_behavior(self, car_list):
        # Handle the CC behavior with a finite state machine
        assert(self.CC_stage != None)

        if self.CC_stage == '0 Not Handled Yet':

            reply = self.CC_get_shifts(car_list)
            self.CC_get_slow_down_speed()


            if reply['CC_is_auto_speed_control'] == False:
                self.CC_stage = '1 Computed'
            else:
                self.CC_stage = '(auto)1 Computed'
                self.CC_auto = True

        elif self.CC_stage == '1 Computed':
            # Trigger when it enter the deceleration stage
            if self.position < (cfg.CCZ_LEN-self.CC_shift):
                self.CC_stage = '2 decelerate'
                speed = self.CC_slow_speed

                dec_time = None
                if self.position < cfg.CCZ_LEN:
                    # Delta: some small error that SUMO unsync with ideal case
                    delta = (cfg.CCZ_LEN-self.CC_shift)-self.position
                    dec_time = (cfg.CCZ_ACC_LEN-delta) / ((cfg.MAX_SPEED+speed)/2)
                else:
                    dec_time = (cfg.CCZ_ACC_LEN) / ((cfg.MAX_SPEED+speed)/2)

                # Workaround of TraCI. slowDown() will change back to constant max speed after finished, so these are to prevent that.
                if speed == 0:
                    traci.vehicle.setMaxSpeed(self.ID, 0.001)
                else:
                    traci.vehicle.setMaxSpeed(self.ID, speed)

                #print(self.ID, dec_time, delta, cfg.CCZ_LEN, self.CC_shift, self.position)
                traci.vehicle.slowDown(self.ID, speed, dec_time)

        elif self.CC_stage == '2 decelerate':
            if self.position < (cfg.CCZ_LEN-cfg.CCZ_ACC_LEN - self.CC_shift):
                traci.vehicle.setSpeed(self.ID, self.CC_slow_speed)

                # Determine if a car needs to stop and go
                if self.CC_stop_n_go_remained == 0:
                    self.CC_stage = '3 const'
                else:
                    self.CC_stage = '2.5 stop and wait'
                    traci.vehicle.setSpeed(self.ID, 0.001)
                    self.CC_stop_n_go_remained -= cfg.TIME_STEP

        elif self.CC_stage == '2.5 stop and wait':
            self.CC_stop_n_go_remained -= cfg.TIME_STEP
            if self.CC_stop_n_go_remained < cfg.TIME_STEP:
                self.CC_stage = '3 const'

        elif self.CC_stage == '3 const':
            if self.position < (cfg.CCZ_ACC_LEN + self.CC_shift_end):
                self.CC_stage = '4 accelerate'
                # Delta: some small error that SUMO unsync with ideal case
                delta = (cfg.CCZ_ACC_LEN + self.CC_shift_end)-self.position
                dec_time = (cfg.CCZ_ACC_LEN-delta) / ((cfg.MAX_SPEED+self.CC_slow_speed)/2)
                traci.vehicle.setMaxSpeed(self.ID, cfg.MAX_SPEED)
                traci.vehicle.slowDown(self.ID, cfg.MAX_SPEED, dec_time)

        elif self.CC_stage == '4 accelerate':
            if self.position < self.CC_shift_end:
                self.CC_stage = '5 const_max_speed'
                traci.vehicle.setSpeed(self.ID, cfg.MAX_SPEED)

        elif self.CC_stage == '5 const_max_speed':

            if self.position < (cfg.CCZ_DEC2_LEN):
                self.CC_stage = '6 dec2'
                # Delta: some small error that SUMO unsync with ideal case
                delta = (cfg.CCZ_DEC2_LEN)-self.position
                dec_time = (cfg.CCZ_DEC2_LEN-delta) / ((self.speed_in_int+cfg.MAX_SPEED)/2)
                traci.vehicle.setMaxSpeed(self.ID, cfg.MAX_SPEED)
                traci.vehicle.slowDown(self.ID, self.speed_in_int, dec_time)

        elif self.CC_stage == '6 dec2':
            if self.position <= 0:
                self.CC_stage = '7 enter_inter'
                traci.vehicle.setSpeed(self.ID, self.speed_in_int)


        elif self.CC_stage == '(auto)1 Computed':
            speed = traci.vehicle.getSpeed(self.ID)

            if self.position < self.CC_front_car.position+self.CC_front_car.length+cfg.HEADWAY+cfg.CCZ_ACC_LEN:
                front_speed = traci.vehicle.getSpeed(self.CC_front_car.ID)+traci.vehicle.getAcceleration(self.CC_front_car.ID)*cfg.TIME_STEP
                traci.vehicle.setSpeedMode(self.ID, 0)
                if front_speed < cfg.CCZ_CATCHUP_MIN_SPEED:
                    dec_time = (cfg.HEADWAY) / (((speed-cfg.CCZ_CATCHUP_MIN_SPEED)+0)/2)
                    traci.vehicle.slowDown(self.ID, (cfg.CCZ_CATCHUP_MIN_SPEED), dec_time)
                elif not front_speed == speed:
                    dec_time = (self.position -( self.CC_front_car.position+self.CC_front_car.length)) / (((max(speed-front_speed, front_speed-speed))))
                    traci.vehicle.slowDown(self.ID, (front_speed), dec_time)
                self.CC_stage = '(auto)1.5 Catchup'

            if self.position < (2*cfg.CCZ_ACC_LEN+cfg.CCZ_DEC2_LEN) + speed*cfg.TIME_STEP:
                self.CC_stage = '(auto)1.5 following'

        elif self.CC_stage == '(auto)1.5 Catchup':
            front_speed = traci.vehicle.getSpeed(self.CC_front_car.ID)+traci.vehicle.getAcceleration(self.CC_front_car.ID)*cfg.TIME_STEP
            speed = traci.vehicle.getSpeed(self.ID)

            if self.position < self.CC_front_car.position+self.CC_front_car.length+cfg.HEADWAY+front_speed*cfg.TIME_STEP:
                traci.vehicle.setSpeed(self.ID, front_speed)
                self.CC_stage = '(auto)1.5 following'
            else:
                if front_speed < cfg.CCZ_CATCHUP_MIN_SPEED:
                    dec_time = (self.position -( self.CC_front_car.position+self.CC_front_car.length)) / (((max(speed-cfg.CCZ_CATCHUP_MIN_SPEED, cfg.CCZ_CATCHUP_MIN_SPEED-speed))+0))
                    traci.vehicle.slowDown(self.ID, (cfg.CCZ_CATCHUP_MIN_SPEED), dec_time)

                elif not front_speed == speed:
                    dec_time = (self.position -( self.CC_front_car.position+self.CC_front_car.length)) / (((max(speed-front_speed, front_speed-speed))))
                    traci.vehicle.slowDown(self.ID, (front_speed), dec_time)
            if self.CC_front_car.CC_stage == '4 accelerate':
                traci.vehicle.setSpeed(self.ID, front_speed)
                delta = (cfg.CCZ_ACC_LEN + self.CC_front_car.CC_shift_end)-self.CC_front_car.position
                dec_time = (cfg.CCZ_ACC_LEN-delta) / ((cfg.MAX_SPEED+self.CC_front_car.CC_slow_speed)/2)
                traci.vehicle.slowDown(self.ID, cfg.MAX_SPEED, dec_time)
                self.CC_stage = '(auto)1.5 following'

            if self.position < ((2*cfg.CCZ_ACC_LEN+cfg.CCZ_DEC2_LEN)+speed*cfg.TIME_STEP):
                self.CC_stage = '(auto)1.5 following'


        elif self.CC_stage == '(auto)1.5 following':

            front_speed = self.CC_get_front_speed()
            speed = traci.vehicle.getSpeed(self.ID)

            traci.vehicle.setSpeed(self.ID, front_speed)


            if cfg.CCZ_LEN - self.position - cfg.HEADWAY > self.CC_shift:
                self.CC_shift = cfg.CCZ_LEN - self.position - cfg.HEADWAY

            if self.position < cfg.CCZ_LEN-self.CC_shift:
                self.CC_shift = cfg.CCZ_LEN - self.position

            if self.position < (2*cfg.CCZ_ACC_LEN+cfg.CCZ_DEC2_LEN):
                # Ready for stop and wait
                # Compute the slowdown speed
                speed = traci.vehicle.getSpeed(self.ID)
                T = self.OT+self.D- ((cfg.CCZ_DEC2_LEN) / ((self.speed_in_int+0)/2))
                k = T/ (2*cfg.CCZ_ACC_LEN)
                a = k
                b = k*(speed+cfg.MAX_SPEED)-2
                c = k*speed*cfg.MAX_SPEED-(speed+cfg.MAX_SPEED)
                slow_down_speed = max( (-b-math.sqrt(b**2-4*a*c))/(2*a), (-b+math.sqrt(b**2-4*a*c))/(2*a))
                slow_down_speed = min(slow_down_speed, cfg.MAX_SPEED)

                time_bound = cfg.CCZ_ACC_LEN/(speed/2) + cfg.CCZ_ACC_LEN/(cfg.MAX_SPEED/2)
                if T > time_bound:
                    self.CC_auto_stop_n_go = True
                    slow_down_speed = 0.1


                #'''
                #if (slow_down_speed < 0):
                if (slow_down_speed < 0):
                    ttt2 = (self.position-cfg.CCZ_DEC2_LEN)/cfg.MAX_SPEED + ((cfg.CCZ_DEC2_LEN) / ((self.speed_in_int+0)/2))
                    print(self.ID, slow_down_speed, time_bound, self.OT+self.D, ttt2)
                    print('T', T)
                    print('a', a)
                    print('b', b)
                    print('c', c)
                    print('speed', speed)
                    print('self.OT+self.D', self.OT+self.D)
                    print('self.speed_in_int', self.speed_in_int)
                    print('front_car', self.CC_front_car.ID)
                #'''


                assert(slow_down_speed > 0)
                self.CC_slow_speed = slow_down_speed


                traci.vehicle.setMaxSpeed(self.ID, slow_down_speed)
                #dec_time = (cfg.CCZ_ACC_LEN) / ((speed+slow_down_speed)/2)
                dec_time = (self.position-(cfg.CCZ_ACC_LEN+cfg.CCZ_DEC2_LEN)) / ((speed+slow_down_speed)/2)

                traci.vehicle.slowDown(self.ID,slow_down_speed, dec_time)
                self.CC_stage = '(auto)2 decelerate'
                self.CC_auto_slow_down_speed = slow_down_speed


        elif self.CC_stage == '(auto)2 decelerate':



            #'''
            if self.position < (cfg.CCZ_ACC_LEN+cfg.CCZ_DEC2_LEN):

                if self.CC_auto_stop_n_go == True:
                    self.CC_stage = '(auto) 2.5 stop and wait'
                    traci.vehicle.setSpeed(self.ID, 0.0001)

                    acc_time = (self.position-(cfg.CCZ_DEC2_LEN)) / ((cfg.MAX_SPEED+0)/2)
                    dec_time = (cfg.CCZ_DEC2_LEN) / ((self.speed_in_int+0)/2)
                    self.CC_stop_n_go_remained = self.OT+self.D-(acc_time+dec_time)
                    self.CC_stop_n_go = self.OT+self.D-(acc_time+dec_time)
                else:
                    traci.vehicle.setSpeed(self.ID, self.CC_auto_slow_down_speed)
                    self.CC_stage = '(auto) 3 const'
            '''
            elif self.position < (cfg.CCZ_ACC_LEN+cfg.CCZ_DEC2_LEN) + cfg.TIME_STEP* traci.vehicle.getSpeed(self.ID):
                if self.CC_auto_stop_n_go == False:
                    self.CC_stage = '(auto) 3 const'
                '''
            '''
            if self.position < (cfg.CCZ_ACC_LEN+cfg.CCZ_DEC2_LEN)+ cfg.TIME_STEP* traci.vehicle.getSpeed(self.ID):

                if self.CC_auto_stop_n_go == True:
                    self.CC_stage = '(auto) 2.5 stop and wait'
                    traci.vehicle.setSpeed(self.ID, 0.0001)

                    acc_time = (cfg.CCZ_ACC_LEN) / ((cfg.MAX_SPEED+0)/2)
                    dec_time = (cfg.CCZ_DEC2_LEN) / ((self.speed_in_int+0)/2)
                    self.CC_stop_n_go_remained = self.OT+self.D-(acc_time+dec_time)
                    self.CC_stop_n_go_remained -= cfg.TIME_STEP
                else:
                    self.CC_stage = '(auto) 3 const'
                    '''

        elif self.CC_stage == '(auto) 2.5 stop and wait':
            self.CC_stop_n_go_remained -= cfg.TIME_STEP
            if self.CC_stop_n_go_remained < cfg.TIME_STEP:
                self.CC_stage = '(auto) 3 const'

        elif self.CC_stage == '(auto) 3 const':
            self.CC_stage = '5 const_max_speed'
            delta = (cfg.CCZ_ACC_LEN + cfg.CCZ_DEC2_LEN)-self.position
            dec_time = (cfg.CCZ_ACC_LEN-delta) / ((cfg.MAX_SPEED+self.CC_slow_speed)/2)
            traci.vehicle.setMaxSpeed(self.ID, cfg.MAX_SPEED)
            traci.vehicle.slowDown(self.ID, cfg.MAX_SPEED, dec_time)


        #if self.ID == 'S_18' or self.ID == 'S_20':
            #print(self.ID, self.OT+self.D)
        '''
        if self.ID == 'S_1099' :
            print(self.ID, self.CC_stage, self.CC_front_car.ID, self.CC_shift, self.CC_auto_slow_down_speed, self.OT+self.D)
        #'''


        return None



    def CC_set_affected(self, front_car):
        self.CC_affected_by_front = True

    def CC_get_front_speed(self):
        if self.CC_front_car.CC_stage == '(auto)1.5 following':
            return self.CC_front_car.CC_get_front_speed()
        elif self.CC_front_car.CC_stage == '7 enter_inter':
            return cfg.MAX_SPEED
        else:
            return traci.vehicle.getSpeed(self.CC_front_car.ID) + traci.vehicle.getAcceleration(self.CC_front_car.ID)*cfg.TIME_STEP



    # Compute the shifts
    def CC_get_shifts(self, car_list):

        # 1.1 Determine how much to advance the car acceleration (shift_end)
        shifting_end = cfg.CCZ_DEC2_LEN
        front_car = self.CC_front_car

        if front_car != None and self.CC_affected_by_front == True:
            # If the distance between two car is smaller than the headway, then keep their distance
            if self.CC_front_pos_diff < front_car.length + cfg.HEADWAY:
                shifting_end = front_car.CC_shift_end + self.CC_front_pos_diff
            else:
                shifting_end = front_car.CC_shift_end + front_car.length + cfg.HEADWAY
        self.CC_shift_end = shifting_end
        #self.CC_shift_end = cfg.CCZ_DEC2_LEN



        # 1.2 Determine the upperbound of the delaying for a car to accelerate
        cc_shift_max = cfg.CCZ_LEN-shifting_end-2*cfg.CCZ_ACC_LEN
        if front_car != None and self.CC_affected_by_front == True and front_car.CC_slow_speed < cfg.MAX_SPEED:
            # First, assume that the two cars decelerate at the same time (So the car has this shift)
            cc_shift_max = front_car.CC_shift - self.CC_front_pos_diff

            # The space between two cars (might < 0, because might smaller than HEADWAY)
            space_between_two = max(self.CC_front_pos_diff - front_car.length - cfg.HEADWAY, 0)

            #2 Compute catch up time and reflect to the space
            catch_up_t = space_between_two/(cfg.MAX_SPEED-front_car.CC_slow_speed)
            cc_shift_max += catch_up_t*cfg.MAX_SPEED


            # 3 Determine if the car is going to follow the front car
            #  The time spent at slow down speed
            inner_delay_front_car = front_car.D
            inner_delay_front_car -= ((2*cfg.CCZ_DEC2_LEN/(cfg.MAX_SPEED+front_car.speed_in_int))-cfg.CCZ_DEC2_LEN/cfg.MAX_SPEED)
            inner_delay_front_car -= 2* (cfg.CCZ_ACC_LEN * 2 / (cfg.MAX_SPEED+front_car.CC_slow_speed) - cfg.CCZ_ACC_LEN / cfg.MAX_SPEED)

            # case 1: The car is not going to catch up at all ()
            if catch_up_t > inner_delay_front_car:
                cc_shift_max = cfg.CCZ_LEN-2*cfg.CCZ_ACC_LEN-cfg.CCZ_DEC2_LEN
                self.CC_following = False
            # case 2: The car is catching up
            else:
                self.CC_following = True


            # Setup a bound for SUMO simulation
            # It requires some slow-down space, but it's too small for sumo for SUMO to reflect the slow-down time
            if self.CC_following == True and cfg.CCZ_LEN-shifting_end-2*cfg.CCZ_ACC_LEN-cc_shift_max < cfg.MAX_SPEED*cfg.TIME_STEP:
                cc_shift_max = cfg.CCZ_LEN-shifting_end-2*cfg.CCZ_ACC_LEN-cfg.MAX_SPEED*cfg.TIME_STEP



            '''
            if self.CC_following==True and cc_shift_max-self.length+cfg.BZ_LEN < 0:
                print("ID", self.ID)
                print("lane", self.lane)
                print("D", self.D)
                print("OT", self.OT)
                print("CC_slow_speed", self.CC_slow_speed)
                print("CC_affected_by_front", self.CC_affected_by_front)
                print("CC_stop_n_go", self.CC_stop_n_go)
                print("CC_shift", self.CC_shift)
                print("CC_shift_end", self.CC_shift_end)
                print("CC_following", self.CC_following)
                print("++++++++++===========")
            '''
            #assert(self.CC_following==False or cc_shift_max > 0)
            #assert(self.CC_following==False or cc_shift_max-self.length > 0)
            #assert(cfg.CCZ_LEN-shifting_end-2*cfg.CCZ_ACC_LEN > 0)
            # DEBUG: see how to handle congestion

            # Hard upper bound of the delaying
            cc_shift_max = min(cc_shift_max, cfg.CCZ_LEN-shifting_end-2*cfg.CCZ_ACC_LEN, cfg.CCZ_LEN-shifting_end-cfg.CCZ_ACC_LEN-self.length)


            '''
            if self.ID == 'S_789':
                print("ID", self.ID)
                print("lane", self.lane)
                print("D", self.D)
                print("OT", self.OT)
                print("CC_slow_speed", self.CC_slow_speed)
                print("CC_affected_by_front", self.CC_affected_by_front)
                print("CC_stop_n_go", self.CC_stop_n_go)
                print("CC_shift", self.CC_shift)
                print("CC_shift_end", self.CC_shift_end)
                print("CC_following", self.CC_following)
                print("cc_shift_max", cc_shift_max)
                print("=======")
                print("CC_front_pos_diff", self.CC_front_pos_diff)
                print("front_car.length", front_car.length)
                print("cfg.HEADWAY", cfg.HEADWAY)
                print("front_car.CC_shift_end", front_car.CC_shift_end)
                print("front_car.length", front_car.length)
                print("front_car.CC_shift_end", front_car.CC_shift_end)
                print("=======")
                '''





        # 1.3 Determine the delay it desires. Reserving for the following cars
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
                count_car_list[car_i].CC_front_pos_diff = count_car_list[car_i].position-count_car_list[car_i-1].position

                # Only update the car behind
                if car_i == 1:
                    count_car_list[car_i].CC_set_affected(self)


                # If the car has been scheduled
                if count_car_list[car_i].D != None:
                    count_distance = count_car_list[car_i].D * cfg.MAX_SPEED + count_car_list[car_i].length + cfg.HEADWAY
                    temp_D = count_car_list[car_i].D
                else:
                    count_distance = temp_D * cfg.MAX_SPEED + count_car_list[car_i].length + cfg.HEADWAY


            else:
                break

        reserve_shift = min(reserve_shift, cfg.CCZ_LEN-shifting_end-2*cfg.CCZ_ACC_LEN, cfg.CCZ_LEN-shifting_end-cfg.CCZ_ACC_LEN-self.length)



        # 1.4 Decide the final shift
        shifting = min(reserve_shift, cc_shift_max)


        #'''
        CC_is_auto_speed_control = False
        if (self.CC_front_car != None) and ((shifting == cc_shift_max and cc_shift_max < reserve_shift) or (shifting == min(cfg.CCZ_LEN-shifting_end-2*cfg.CCZ_ACC_LEN, cfg.CCZ_LEN-shifting_end-cfg.CCZ_ACC_LEN-self.length))):
            #print(self.ID, "Potential contestion????", reserve_shift, cc_shift_max, self.CC_following)
            CC_is_auto_speed_control = True

        if (self.CC_front_car != None) and (self.CC_front_car.OT+self.CC_front_car.D-self.CC_front_car.position/cfg.MAX_SPEED) > (self.position-self.CC_front_car.position-self.CC_front_car.length-cfg.HEADWAY)/cfg.MAX_SPEED:
            #The carr will catch up the front
            CC_is_auto_speed_control = True
        #'''
        if (self.CC_front_car != None) and (self.CC_front_car.CC_stage == '7 enter_inter' or not self.CC_front_car.ID in car_list):
            self.CC_front_car = None
            CC_is_auto_speed_control = False
            result = self.CC_get_shifts(car_list)
            return result

        if front_car != None and self.CC_following == True:
            shifting = cc_shift_max

        self.CC_shift = shifting

        return {'shifting': shifting, 'shifting_end': shifting_end, 'CC_is_auto_speed_control': CC_is_auto_speed_control}



    # Compute the slow-slow speed
    def CC_get_slow_down_speed(self):
        D = self.D
        AT = D + ((cfg.CCZ_LEN)/cfg.MAX_SPEED) - (self.CC_shift/cfg.MAX_SPEED) - ((self.CC_shift_end-cfg.CCZ_DEC2_LEN)/cfg.MAX_SPEED) - (2*cfg.CCZ_DEC2_LEN/(cfg.MAX_SPEED+self.speed_in_int))

        # ----\_________/----
        #     S   S2    S
        #

        S = cfg.CCZ_ACC_LEN
        S2 = cfg.CCZ_LEN - 2*cfg.CCZ_ACC_LEN - self.CC_shift - self.CC_shift_end
        T = ((cfg.CCZ_LEN)/cfg.MAX_SPEED)+D - ((self.CC_shift+self.CC_shift_end-cfg.CCZ_DEC2_LEN)/cfg.MAX_SPEED +2*cfg.CCZ_DEC2_LEN/(cfg.MAX_SPEED+self.speed_in_int))

        poly_coeff = [0]*3
        poly_coeff[0] = T
        poly_coeff[1] = -(S2 + 4*S - T*cfg.MAX_SPEED)
        poly_coeff[2] = -S2 * cfg.MAX_SPEED

        ans = np.roots(poly_coeff)
        speed = min(max(ans), cfg.MAX_SPEED)


        # Determine if there's stop and go
        if speed == 0:
            self.CC_stop_n_go = self.D - ((2*cfg.CCZ_DEC2_LEN/(cfg.MAX_SPEED+self.speed_in_int)) - (cfg.CCZ_DEC2_LEN/cfg.MAX_SPEED)) - (2*(2*cfg.CCZ_ACC_LEN/(0+cfg.MAX_SPEED)) - 2*cfg.CCZ_ACC_LEN/cfg.MAX_SPEED)
            self.CC_stop_n_go_remained = self.CC_stop_n_go


        self.CC_slow_speed = speed
