import copy
import random
import config as cfg
from random import randrange
import json
import math

RESOLUTION = 2  # According to gen_advise.cpp

class LaneAdviser:
    def __init__(self):
        # Number of the boxes in one dimention
        self.num_di = RESOLUTION*(cfg.LANE_NUM_PER_DIRECTION*2+1)
        # 2D matrix of timing for advising
        self.timeMatrix = [[0]* self.num_di for i in range(self.num_di)]
        self.ori_timeMatrix = [[0]* self.num_di for i in range(self.num_di)]


        with open('advise_info/advise_info'+str(cfg.LANE_NUM_PER_DIRECTION)+'.json', 'r') as file:
            self.lane_dic = json.load(file)

        # Number of advised car on each lane
        self.laneCount = [0]*(cfg.LANE_NUM_PER_DIRECTION*4)



    '''
        Coordinates:
            origin: upper left corner (0, 0)
            x-axis: vertical ray extending southward from origin
            y-axis: horizontal ray extending eastward from origin
    '''


    # Public functions

    # Get the to pass the intersection with certain direction
    def get_max_time(self, lane, direction):
        return self.get_max_time_private(lane, direction, self.timeMatrix)

    def get_max_time_ori(self, lane, direction):
        return self.get_max_time_private(lane, direction, self.ori_timeMatrix)


    # Get the number of advised cars
    def get_lane_count(self, lane):
        return self.laneCount[lane]

    # Update the table (final version for simulation usage)
    def update_table(self, n_sched_car):
        self.reset_advise_table()
        for car in n_sched_car:
            self.update_iccac(car.lane, car.turning, car.OT+car.D)
        self.done_update_iccac()

    def done_update_iccac(self):
        self.ori_timeMatrix = copy.deepcopy(self.timeMatrix)

    # Update the table with given results (delays)
    def update_iccac(self, lane, direction, time):
        self.update_iccac_private(lane, direction, time, self.timeMatrix)

    # Reset the records
    def reset_advise_table(self):
        self.timeMatrix = [[0] * self.num_di for i in range(self.num_di)]
        self.laneCount = [0]*(cfg.LANE_NUM_PER_DIRECTION*4)

    # Update the value after the advise
    def update_after_advise(self, lane, direction, diff_time):
        n1 = self.get_max_time(lane, direction)
        self.update_after_advise_private(lane, direction, diff_time, self.timeMatrix)
        n2 = self.get_max_time(lane, direction)
        self.laneCount[lane]+=1

        '''
        if (lane < 2):
            print("Update: ", lane, direction, n1, n2, diff_time)
        #'''
        #TODO: DEBUG

    # Print the matrix for debugging
    def print_matrix(self):
        for mat in self.timeMatrix:
            print(mat)



    def get_max_time_private(self, lane, direction, timeMatrix):

        # get the earliest time when a car can drive
        # through lane "lane" in direction "direction".

        # lane : int
        # direction : string

        temp = []
        quotient = lane // cfg.LANE_NUM_PER_DIRECTION

        ans = [0,0]
        max_val = -1

        if quotient == 0:
            # no rotation
            for e in self.lane_dic[str(lane % cfg.LANE_NUM_PER_DIRECTION) + direction]:
                temp.append(timeMatrix[e[0]][e[1]])

                if timeMatrix[e[0]][e[1]] > max_val:
                    max_val = timeMatrix[e[0]][e[1]]
                    ans[0] = e[0]
                    ans[1] = e[1]

        elif quotient == 1:
            # rotate 90 degree clockwise
            for e in self.lane_dic[str(lane % cfg.LANE_NUM_PER_DIRECTION) + direction]:
                temp.append(timeMatrix[self.num_di-1 - e[1]][e[0]])

        elif quotient == 2:
            # rotate 180 degree clockwise
            for e in self.lane_dic[str(lane % cfg.LANE_NUM_PER_DIRECTION) + direction]:
                temp.append(timeMatrix[self.num_di-1 - e[0]][self.num_di-1 - e[1]])

        else:  # quotient == 4
            # rotate 270 degree clockwise
            for e in self.lane_dic[str(lane % cfg.LANE_NUM_PER_DIRECTION) + direction]:
                temp.append(timeMatrix[e[1]][self.num_di-1 - e[0]])


        return max(temp)


    def update_iccac_private(self, lane, direction, time, timeMatrix):

        # Update all the squares on the trajectory "lane + direction" to "time".

        quotient = (lane) // cfg.LANE_NUM_PER_DIRECTION

        if quotient == 0:
            # no rotation
            for e in self.lane_dic[str(lane % cfg.LANE_NUM_PER_DIRECTION) + direction]:
                if (time > timeMatrix[e[0]][e[1]]):
                    timeMatrix[e[0]][e[1]] = time


        elif quotient == 1:
            # rotate 90 degree clockwise
            for e in self.lane_dic[str(lane % cfg.LANE_NUM_PER_DIRECTION) + direction]:
                if (time > timeMatrix[self.num_di-1 - e[1]][e[0]]):
                    timeMatrix[self.num_di-1 - e[1]][e[0]] = time


        elif quotient == 2:
            # rotate 180 degree clockwise
            for e in self.lane_dic[str(lane % cfg.LANE_NUM_PER_DIRECTION) + direction]:
                if (time > timeMatrix[self.num_di-1 - e[0]][self.num_di-1 - e[1]]):
                    timeMatrix[self.num_di-1 - e[0]][self.num_di-1 - e[1]] = time


        else:  # quotient == 3
            # rotate 270 degree clockwise
            for e in self.lane_dic[str(lane % cfg.LANE_NUM_PER_DIRECTION) + direction]:
                if (time > timeMatrix[e[1]][self.num_di-1 - e[0]]):
                    timeMatrix[e[1]][self.num_di-1 - e[0]] = time


    def update_after_advise_private(self, lane, direction, diff_time, timeMatrix):

        # Update all the squares on the trajectory "lane + direction" to "diff_time".

        quotient = lane // cfg.LANE_NUM_PER_DIRECTION


        if quotient == 0:
            # lane 0 or 1, no rotation
            for e in self.lane_dic[str(lane % cfg.LANE_NUM_PER_DIRECTION) + direction]:
                timeMatrix[e[0]][e[1]] += diff_time

        elif quotient == 1:
            # lane 2 or 3, rotate 90 degree clockwise
            for e in self.lane_dic[str(lane % cfg.LANE_NUM_PER_DIRECTION) + direction]:
                timeMatrix[self.num_di-1 - e[1]][e[0]] += diff_time

        elif quotient == 2:
            # lane 4 or 5, rotate 180 degree clockwise
            for e in self.lane_dic[str(lane % cfg.LANE_NUM_PER_DIRECTION) + direction]:
                timeMatrix[self.num_di-1 - e[0]][self.num_di-1 - e[1]] += diff_time

        else:  # quotient == 3
            # lane 6 or 7, rotate 270 degree clockwise
            for e in self.lane_dic[str(lane % cfg.LANE_NUM_PER_DIRECTION) + direction]:
                timeMatrix[e[1]][self.num_di-1 - e[0]] += diff_time


##########################################################

def GreedyAdvise(car, lane_advisor):
    start_lane = (car.lane//cfg.LANE_NUM_PER_DIRECTION)*cfg.LANE_NUM_PER_DIRECTION
    occup_time_list = [lane_advisor.get_max_time(start_lane+idx, car.turning) for idx in range(cfg.LANE_NUM_PER_DIRECTION)]
    #'''
    for idx in range(len(occup_time_list)):
        occup_time_list[idx]+=lane_advisor.get_lane_count(start_lane+idx)
    #'''
    # Equal cost advise
    min_idx = occup_time_list.index(min(occup_time_list))

    occup_time = 0

    if car.turning == "S":
        # Update cost
        occup_time = (cfg.LANE_WIDTH*(cfg.LANE_NUM_PER_DIRECTION*2+1)+car.length)/cfg.MAX_SPEED
    elif car.turning == "R":
        lane_idx_turn = car.lane%cfg.LANE_NUM_PER_DIRECTION
        radius = cfg.LANE_NUM_PER_DIRECTION*2+1-lane_idx_turn-(cfg.LANE_NUM_PER_DIRECTION+1)
        occup_time=(cfg.LANE_WIDTH*(radius*math.pi*2/4)+car.length)/cfg.TURN_SPEED
    elif car.turning == "L":
        lane_idx_turn = car.lane%cfg.LANE_NUM_PER_DIRECTION
        radius = lane_idx_turn+cfg.LANE_NUM_PER_DIRECTION+1
        occup_time=(cfg.LANE_WIDTH*(radius*math.pi*2/4)+car.length)/cfg.TURN_SPEED

    lane_advisor.update_after_advise(car.lane,car.turning,occup_time)

    return cfg.LANE_NUM_PER_DIRECTION-min_idx-1 # The index of SUMO is reversed

def PreAssignedLaneAdvise(car):
    min_idx = car.lane%cfg.LANE_NUM_PER_DIRECTION
    if (cfg.LANE_NUM_PER_DIRECTION == 2):
        if car.turning == "L":
            if car.lane % 2 != 0:
                min_idx = 0


        elif car.turning == "R":
            if car.lane % 2 != 1:
                min_idx = 1
    elif (cfg.LANE_NUM_PER_DIRECTION > 2):
        if car.turning == "L":
            if car.lane % cfg.LANE_NUM_PER_DIRECTION >= (cfg.LANE_NUM_PER_DIRECTION+1)//3:
                min_idx = randrange(0,(cfg.LANE_NUM_PER_DIRECTION+1)//3)


        elif car.turning == "R":
            if car.lane % cfg.LANE_NUM_PER_DIRECTION <= cfg.LANE_NUM_PER_DIRECTION-(cfg.LANE_NUM_PER_DIRECTION+1)//3 -1:
                min_idx = randrange(cfg.LANE_NUM_PER_DIRECTION-(cfg.LANE_NUM_PER_DIRECTION+1)//3,cfg.LANE_NUM_PER_DIRECTION)


    return cfg.LANE_NUM_PER_DIRECTION-min_idx-1 # The index of SUMO is reversed


def HybridAdvise(car, lane_advisor):
    start_lane = (car.lane//cfg.LANE_NUM_PER_DIRECTION)*cfg.LANE_NUM_PER_DIRECTION
    occup_time_list = [lane_advisor.get_max_time_ori(start_lane+idx, car.turning) for idx in range(cfg.LANE_NUM_PER_DIRECTION)]

    if max(occup_time_list) >= (cfg.CCZ_LEN+cfg.GZ_LEN)/cfg.MAX_SPEED:
        return PreAssignedLaneAdvise(car)
    else:
        return GreedyAdvise(car, lane_advisor)

def HybridAdvise_min(car, lane_advisor):
    start_lane = (car.lane//cfg.LANE_NUM_PER_DIRECTION)*cfg.LANE_NUM_PER_DIRECTION
    occup_time_list = [lane_advisor.get_max_time_ori(start_lane+idx, car.turning) for idx in range(cfg.LANE_NUM_PER_DIRECTION)]

    if min(occup_time_list) >= (cfg.CCZ_LEN+cfg.GZ_LEN)/cfg.MAX_SPEED:
        return PreAssignedLaneAdvise(car)
    else:
        return GreedyAdvise(car, lane_advisor)

def HybridAdvise_avg(car, lane_advisor):
    start_lane = (car.lane//cfg.LANE_NUM_PER_DIRECTION)*cfg.LANE_NUM_PER_DIRECTION
    occup_time_list = [lane_advisor.get_max_time_ori(start_lane+idx, car.turning) for idx in range(cfg.LANE_NUM_PER_DIRECTION)]

    if sum(occup_time_list)/len(occup_time_list) >= (cfg.CCZ_LEN+cfg.GZ_LEN)/cfg.MAX_SPEED:
        return PreAssignedLaneAdvise(car)
    else:
        return GreedyAdvise(car, lane_advisor)
