from __future__ import absolute_import
from __future__ import print_function


import os
import sys
import optparse
import random
import numpy
import time
import numpy as np

import traceback

import sys
sys.path.append('..')
import config as cfg
import json
import math

def get_distance(data_i, data_j):
    diff_x = data_i["X"] - data_j["X"]
    diff_y = data_i["Y"] - data_j["Y"]
    distance = math.sqrt(diff_x*diff_x+diff_y*diff_y)
    return distance


###########################
# Main function
if __name__ == "__main__":
    print("Usage: python code.py")


    trajectory_list_dict = None
    with open("inter_data/sumo_inter_info"+str(cfg.LANE_NUM_PER_DIRECTION)+".json") as f:
        trajectory_list_dict = json.load(f)

    lane_info_dict = dict()

    for id_i, list_i in trajectory_list_dict.items():
        lane_num_i = int(id_i[:-1])
        if lane_num_i >= cfg.LANE_NUM_PER_DIRECTION:
            continue

        for id_j, list_j in trajectory_list_dict.items():
            lane_num_j = int(id_j[:-1])
            if lane_num_j <= lane_num_i:
                continue

            Xm = 999999
            Ym = 999999
            Xd = -1
            Yd = -1

            distance_diff_max_12 = -99999
            distance_diff_max_21 = -99999

            key_str = id_i+id_j
            is_potential_collision = False
            for data_i in list_i:
                for data_j in list_j:
                    distance = get_distance(data_i, data_j)
                    if distance < cfg.LANE_WIDTH/1.5:
                        is_potential_collision = True

                        if data_i['distance'] < Xm:
                            Xm = data_i['distance']
                        if data_j['distance'] < Ym:
                            Ym = data_j['distance']
                        if data_i['distance'] > Xd:
                            Xd = data_i['distance']
                        if data_j['distance'] > Yd:
                            Yd = data_j['distance']

                        #if key_str == '0S1S':
                            #print(key_str, data_i['distance'], data_j['distance'])

                        if distance_diff_max_12 < data_i['distance']-data_j['distance']:
                            # car i is at the front
                            distance_diff_max_12 = data_i['distance']-data_j['distance']
                        elif distance_diff_max_21 < data_j['distance']-data_i['distance']:
                            # car j is at the front
                            distance_diff_max_21 = data_j['distance']-data_i['distance']

            if is_potential_collision:
                #diff = abs(distance_diff_max_12-(-distance_diff_max_21))
                #if  diff < 2*cfg.LANE_WIDTH:
                    #print(key_str)
                lane_info_dict[key_str] = {'distance_diff_max_12':distance_diff_max_12, 'distance_diff_max_21':distance_diff_max_21}


    with open("../inter_info/sumo_lane_info"+str(cfg.LANE_NUM_PER_DIRECTION)+".json", 'w') as file:
                file.write(json.dumps(lane_info_dict))
