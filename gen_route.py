from __future__ import absolute_import
from __future__ import print_function

import os
import sys

import optparse
import random
import numpy

import traci

import config as cfg
import json

# Route id = sink id in MiniVnet


''' ###############################
#Notation
        2
        |
    1 --i-- 3   1 --i-- 3 ...
        |
        4
############################### '''


def generate_routefile_with_src_dst(inter_size, arrival_rate, rand_seed, time_steps):

    path = "data/routes/"
    file_name = str(inter_size) + "_" + str(arrival_rate) + "_" + str(rand_seed)
    routes = open(path + file_name + ".rou.xml", "w")
    src_dst = open(path + file_name + "_src_dst" + ".json", "w")

    print("<routes>\n", file=routes)

    for i in range(5,10):
        vType_str = '\t<vType id="car%i" accel="100.0" decel="100.0" sigma="0.0" length="%i" minGap="%f" maxSpeed="%f" tau="%f" carFollowModel="CACC" color="255,255,255"/>' % (i, i, cfg.HEADWAY, cfg.MAX_SPEED, cfg.TIME_STEP);

        print(vType_str, file=routes)

    route_list = []

    car_dict = dict()

    route_str = "\n"
    rout_to_car_coordination = dict()
    for x_idx in range(1, inter_size+1):

        y_idx = 1
        route_id = 0*inter_size + (x_idx-1)
        src_lane = "%3.3o"%(y_idx) + "_" + "%3.3o"%(x_idx) + "_1"
        route_str += "\t<route id=\""
        route_str += str(route_id)
        route_str += "\" edges=\""
        route_str += " " + str(src_lane)
        route_str += "\"/>\n"
        route_list.append(str(route_id))
        rout_to_car_coordination[route_id] = (y_idx-1, x_idx)

        y_idx = inter_size
        route_id = 1*inter_size + (x_idx-1)
        src_lane = "%3.3o"%(x_idx) + "_" + "%3.3o"%(y_idx) + "_2"
        route_str += "\t<route id=\""
        route_str += str(route_id)
        route_str += "\" edges=\""
        route_str += " " + str(src_lane)
        route_str += "\"/>\n"
        route_list.append(str(route_id))
        rout_to_car_coordination[route_id] = (x_idx, y_idx+1)


        y_idx = inter_size
        route_id = 2*inter_size + (inter_size - x_idx)
        src_lane = "%3.3o"%(y_idx) + "_" + "%3.3o"%(x_idx) + "_3"
        route_str += "\t<route id=\""
        route_str += str(route_id)
        route_str += "\" edges=\""
        route_str += " " + str(src_lane)
        route_str += "\"/>\n"
        route_list.append(str(route_id))
        rout_to_car_coordination[route_id] = (y_idx+1, x_idx)

        y_idx = 1
        route_id = 3*inter_size + (inter_size - x_idx)
        src_lane = "%3.3o"%(x_idx) + "_" + "%3.3o"%(y_idx) + "_4"
        route_str += "\t<route id=\""
        route_str += str(route_id)
        route_str += "\" edges=\""
        route_str += " " + str(src_lane)
        route_str += "\"/>\n"
        route_list.append(str(route_id))
        rout_to_car_coordination[route_id] = (x_idx, y_idx-1)

    print(route_str, file=routes)


    vehNr = 0
    # For every source node, generate CAVs with some distribution
    for route in route_list:
        # Exponential distribution for arrival headway (Poisson process)
        time_step = random.expovariate(arrival_rate)
        while time_step <= time_steps:
            # Generate one CAV
            car_length = random.randrange(5,10) # Car length
            #lane_r = random.randrange(cfg.LANE_NUM_PER_DIRECTION)
            lane_r = 0    # Currently, only one lane

            src_node_idx = int(route)   # Source node index
            car_facing_direction = int(src_node_idx//inter_size)
            # 0: E,  1: S,  2: W,  3: N

            init_x, init_y = rout_to_car_coordination[int(route)] #Initial position (x, y)
            car_coord_position = [init_x, init_y]

            # Generate routes
            car_route_str = ""
            while True:
                # Move the car
                if car_facing_direction == 0:   # Move east: x+1
                    car_coord_position[0] += 1
                elif car_facing_direction == 1:    # Move south: y-1
                    car_coord_position[1] -= 1
                elif car_facing_direction == 2:   # Move west: x-1
                    car_coord_position[0] -= 1
                elif car_facing_direction == 3:    # Move north: y+1
                    car_coord_position[1] += 1

                # Check if the car reach the edge
                if car_coord_position[0] == 0:  # Reach x edge
                    break
                elif car_coord_position[0] == inter_size+1:  # Reach x edge
                    break
                elif car_coord_position[1] == 0:  # Reach y edge
                    break
                elif car_coord_position[1] == inter_size+1:  # Reach y edge
                    break

                # Choose the turning
                turning = random.choice(["R", "S", "L"])
                car_route_str += turning
                # Change the facing direction
                if turning == "R":
                    car_facing_direction = (car_facing_direction+1) % 4
                elif turning == "L":
                    car_facing_direction = (car_facing_direction-1) % 4

                # Restart if the route is too long
                if len(car_route_str) > 2*inter_size-1:
                    # Reset the status
                    car_facing_direction = int(int(route)//inter_size)
                    init_x, init_y = rout_to_car_coordination[int(route)] #Initial position (x, y)
                    car_coord_position = [init_x, init_y]
                    car_route_str = ""
                    continue

            car_dict[(time_step, vehNr)] = ("car_"+str(vehNr)+"_"+car_route_str,
                                            car_length, route, time_step, lane_r,
                                            cfg.MAX_SPEED)
            vehNr += 1

            time_step += random.expovariate(arrival_rate)

    sorted_car_list = sorted(car_dict.items(), key=lambda x: x[0][0])

    for key, car in sorted_car_list:
        # Build CAV profile for SUMO
        veh_str = "\t<vehicle id=\""
        veh_str += '%s" type="car%i" ' % (car[0], car[1])
        veh_str += 'route="%s" depart="%i" ' % (car[2], car[3])
        veh_str += 'departLane = "%i" departSpeed="%f"/>' % (car[4], car[5])
        print(veh_str, file=routes)

    print("</routes>", file=routes)

    json.dump(sorted_car_list, src_dst)
