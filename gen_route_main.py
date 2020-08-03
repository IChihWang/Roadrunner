from __future__ import absolute_import
from __future__ import print_function

import os
import sys

import optparse
import random
import numpy

import json
from gen_route import generate_routefile_with_src_dst
import config as cfg


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

    car_src_dst_dict = dict()   # {car_id: (src, dst)}

    route_str = "\n"
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

        y_idx = inter_size
        route_id = 1*inter_size + (x_idx-1)
        src_lane = "%3.3o"%(x_idx) + "_" + "%3.3o"%(y_idx) + "_2"
        route_str += "\t<route id=\""
        route_str += str(route_id)
        route_str += "\" edges=\""
        route_str += " " + str(src_lane)
        route_str += "\"/>\n"
        route_list.append(str(route_id))


        y_idx = inter_size
        route_id = 2*inter_size + (inter_size - x_idx)
        src_lane = "%3.3o"%(y_idx) + "_" + "%3.3o"%(x_idx) + "_3"
        route_str += "\t<route id=\""
        route_str += str(route_id)
        route_str += "\" edges=\""
        route_str += " " + str(src_lane)
        route_str += "\"/>\n"
        route_list.append(str(route_id))

        y_idx = 1
        route_id = 3*inter_size + (inter_size - x_idx)
        src_lane = "%3.3o"%(x_idx) + "_" + "%3.3o"%(y_idx) + "_4"
        route_str += "\t<route id=\""
        route_str += str(route_id)
        route_str += "\" edges=\""
        route_str += " " + str(src_lane)
        route_str += "\"/>\n"
        route_list.append(str(route_id))

    print(route_str, file=routes)


    vehNr = 0
    for i in range(time_steps):
        for route in route_list:
            if random.uniform(0, 1) < arrival_rate:
                car_length = random.randrange(5,10)
                veh_str = "\t<vehicle id=\"car"
                lane_r = random.randrange(cfg.LANE_NUM_PER_DIRECTION)

                veh_str += '_%i" type="car%i" route="%s" depart="%i" departLane = "%i" departSpeed="%f"/>' % (vehNr, car_length, route, i, lane_r, cfg.MAX_SPEED);


                src_node_idx = int(route)
                # Genterate destination
                dst_node_idx = src_node_idx
                while src_node_idx == dst_node_idx:
                    dst_node_idx = random.randrange(0,inter_size*4)

                car_src_dst_dict["car_"+str(vehNr)] = (src_node_idx, dst_node_idx)

                print(veh_str, file=routes)

                vehNr += 1


    print("</routes>", file=routes)

    json.dump(car_src_dst_dict, src_dst)



# Main function
if __name__ == "__main__":
    total_time_steps = 3600  # 10 min

    for inter_size in range(2,50):
        for seed in range(10):
            random.seed(seed)  # make tests reproducible
            numpy.random.seed(seed)
            arrival_rate = 0.1
            while arrival_rate < 0.8:
                generate_routefile_with_src_dst(inter_size, arrival_rate, seed, total_time_steps)
                arrival_rate += 0.1
