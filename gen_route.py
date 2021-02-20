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



def generate_routefile_with_src_dst(inter_size, arrival_rate, rand_seed, time_steps):

    path = "data/routes/"
    file_name = str(inter_size) + "_" + str(arrival_rate) + "_" + str(rand_seed)
    routes = open(path + file_name + ".rou.xml", "w")
    src_dst = open(path + file_name + "_src_dst" + ".json", "w")

    print("<routes>\n", file=routes)

    for i in range(5,10):
        vType_str = '\t<vType id="car%i" accel="200.0" decel="200.0" sigma="0.0" length="%i" minGap="%f" maxSpeed="%f" tau="%f" carFollowModel="CACC" color="255,255,255"/>' % (i, i, cfg.HEADWAY, cfg.MAX_SPEED, cfg.TIME_STEP);

        print(vType_str, file=routes)

    route_list = []

    car_src_dst_dict = dict()   # {car_id: (src, dst)}

    route_str = "\n"
    for x_idx in range(1, inter_size+1):

        y_idx = 1
        route_id = 0*inter_size + (x_idx-1)
        src_lane = "00%i"%(y_idx) + "_" + "00%i"%(x_idx) + "_1"
        route_str += "\t<route id=\""
        route_str += str(route_id)
        route_str += "\" edges=\""
        route_str += " " + str(src_lane)
        route_str += "\"/>\n"
        route_list.append(str(route_id))


        y_idx = inter_size
        route_id = 1*inter_size + (x_idx-1)

        src_lane = "00%i"%(x_idx) + "_" + "00%i"%(y_idx) + "_2"
        route_str += "\t<route id=\""
        route_str += str(route_id)
        route_str += "\" edges=\""
        route_str += " " + str(src_lane)
        route_str += "\"/>\n"
        route_list.append(str(route_id))


        y_idx = inter_size
        route_id = 2*inter_size + (inter_size - x_idx)
        src_lane = "00%i"%(y_idx) + "_" + "00%i"%(x_idx) + "_3"
        route_str += "\t<route id=\""
        route_str += str(route_id)
        route_str += "\" edges=\""
        route_str += " " + str(src_lane)
        route_str += "\"/>\n"
        route_list.append(str(route_id))

        y_idx = 1
        route_id = 3*inter_size + (inter_size - x_idx)
        src_lane = "00%i"%(x_idx) + "_" + "00%i"%(y_idx) + "_4"
        route_str += "\t<route id=\""
        route_str += str(route_id)
        route_str += "\" edges=\""
        route_str += " " + str(src_lane)
        route_str += "\"/>\n"
        route_list.append(str(route_id))

    print(route_str, file=routes)


    dir_prob = numpy.random.uniform(0, arrival_rate*1.5, len(route_list)).tolist()
    #print(dir_prob)
    #dir_prob = [arrival_rate]*len(route_list)

    vehNr = 0
    for i in range(time_steps):
        for idx_route in range(len(route_list)):
            route = route_list[idx_route]
            if random.uniform(0, 1) < dir_prob[idx_route]:
                car_length = random.randrange(5,10)
                veh_str = "\t<vehicle id=\"car"
                lane_r = random.randrange(cfg.LANE_NUM_PER_DIRECTION)

                veh_str += '_%i" type="car%i" route="%s" depart="%i" departLane = "%i" departSpeed="%f"/>' % (vehNr, car_length, route, i, lane_r, cfg.MAX_SPEED);


                src_node_idx = int(route)
                # Genterate destination
                dst_node_idx = src_node_idx
                while (src_node_idx-dst_node_idx)%(cfg.INTER_SIZE*4) < cfg.INTER_SIZE-1 or (dst_node_idx-src_node_idx)%(cfg.INTER_SIZE*4) < cfg.INTER_SIZE-1:
                    dst_node_idx = random.randrange(0,cfg.INTER_SIZE*4)

                #while src_node_idx == dst_node_idx:
                #    dst_node_idx = random.randrange(0,cfg.INTER_SIZE*4)

                car_src_dst_dict["car_"+str(vehNr)] = (src_node_idx, dst_node_idx)

                print(veh_str, file=routes)

                vehNr += 1


    print("</routes>", file=routes)

    json.dump(car_src_dst_dict, src_dst)

def generate_routefile_with_src_dst_event(inter_size, arrival_rate, rand_seed, time_steps):

    path = "data/routes/"
    file_name = str(inter_size) + "_" + str(arrival_rate) + "_" + str(rand_seed)
    routes = open(path + file_name + ".rou.xml", "w")
    src_dst = open(path + file_name + "_src_dst" + ".json", "w")

    print("<routes>\n", file=routes)

    for i in range(5,10):
        vType_str = '\t<vType id="car%i" accel="200.0" decel="200.0" sigma="0.0" length="%i" minGap="%f" maxSpeed="%f" tau="%f" carFollowModel="CACC" color="255,255,255"/>' % (i, i, cfg.HEADWAY, cfg.MAX_SPEED, cfg.TIME_STEP);

        print(vType_str, file=routes)

    route_list = []

    car_src_dst_dict = dict()   # {car_id: (src, dst)}

    route_str = "\n"
    for x_idx in range(1, inter_size+1):

        y_idx = 1
        route_id = 0*inter_size + (x_idx-1)
        src_lane = "00%i"%(y_idx) + "_" + "00%i"%(x_idx) + "_1"
        route_str += "\t<route id=\""
        route_str += str(route_id)
        route_str += "\" edges=\""
        route_str += " " + str(src_lane)
        route_str += "\"/>\n"
        route_list.append(str(route_id))


        y_idx = inter_size
        route_id = 1*inter_size + (x_idx-1)

        src_lane = "00%i"%(x_idx) + "_" + "00%i"%(y_idx) + "_2"
        route_str += "\t<route id=\""
        route_str += str(route_id)
        route_str += "\" edges=\""
        route_str += " " + str(src_lane)
        route_str += "\"/>\n"
        route_list.append(str(route_id))


        y_idx = inter_size
        route_id = 2*inter_size + (inter_size - x_idx)
        src_lane = "00%i"%(y_idx) + "_" + "00%i"%(x_idx) + "_3"
        route_str += "\t<route id=\""
        route_str += str(route_id)
        route_str += "\" edges=\""
        route_str += " " + str(src_lane)
        route_str += "\"/>\n"
        route_list.append(str(route_id))

        y_idx = 1
        route_id = 3*inter_size + (inter_size - x_idx)
        src_lane = "00%i"%(x_idx) + "_" + "00%i"%(y_idx) + "_4"
        route_str += "\t<route id=\""
        route_str += str(route_id)
        route_str += "\" edges=\""
        route_str += " " + str(src_lane)
        route_str += "\"/>\n"
        route_list.append(str(route_id))

    print(route_str, file=routes)


    #dir_prob = numpy.random.uniform(0, arrival_rate*1.5, len(route_list)).tolist()
    #print(dir_prob)
    dir_prob = [0]*len(route_list)
    dir_prob[0] = arrival_rate

    dir_prob[-2] = arrival_rate

    vehNr = 0
    for i in range(time_steps):
        for idx_route in range(len(route_list)):
            route = route_list[idx_route]
            if random.uniform(0, 1) < dir_prob[idx_route]:
                car_length = random.randrange(5,10)
                veh_str = "\t<vehicle id=\"car"
                lane_r = random.randrange(cfg.LANE_NUM_PER_DIRECTION)

                veh_str += '_%i" type="car%i" route="%s" depart="%i" departLane = "%i" departSpeed="%f"/>' % (vehNr, car_length, route, i, lane_r, cfg.MAX_SPEED);


                src_node_idx = int(route)
                # Genterate destination
                dst_node_idx = src_node_idx
                while (src_node_idx-dst_node_idx)%(cfg.INTER_SIZE*4) < cfg.INTER_SIZE*2-1 or (dst_node_idx-src_node_idx)%(cfg.INTER_SIZE*4) < cfg.INTER_SIZE*2-1:
                    dst_node_idx = random.randrange(0,cfg.INTER_SIZE*4)

                #while src_node_idx == dst_node_idx:
                #    dst_node_idx = random.randrange(0,cfg.INTER_SIZE*4)

                car_src_dst_dict["car_"+str(vehNr)] = (src_node_idx, dst_node_idx)

                print(veh_str, file=routes)

                vehNr += 1


    print("</routes>", file=routes)

    json.dump(car_src_dst_dict, src_dst)


def generate_routefile(arrival_rate):
    idllist=[]
    # demand per second from different directions
    #dir_prob = [0.3, 0.3, 0.3, 0.3]
    #dir_prob = [0.8, 0.8, 0.8, 0.8]
    dir_prob = [arrival_rate]*4

    # Turning probability (L, S, R)
    #turn_prob =[1./32, 1./32, 15./16]
    turn_prob =[1./3, 1./3, 1./3]


    with open("data/icacc+.rou.xml", "w") as routes:
        print("<routes>\n", file=routes)

        for i in range(5,10):
            vType_str = '\t<vType id="car%i" accel="200.0" decel="200.0" sigma="0.0" length="%i" minGap="%f" maxSpeed="%f" tau="%f" carFollowModel="CACC" color="255,255,255"/>' % (i, i, cfg.HEADWAY, cfg.MAX_SPEED, cfg.TIME_STEP);

            print(vType_str, file=routes)


        route_str = "\n"
        for intersection_idx in range(2):
            intersection_id = "00%i"%(intersection_idx+1) + '_' + "001"
            for src_idx in range(1,5):
                for dst_idx in range(1,5):
                    if src_idx != dst_idx:
                        route_str += "\t<route id=\"route"
                        route_str += str(intersection_idx)+'_'+str(src_idx)+'_'+str(dst_idx)
                        route_str += "\" edges=\""
                        route_str += " " + intersection_id+'_'+str(src_idx)
                        route_str += "\"/>\n"


        print(route_str, file=routes)

        vehNr = 0
        for i in range(cfg.N_TIME_STEP):

            # intersection 1
            intersection_id = "001_001"

            for in_direction in [1, 2, 4]:
                if random.uniform(0, 1) < dir_prob[in_direction-1]:
                    #dir_r = random.randrange(3)+1
                    dir_r = numpy.random.choice(numpy.arange(1, 4), p=turn_prob)
                    #lane_r = random.randrange(cfg.LANE_NUM_PER_DIRECTION)
                    lane_r = 0
                    car_length = random.randrange(5,10)


                    veh_str = "\t<vehicle id=\""
                    if dir_r == 1:
                        veh_str += "L"
                        lane_r = 2
                    elif dir_r == 2:
                        veh_str += "S"
                        lane_r = 1
                    elif dir_r == 3:
                        veh_str += "R"
                        lane_r = 0

                    dst_direction = (in_direction-1 + dir_r)%4 + 1
                    if dst_direction == 3:
                        dir_r = numpy.random.choice(numpy.arange(1, 4), p=turn_prob)
                        if dir_r == 1:
                            veh_str += "L"
                        elif dir_r == 2:
                            veh_str += "S"
                        elif dir_r == 3:
                            veh_str += "R"
                    else:
                        veh_str += "X"  # Doesn't matter

                    veh_str += '_%i" type="car%i" route="route%s" depart="%i" departLane = "%i" departSpeed="%f"/>' % (vehNr, car_length, '0_'+str(in_direction)+'_'+str(dst_direction), i, lane_r, cfg.MAX_SPEED);
                    print(veh_str, file=routes)

                    vehNr += 1

                    #print("</routes>", file=routes)
                    #return(idllist)

            # intersection 2
            intersection_id = "002_001"

            for in_direction in [2, 3, 4]:
                if random.uniform(0, 1) < dir_prob[in_direction-1]:
                    #dir_r = random.randrange(3)+1
                    dir_r = numpy.random.choice(numpy.arange(1, 4), p=turn_prob)
                    #lane_r = random.randrange(cfg.LANE_NUM_PER_DIRECTION)
                    lane_r = 0
                    car_length = random.randrange(5,10)


                    veh_str = "\t<vehicle id=\""
                    if dir_r == 1:
                        veh_str += "L"
                        lane_r = 2
                    elif dir_r == 2:
                        veh_str += "S"
                        lane_r = 1
                    elif dir_r == 3:
                        veh_str += "R"
                        lane_r = 0

                    dst_direction = (in_direction-1 + dir_r)%4 + 1
                    if dst_direction == 1:
                        dir_r = numpy.random.choice(numpy.arange(1, 4), p=turn_prob)
                        if dir_r == 1:
                            veh_str += "L"
                        elif dir_r == 2:
                            veh_str += "S"
                        elif dir_r == 3:
                            veh_str += "R"
                    else:
                        veh_str += "X"  # Doesn't matter

                    veh_str += '_%i" type="car%i" route="route%s" depart="%i" departLane = "%i" departSpeed="%f"/>' % (vehNr, car_length, '1_'+str(in_direction)+'_'+str(dst_direction), i, lane_r, cfg.MAX_SPEED);
                    print(veh_str, file=routes)

                    vehNr += 1

                    #print("</routes>", file=routes)
                    #return(idllist)

        print("</routes>", file=routes)
    return(vehNr)
