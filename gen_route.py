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

        for idx in range(1, 2+1):
            for jdx in range(1, 2+1):
                intersection_manager_id = "%3.3o"%(idx) + "_" + "%3.3o"%(jdx)
                for src_idx in range(1,5):
                    for dst_idx in range(1,5):
                        if src_idx != dst_idx:
                            route_str += "\t<route id=\"route"
                            route_str += intersection_manager_id +'_'+str(src_idx)+'_'+str(dst_idx)
                            route_str += "\" edges=\""
                            route_str += " " + intersection_manager_id+'_'+str(src_idx)
                            route_str += "\"/>\n"


        print(route_str, file=routes)

        vehNr = 0
        for i in range(cfg.N_TIME_STEP):

            # intersection 1
            intersection_id = "001_001"

            for in_direction in [1, 4]:
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
                    if dst_direction == 2:
                        dir_r = numpy.random.choice(numpy.arange(1, 4), p=turn_prob)
                        if dir_r == 1:
                            veh_str += "L"
                        elif dir_r == 2:
                            veh_str += "S"
                        elif dir_r == 3:
                            veh_str += "R"
                            veh_str += numpy.random.choice(["L", "S"])

                    elif dst_direction == 3:
                        dir_r = numpy.random.choice(numpy.arange(1, 4), p=turn_prob)
                        if dir_r == 1:
                            veh_str += "L"
                            veh_str += numpy.random.choice(["S", "R"])
                        elif dir_r == 2:
                            veh_str += "S"
                        elif dir_r == 3:
                            veh_str += "R"

                    veh_str += "X"  # Doesn't matter

                    veh_str += '_%i" type="car%i" route="route%s" depart="%i" departLane = "%i" departSpeed="%f"/>' % (vehNr, car_length, intersection_id+'_'+str(in_direction)+'_'+str(dst_direction), i, lane_r, cfg.MAX_SPEED);
                    print(veh_str, file=routes)

                    vehNr += 1


            # intersection 2
            intersection_id = "002_001"

            for in_direction in [3, 4]:
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
                            veh_str += numpy.random.choice(["L", "S"])

                    elif dst_direction == 2:
                        dir_r = numpy.random.choice(numpy.arange(1, 4), p=turn_prob)
                        if dir_r == 1:
                            veh_str += "L"
                            veh_str += numpy.random.choice(["S", "R"])
                        elif dir_r == 2:
                            veh_str += "S"
                        elif dir_r == 3:
                            veh_str += "R"

                    veh_str += "X"  # Doesn't matter

                    veh_str += '_%i" type="car%i" route="route%s" depart="%i" departLane = "%i" departSpeed="%f"/>' % (vehNr, car_length, intersection_id+'_'+str(in_direction)+'_'+str(dst_direction), i, lane_r, cfg.MAX_SPEED);
                    print(veh_str, file=routes)

                    vehNr += 1


            # intersection 3
            intersection_id = "001_002"

            for in_direction in [1, 2]:
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
                            veh_str += numpy.random.choice(["L", "S"])

                    elif dst_direction == 4:
                        dir_r = numpy.random.choice(numpy.arange(1, 4), p=turn_prob)
                        if dir_r == 1:
                            veh_str += "L"
                            veh_str += numpy.random.choice(["S", "R"])
                        elif dir_r == 2:
                            veh_str += "S"
                        elif dir_r == 3:
                            veh_str += "R"

                    veh_str += "X"  # Doesn't matter

                    veh_str += '_%i" type="car%i" route="route%s" depart="%i" departLane = "%i" departSpeed="%f"/>' % (vehNr, car_length, intersection_id+'_'+str(in_direction)+'_'+str(dst_direction), i, lane_r, cfg.MAX_SPEED);
                    print(veh_str, file=routes)

                    vehNr += 1

            # intersection 4
            intersection_id = "002_002"

            for in_direction in [2, 3]:
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
                            veh_str += numpy.random.choice(["R", "S"])
                        elif dir_r == 2:
                            veh_str += "S"
                        elif dir_r == 3:
                            veh_str += "R"

                    elif dst_direction == 4:
                        dir_r = numpy.random.choice(numpy.arange(1, 4), p=turn_prob)
                        if dir_r == 1:
                            veh_str += "L"
                        elif dir_r == 2:
                            veh_str += "S"
                        elif dir_r == 3:
                            veh_str += "R"
                            veh_str += numpy.random.choice(["S", "L"])

                    veh_str += "X"  # Doesn't matter

                    veh_str += '_%i" type="car%i" route="route%s" depart="%i" departLane = "%i" departSpeed="%f"/>' % (vehNr, car_length, intersection_id+'_'+str(in_direction)+'_'+str(dst_direction), i, lane_r, cfg.MAX_SPEED);
                    print(veh_str, file=routes)

                    vehNr += 1

        print("</routes>", file=routes)
    return(vehNr)
