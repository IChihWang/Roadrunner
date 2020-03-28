from __future__ import absolute_import
from __future__ import print_function

import os
import sys

import optparse
import random
import numpy

import traci

import config as cfg

def generate_routefile(arrival_rate):
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
            vType_str = '\t<vType id="car%i" accel="100.0" decel="100.0" sigma="0.0" length="%i" minGap="%f" maxSpeed="%f" tau="%f" carFollowModel="CACC" color="255,255,255"/>' % (i, i, cfg.HEADWAY, cfg.MAX_SPEED, cfg.TIME_STEP);

            print(vType_str, file=routes)

        route_list = []

        route_str = "\n"
        for x_idx in range(1, cfg.INTER_SIZE+1):
            y_idx = 1
            src_lane = "%3.3o"%(x_idx) + "_" + "%3.3o"%(y_idx) + "_4"
            route_str += "\t<route id=\"route"
            route_str += src_lane
            route_str += "\" edges=\""
            route_str += " " + str(src_lane)
            route_str += "\"/>\n"
            route_list.append("route" + src_lane)

            src_lane = "%3.3o"%(y_idx) + "_" + "%3.3o"%(x_idx) + "_1"
            route_str += "\t<route id=\"route"
            route_str += src_lane
            route_str += "\" edges=\""
            route_str += " " + str(src_lane)
            route_str += "\"/>\n"
            route_list.append("route" + src_lane)

            y_idx = cfg.INTER_SIZE
            src_lane = "%3.3o"%(x_idx) + "_" + "%3.3o"%(y_idx) + "_2"
            route_str += "\t<route id=\"route"
            route_str += src_lane
            route_str += "\" edges=\""
            route_str += " " + str(src_lane)
            route_str += "\"/>\n"
            route_list.append("route" + src_lane)

            src_lane = "%3.3o"%(y_idx) + "_" + "%3.3o"%(x_idx) + "_3"
            route_str += "\t<route id=\"route"
            route_str += src_lane
            route_str += "\" edges=\""
            route_str += " " + str(src_lane)
            route_str += "\"/>\n"
            route_list.append("route" + src_lane)



        '''
        route_str = "\n"
        for src_idx in range(1,5):
            for dst_idx in range(1,5):
                if src_idx != dst_idx:
                    route_str += "\t<route id=\"route"
                    route_str += str(src_idx)+'_'+str(dst_idx)
                    route_str += "\" edges=\""
                    route_str += " " + str(src_idx)
                    route_str += " -" + str(dst_idx)
                    route_str += "\"/>\n"
        '''


        print(route_str, file=routes)

        '''


        vehNr = 0
        for i in range(cfg.N_TIME_STEP):
            for idx in range(4):
                if random.uniform(0, 1) < dir_prob[idx]:
                    #dir_r = random.randrange(3)+1
                    dir_r = numpy.random.choice(numpy.arange(1, 4), p=turn_prob)
                    lane_r = random.randrange(cfg.LANE_NUM_PER_DIRECTION)
                    car_length = random.randrange(5,10)


                    veh_str = "\t<vehicle id=\""
                    if dir_r == 1:
                        veh_str += "L"
                        idl = "L"
                    elif dir_r == 2:
                        veh_str += "S"
                        idl = "S"
                    elif dir_r == 3:
                        veh_str += "R"
                        idl = "R"

                    veh_str += '_%i" type="car%i" route="route%s" depart="%i" departLane = "%i" departSpeed="%f"/>' % (vehNr, car_length, str(idx+1)+'_'+str((idx+dir_r)%4+1), i, lane_r, cfg.MAX_SPEED);
                    idl += '_%i'% (vehNr);
                    idllist.append(idl)
                    print(veh_str, file=routes)

                    vehNr += 1

                    #print("</routes>", file=routes)
                    #return(idllist)

        '''


        vehNr = 0
        for i in range(cfg.N_TIME_STEP):
            for route in route_list:
                if random.uniform(0, 1) < arrival_rate:
                    car_length = random.randrange(5,10)
                    veh_str = "\t<vehicle id=\"car"
                    lane_r = random.randrange(cfg.LANE_NUM_PER_DIRECTION)

                    veh_str += '_%i" type="car%i" route="%s" depart="%i" departLane = "%i" departSpeed="%f"/>' % (vehNr, car_length, route, i, lane_r, cfg.MAX_SPEED);
                    print(veh_str, file=routes)

                    vehNr += 1

                    #print("</routes>", file=routes)
                    #return(idllist)


        print("</routes>", file=routes)
