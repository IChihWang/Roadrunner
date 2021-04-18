from __future__ import absolute_import
from __future__ import print_function

import os
import sys

import optparse
import random
import numpy

import traci
import sys
sys.path.append('..')
import config as cfg

def generate_one_car_routefile(in_lane_1, turning_1):
    t1 = 0
    dir_1 = in_lane_1 // cfg.LANE_NUM_PER_DIRECTION
    sub_lane_1 = in_lane_1 % cfg.LANE_NUM_PER_DIRECTION

    with open("data/icacc+.rou.xml", "w") as routes:
        print("<routes>\n", file=routes)

        assert sub_lane_1 < cfg.LANE_NUM_PER_DIRECTION, "lane num should be < " + str(cfg.LANE_NUM_PER_DIRECTION)

        lane_1 = cfg.LANE_NUM_PER_DIRECTION - sub_lane_1 -1

        vType_str = '\t<vType id="car" accel="100.0" decel="100.0" sigma="0.0" length="%i" minGap="%f" maxSpeed="%f" tau="%f" carFollowModel="CACC" color="255,255,255"/>' % (5, cfg.HEADWAY, cfg.MAX_SPEED, cfg.TIME_STEP);
        print(vType_str, file=routes)

        route_str = "\n"
        for src_idx in range(1,5):
            for dst_idx in range(1,5):
                if src_idx != dst_idx:
                    route_str += "\t<route id=\"route"
                    route_str += str(src_idx)+'_'+str(dst_idx)
                    route_str += "\" edges=\""
                    route_str += " " + str(src_idx)
                    pre_idx = src_idx
                    next_idx = (src_idx-1 - 1)%4 +1
                    while (pre_idx != dst_idx):
                        route_str += " " + str(pre_idx)+str(pre_idx)
                        pre_idx = next_idx
                        next_idx = (pre_idx-1 - 1)%4  +1

                    route_str += " -" + str(dst_idx)
                    route_str += "\"/>\n"


        print(route_str, file=routes)


        dir_r = 0
        if turning_1 == "L":
            dir_r = 1
        elif turning_1 == "S":
            dir_r = 2
        elif turning_1 == "R":
            dir_r = 3

        veh_str1 = "\t<vehicle id=\""
        veh_str1 += turning_1
        veh_str1 += '_0" type="car" route="route%s" depart="%f" departLane = "%i" departSpeed="%f"/>' % (str(4-dir_1)+'_'+str((4-dir_1-1+dir_r)%4+1), t1, lane_1, cfg.MAX_SPEED);

        print(veh_str1, file=routes)

        print("</routes>", file=routes)

    return

def generate_routefile(time_gap, sub_lane_1, turning_1, sub_lane_2, turning_2, dir_2):
    t1 = 0
    t2 = time_gap

    if time_gap < 0:
        t1-=time_gap
        t2-=time_gap

    with open("data/icacc+.rou.xml", "w") as routes:
        print("<routes>\n", file=routes)

        assert sub_lane_1 < cfg.LANE_NUM_PER_DIRECTION, "lane num should be < " + str(cfg.LANE_NUM_PER_DIRECTION)
        assert sub_lane_2 < cfg.LANE_NUM_PER_DIRECTION, "lane num should be < " + str(cfg.LANE_NUM_PER_DIRECTION)

        lane_1 = cfg.LANE_NUM_PER_DIRECTION - sub_lane_1 -1
        lane_2 = cfg.LANE_NUM_PER_DIRECTION - sub_lane_2 -1

        assert dir_2 < 4, "dict >= 4?!"
        '''
        dir_2 start from zero, clockwise!
        '''

        vType_str = '\t<vType id="car" accel="100.0" decel="100.0" sigma="0.0" length="%i" minGap="%f" maxSpeed="%f" tau="%f" carFollowModel="CACC" color="255,255,255"/>' % (5, cfg.HEADWAY, cfg.MAX_SPEED, cfg.TIME_STEP);
        print(vType_str, file=routes)

        route_str = "\n"
        for src_idx in range(1,5):
            for dst_idx in range(1,5):
                if src_idx != dst_idx:
                    route_str += "\t<route id=\"route"
                    route_str += str(src_idx)+'_'+str(dst_idx)
                    route_str += "\" edges=\""
                    route_str += " " + str(src_idx)
                    pre_idx = src_idx
                    next_idx = (src_idx-1 - 1)%4 +1
                    while (pre_idx != dst_idx):
                        route_str += " " + str(pre_idx)+str(pre_idx)
                        pre_idx = next_idx
                        next_idx = (pre_idx-1 - 1)%4  +1

                    route_str += " -" + str(dst_idx)
                    route_str += "\"/>\n"


        print(route_str, file=routes)


        dir_r = 0
        if turning_1 == "L":
            dir_r = 1
        elif turning_1 == "S":
            dir_r = 2
        elif turning_1 == "R":
            dir_r = 3

        veh_str1 = "\t<vehicle id=\""
        veh_str1 += turning_1
        veh_str1 += '_0" type="car" route="route%s" depart="%f" departLane = "%i" departSpeed="%f"/>' % ('4_'+str((3+dir_r)%4+1), t1, lane_1, cfg.MAX_SPEED);


        dir_r = 0
        if turning_2 == "L":
            dir_r = 1
        elif turning_2 == "S":
            dir_r = 2
        elif turning_2 == "R":
            dir_r = 3

        veh_str2 = "\t<vehicle id=\""
        veh_str2 += turning_2
        veh_str2 += '_1" type="car" route="route%s" depart="%f" departLane = "%i" departSpeed="%f"/>' % (str(4-dir_2)+'_'+str((4-dir_2-1+dir_r)%4+1), t2, lane_2, cfg.MAX_SPEED);

        if time_gap >= 0:
            print(veh_str1, file=routes)
            print(veh_str2, file=routes)
        else:
            print(veh_str2, file=routes)
            print(veh_str1, file=routes)


        print("</routes>", file=routes)

    return
