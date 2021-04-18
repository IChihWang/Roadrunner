import json
import copy

import config as cfg


#def main():
    #ans = getconflictregion(1, "s", 2, "r")
    #print (ans[0][1])


class Data:
    def __init__(self):
        with open('inter_length_info/sumo_lane'+str(cfg.LANE_NUM_PER_DIRECTION)+'.json', 'r') as file:
            self.time_dict = json.load(file)



    def getIntertime(self, in_lane, in_turn):

        test_str = str(in_lane) + in_turn
        time = self.time_dict[test_str]


        return time
