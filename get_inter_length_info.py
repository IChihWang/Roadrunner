import json
import copy

import config as cfg


#def main():
    #ans = getconflictregion(1, "s", 2, "r")
    #print (ans[0][1])


class Data:
    def __init__(self):
        with open('inter_length_info/lane_info'+str(cfg.LANE_NUM_PER_DIRECTION)+'.json', 'r') as file:
            self.length_dict = json.load(file)



    def getIntertime(self, in_lane, in_turn):

        test_str = str(in_lane% cfg.LANE_NUM_PER_DIRECTION) + in_turn
        length = self.length_dict[test_str]
        
        time = None
        if in_turn == "S":
            time = length/cfg.MAX_SPEED
        else:
            time = length/cfg.TURN_SPEED

        return time

