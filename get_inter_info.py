import json
import copy

import config as cfg


#def main():
    #ans = getconflictregion(1, "s", 2, "r")
    #print (ans[0][1])


class Data:
    def __init__(self):
        with open('inter_info/sumo_lane_info'+str(cfg.LANE_NUM_PER_DIRECTION)+'.json', 'r') as file:
            self.tau = json.load(file)



    def getConflictRegion(self, car1, car2):
        reverse_flag = False


        if (car1.lane > car2.lane):
            car1, car2 = car2, car1
            reverse_flag = True

        l1 = car1.lane
        l2 = car2.lane

        # rotate to corresponding cases
        if l1 >= cfg.LANE_NUM_PER_DIRECTION:
            l2 = (l2 - (l1//cfg.LANE_NUM_PER_DIRECTION)*cfg.LANE_NUM_PER_DIRECTION) % (cfg.LANE_NUM_PER_DIRECTION*4)
            l1 = (l1 - (l1//cfg.LANE_NUM_PER_DIRECTION)*cfg.LANE_NUM_PER_DIRECTION)


        # Test if there is conflict between l1 & l2
        # and give the conflict region.

        test_str = str(l1) + car1.turning + str(l2) + car2.turning

        if (test_str) in self.tau:
            val = self.tau[test_str].copy()


            # 1. Compute tau_S1_S2
            tau_S1_S2 = None
            tau_S1_S2 = val['distance_diff_max_12']/cfg.MAX_SPEED + (car1.length+cfg.HEADWAY)/cfg.MAX_SPEED


            # 2. Compute tau_S2_S1
            tau_S2_S1 = None
            tau_S2_S1 = val['distance_diff_max_21']/cfg.MAX_SPEED + (car2.length+cfg.HEADWAY)/cfg.MAX_SPEED

            ans = [tau_S1_S2, tau_S2_S1]



            if (reverse_flag):
                ans[0], ans[1] = ans[1], ans[0]

            # tau_S1_S2: car1 is in front of car2
            return ans
        else:
            return []
