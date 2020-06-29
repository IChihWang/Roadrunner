import math
import json
import sys
from ortools.linear_solver import pywraplp

sys.path.append('..')
import config as cfg


STRAIGHT_SPEED = cfg.MAX_SPEED
TURN_SPEED = cfg.TURN_SPEED
LANE_WIDTH = cfg.LANE_WIDTH

def main():
    if len(sys.argv) < 3:
        print("Usage: "+sys.argv[0]+" <input> <output>")
        exit()

    file_name = sys.argv[1]
    file = open(file_name, 'r')
    file_all = file.read()
    file_lines = file_all.splitlines()

    d = dict()
    length_dict = dict()
    for line in file_lines:
        data = line.split()
        data_key = data[0]
        data_Xm = float(data[1])
        data_Ym = float(data[2])
        data_Xd = float(data[3])
        data_Yd = float(data[4])
        data_Px = float(data[5])
        data_Py = float(data[6])
        d[data_key] = ((data_Xm,data_Ym), (data_Xd,data_Yd), (data_Px,data_Py))

        end_str_idx = len(data_key)
        for str_idx in range(len(data_key)):
            if data_key[str_idx].isalpha():
                end_str_idx = str_idx+1
                break

        length_key = data_key[0:end_str_idx]
        length_value = data_Px
        if length_key in length_dict:
            assert (length_dict[length_key] == length_value), "Not matched"
            pass

        length_dict[length_key] = length_value


    genConflictRegionData(d)
    genLengthData(length_dict)


def genConflictRegionData(d):

    # Conflict region dictionary
    # Value format: ((Xm, Ym), (Xd, Yd), (Pi, Pj))

    # Example: "1L2R" - 1L is x axis and 2R is y axis.

    tau = dict()

    for key, value in d.items():

        Xm = value[0][0]*LANE_WIDTH
        Ym = value[0][1]*LANE_WIDTH
        Xd = value[1][0]*LANE_WIDTH
        Yd = value[1][1]*LANE_WIDTH


        tau[key]={'Xm':Xm, 'Ym':Ym, 'Xd':Xd, 'Yd':Yd}


    with open("../inter_info/"+sys.argv[2]+".json", 'w') as file:
        file.write(json.dumps(tau))

def genLengthData(length_dict):

    for key, value in length_dict.items():
        length_dict[key] = value*LANE_WIDTH
    with open("../inter_length_info/"+sys.argv[2]+".json", 'w') as file:
        file.write(json.dumps(length_dict))


if __name__ == '__main__':
    main()
