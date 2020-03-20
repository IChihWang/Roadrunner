import math
import json
import sys

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
    for line in file_lines:
        data = line.split()
        data_key = data[0]

        d[data_key] = []
        for cord_idx in range(1, len(data),3):
            cord_list = dict()
            cord_list['X'] = int(data[cord_idx])
            cord_list['Y'] = int(data[cord_idx+1])
            cord_list['distance'] = float(data[cord_idx+2])

            d[data_key].append(cord_list)

    with open("../advise_info/"+sys.argv[2]+".json", 'w') as file:
        file.write(json.dumps(d))


if __name__ == '__main__':
    main()
