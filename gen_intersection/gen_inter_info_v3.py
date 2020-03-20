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

    genConflictRegionData(d)

'''
def genConflictRegionData(d):
    pi = math.pi

    # Conflict region dictionary
    # Value format: ((Xm, Ym), (Xd, Yd), (Pi, Pj))

    # Example: "1L2R" - 1L is x axis and 2R is y axis.

    tau = dict()

    for key, value in d.items():
        car_1_lane = key[2]
        car_2_lane = key[0]
        car_1_dir = key[3]
        car_2_dir = key[1]

        car_1_speed = STRAIGHT_SPEED;
        car_2_speed = STRAIGHT_SPEED;

        if key == '0S7L':
            print(value[0][0]*LANE_WIDTH, value[0][1]*LANE_WIDTH, value[1][0]*LANE_WIDTH, value[1][1]*LANE_WIDTH, value[2][0]*LANE_WIDTH, value[2][1]*LANE_WIDTH)

        if car_1_dir != 'S':
            car_1_speed = TURN_SPEED
        if car_2_dir != 'S':
            car_2_speed = TURN_SPEED

        sub_solver = pywraplp.Solver('SolveSubProblem',pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)
        S1 = sub_solver.NumVar(0, value[2][0]*LANE_WIDTH, 's1')
        S2 = sub_solver.NumVar(0, value[2][1]*LANE_WIDTH, 's2')

        tmp_conts1 = sub_solver.Constraint(value[0][0]*LANE_WIDTH, value[1][0]*LANE_WIDTH)
        tmp_conts1.SetCoefficient(S1,1)
        tmp_conts2 = sub_solver.Constraint(value[0][1]*LANE_WIDTH, value[1][1]*LANE_WIDTH)
        tmp_conts2.SetCoefficient(S2,1)


        # (Ym-Yd)Xm-(Xm-Xd)Ym
        bound = (value[0][1]*LANE_WIDTH-value[1][1]*LANE_WIDTH)*value[0][0]*LANE_WIDTH
        bound += -(value[0][0]*LANE_WIDTH-value[1][0]*LANE_WIDTH)*value[0][1]*LANE_WIDTH
        tmp_conts3 = sub_solver.Constraint(bound, bound)
        # (Ym-Yd)S1-(Xm-Xd)S2
        tmp_conts3.SetCoefficient(S1,(value[0][1]*LANE_WIDTH-value[1][1]*LANE_WIDTH))
        tmp_conts3.SetCoefficient(S2,-(value[0][0]*LANE_WIDTH-value[1][0]*LANE_WIDTH))


        objective = sub_solver.Objective()
        objective.SetCoefficient(S1, 1/car_1_speed)
        objective.SetCoefficient(S2, -1/car_2_speed)

        objective.SetMaximization()
        status = sub_solver.Solve()
        if status == sub_solver.OPTIMAL:
            tau_S1_S2 = S1.solution_value()/car_1_speed-S2.solution_value()/car_2_speed
        else:
            print ("Fail to find")
            print (key)


        objective.SetMinimization()
        status = sub_solver.Solve()
        if status == sub_solver.OPTIMAL:
            tau_S2_S1 = -( S1.solution_value()/car_1_speed-S2.solution_value()/car_2_speed )
        else:
            print ("Fail to find")
            print (key)

        tau[key]=(tau_S1_S2, tau_S2_S1)


    with open("../inter_info/"+sys.argv[2]+".json", 'w') as file:
        file.write(json.dumps(tau))
'''

def genConflictRegionData(d):
    pi = math.pi

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


if __name__ == '__main__':
    main()
