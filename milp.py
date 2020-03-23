
from get_inter_info import Data
from ortools.linear_solver import pywraplp
import config as cfg
import copy


data = Data()

def Icacc(old_cars, new_cars):
    # part 1: calculate OT
    for c_idx in range(len(new_cars)):
        OT = new_cars[c_idx].position/cfg.MAX_SPEED
        new_cars[c_idx].OT = OT + cfg.SUMO_TIME_ERR

    # part 2: build the solver
    solver = pywraplp.Solver('SolveIntegerProblem',pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)

    # part 3: claim parameters
    for c_idx in range(len(new_cars)):
        if new_cars[c_idx].turning == 'S':
            new_cars[c_idx].D = solver.NumVar(0, solver.infinity(), 'd'+str(c_idx))
        else:
            min_d = (2*cfg.CCZ_DEC2_LEN/(cfg.MAX_SPEED+cfg.TURN_SPEED)) - (cfg.CCZ_DEC2_LEN/cfg.MAX_SPEED)
            new_cars[c_idx].D = solver.NumVar(min_d, solver.infinity(), 'd'+str(c_idx))


    # part 4: set constrain (10)
    all_cars = old_cars+new_cars
    for c_idx in range(len(all_cars)):
        for c_jdx in range(c_idx+1, len(all_cars)):
            if (all_cars[c_idx].lane == all_cars[c_jdx].lane):


                if (type(all_cars[c_jdx].D)==float or type(all_cars[c_idx].D)==float):
                    if (type(all_cars[c_idx].D)!=float and type(all_cars[c_jdx].D)==float):
                        bound = all_cars[c_jdx].length/all_cars[c_jdx].speed_in_intersection + (all_cars[c_jdx].OT+all_cars[c_jdx].D)
                        bound += cfg.HEADWAY/all_cars[c_jdx].speed_in_intersection
                        if all_cars[c_idx].turning == 'S' and all_cars[c_jdx].turning != 'S':
                            bound += (cfg.MAX_SPEED-cfg.TURN_SPEED)*(cfg.CCZ_DEC2_LEN)/(cfg.MAX_SPEED*(cfg.MAX_SPEED+cfg.TURN_SPEED))
                        bound = bound - all_cars[c_idx].OT
                        tmp_conts = solver.Constraint(bound,solver.infinity())
                        tmp_conts.SetCoefficient(all_cars[c_idx].D, 1)
                        #print("eq1-1 ", bound, all_cars[c_idx]['ID'], all_cars[c_jdx]['ID'])
                    elif (type(all_cars[c_idx].D)==float and type(all_cars[c_jdx].D)!=float):
                        bound = all_cars[c_idx].length/all_cars[c_idx].speed_in_intersection + (all_cars[c_idx].OT+all_cars[c_idx].D)
                        bound += cfg.HEADWAY/all_cars[c_idx].speed_in_intersection
                        if all_cars[c_jdx].turning == 'S' and all_cars[c_idx].turning != 'S':
                            bound += (cfg.MAX_SPEED-cfg.TURN_SPEED)*(cfg.CCZ_DEC2_LEN)/(cfg.MAX_SPEED*(cfg.MAX_SPEED+cfg.TURN_SPEED))
                        bound = bound - all_cars[c_jdx].OT
                        tmp_conts = solver.Constraint(bound, solver.infinity())
                        tmp_conts.SetCoefficient(all_cars[c_jdx].D, 1)
                        #print("eq1-2 ", bound, all_cars[c_idx]['ID'], all_cars[c_jdx]['ID'])

                elif (type(all_cars[c_jdx].D)!=float or type(all_cars[c_idx].D)!=float):
                    if (all_cars[c_idx].OT > all_cars[c_jdx].OT):


                        bound = all_cars[c_jdx].length/all_cars[c_jdx].speed_in_intersection - all_cars[c_idx].OT+all_cars[c_jdx].OT
                        bound += cfg.HEADWAY/all_cars[c_jdx].speed_in_intersection
                        if all_cars[c_idx].turning == 'S' and all_cars[c_jdx].turning != 'S':
                            bound += (cfg.MAX_SPEED-cfg.TURN_SPEED)*(cfg.CCZ_DEC2_LEN)/(cfg.MAX_SPEED*(cfg.MAX_SPEED+cfg.TURN_SPEED))
                        tmp_conts = solver.Constraint(bound, solver.infinity())
                        tmp_conts.SetCoefficient(all_cars[c_idx].D, 1)
                        tmp_conts.SetCoefficient(all_cars[c_jdx].D, -1)
                        #print("eq1-3 ", bound, all_cars[c_idx]['ID'], all_cars[c_jdx]['ID'])
                    elif (all_cars[c_idx].OT < all_cars[c_jdx].OT):

                        bound = all_cars[c_idx].length/all_cars[c_idx].speed_in_intersection + all_cars[c_idx].OT-all_cars[c_jdx].OT
                        bound += cfg.HEADWAY/all_cars[c_idx].speed_in_intersection
                        if all_cars[c_jdx].turning == 'S' and all_cars[c_idx].turning != 'S':
                            bound += (cfg.MAX_SPEED-cfg.TURN_SPEED)*(cfg.CCZ_DEC2_LEN)/(cfg.MAX_SPEED*(cfg.MAX_SPEED+cfg.TURN_SPEED))
                        tmp_conts = solver.Constraint(bound, solver.infinity())
                        tmp_conts.SetCoefficient(all_cars[c_idx].D, -1)
                        tmp_conts.SetCoefficient(all_cars[c_jdx].D, 1)
                        #print("eq1-4 ", bound, all_cars[c_idx]['ID'], all_cars[c_jdx]['ID'])



    # part 5: set constrain (11)
    for c_idx in range(len(new_cars)):
        for c_jdx in range(c_idx+1, len(new_cars)):

            if (new_cars[c_idx].lane == new_cars[c_jdx].lane):
                continue


            #ans = data.getConflictRegion(new_cars[c_idx].lane, new_cars[c_idx].turning, new_cars[c_jdx].lane, new_cars[c_jdx].turning)
            ans = data.getConflictRegion(new_cars[c_idx], new_cars[c_jdx])


            if (len(ans) > 0):
                tau_S1_S2 = ans[0]
                tau_S2_S1 = ans[1]


                flag = solver.IntVar(0, 1, 'flag'+str(c_idx)+"_"+str(c_jdx))

                bound = -new_cars[c_jdx].OT + new_cars[c_idx].OT + tau_S1_S2 - cfg.LARGE_NUM
                tmp_conts2 = solver.Constraint(bound, solver.infinity())
                tmp_conts2.SetCoefficient(new_cars[c_idx].D, -1)
                tmp_conts2.SetCoefficient(new_cars[c_jdx].D, 1)
                tmp_conts2.SetCoefficient(flag, -cfg.LARGE_NUM)
                #print("eq2-1 ", bound, new_cars[c_idx]['ID'], new_cars[c_jdx]['ID'])

                bound = -new_cars[c_idx].OT + new_cars[c_jdx].OT + tau_S2_S1
                tmp_conts1 = solver.Constraint(bound, solver.infinity())
                tmp_conts1.SetCoefficient(new_cars[c_idx].D, 1)
                tmp_conts1.SetCoefficient(new_cars[c_jdx].D, -1)
                tmp_conts1.SetCoefficient(flag, cfg.LARGE_NUM)
                #print("eq2-2 ", bound, new_cars[c_idx]['ID'], new_cars[c_jdx]['ID'])


    # part 6: set constrain (12)
    for nc_idx in range(len(new_cars)):
        for oc_idx in range(len(old_cars)):
            if (new_cars[nc_idx].lane == old_cars[oc_idx].lane):
                continue


            #ans = data.getConflictRegion(new_cars[nc_idx].lane, new_cars[nc_idx].turning, old_cars[oc_idx].lane, old_cars[oc_idx].turning)

            ans = data.getConflictRegion(new_cars[nc_idx], old_cars[oc_idx])


            if (len(ans) > 0):
                tau_S1_S2 = ans[0]
                tau_S2_S1 = ans[1]

                bound = old_cars[oc_idx].D + old_cars[oc_idx].OT - new_cars[nc_idx].OT + tau_S2_S1
                tmp_conts4 = solver.Constraint(bound, solver.infinity())
                tmp_conts4.SetCoefficient(new_cars[nc_idx].D, 1)


                #print("eq3-1 ", bound, old_cars[oc_idx]['ID'], new_cars[nc_idx]['ID'])

    # part 7: set objective
    objective = solver.Objective()
    for c_idx in range(len(new_cars)):
        objective.SetCoefficient(new_cars[c_idx].D, 1)
    objective.SetMinimization()


    # part 8: Solve the problem
    sol_status = solver.Solve()
    if (sol_status == 2):
        # Unfeasible
        #print ([car.position for car in old_cars])
        #print ([car.position for car in new_cars])
        print("Error: no fesible solution")
        exit()


    for nc_idx in range(len(new_cars)):
        new_cars[nc_idx].D = new_cars[nc_idx].D.solution_value()

    #print('Solution:')
    avg_delay = 0
    for nc_idx in range(len(new_cars)):
        #print(new_cars[nc_idx]['ID'], " = ", new_cars[nc_idx].D.solution_value())
        avg_delay = avg_delay + new_cars[nc_idx].D
    if len(new_cars) > 0:
        avg_delay = avg_delay/len(new_cars)

    return avg_delay

def IcaccPlus(old_cars, new_cars):
    # part 1: calculate OT
    for c_idx in range(len(new_cars)):
        OT = new_cars[c_idx].position/cfg.MAX_SPEED
        new_cars[c_idx].OT = OT + cfg.SUMO_TIME_ERR

    # part 2: build the solver
    solver = pywraplp.Solver('SolveIntegerProblem',pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)

    # part 3: claim parameters
    for c_idx in range(len(new_cars)):
        if new_cars[c_idx].turning == 'S':
            new_cars[c_idx].D = solver.NumVar(0, solver.infinity(), 'd'+str(c_idx))
        else:
            min_d = (2*cfg.CCZ_DEC2_LEN/(cfg.MAX_SPEED+cfg.TURN_SPEED)) - (cfg.CCZ_DEC2_LEN/cfg.MAX_SPEED)
            new_cars[c_idx].D = solver.NumVar(min_d, solver.infinity(), 'd'+str(c_idx))


    # part 4: set constrain (10)
    all_cars = old_cars+new_cars
    for c_idx in range(len(all_cars)):
        for c_jdx in range(c_idx+1, len(all_cars)):
            if (all_cars[c_idx].lane == all_cars[c_jdx].lane):


                if (type(all_cars[c_jdx].D)==float or type(all_cars[c_idx].D)==float):
                    if (type(all_cars[c_idx].D)!=float and type(all_cars[c_jdx].D)==float):
                        bound = all_cars[c_jdx].length/all_cars[c_jdx].speed_in_intersection + (all_cars[c_jdx].OT+all_cars[c_jdx].D)
                        bound += cfg.HEADWAY/all_cars[c_jdx].speed_in_intersection
                        if all_cars[c_idx].turning == 'S' and all_cars[c_jdx].turning != 'S':
                            bound += (cfg.MAX_SPEED-cfg.TURN_SPEED)*(cfg.CCZ_DEC2_LEN)/(cfg.MAX_SPEED*(cfg.MAX_SPEED+cfg.TURN_SPEED))
                        bound = bound - all_cars[c_idx].OT
                        tmp_conts = solver.Constraint(bound,solver.infinity())
                        tmp_conts.SetCoefficient(all_cars[c_idx].D, 1)
                        #print("eq1-1 ", bound, all_cars[c_idx]['ID'], all_cars[c_jdx]['ID'])
                    elif (type(all_cars[c_idx].D)==float and type(all_cars[c_jdx].D)!=float):
                        bound = all_cars[c_idx].length/all_cars[c_idx].speed_in_intersection + (all_cars[c_idx].OT+all_cars[c_idx].D)
                        bound += cfg.HEADWAY/all_cars[c_idx].speed_in_intersection
                        if all_cars[c_jdx].turning == 'S' and all_cars[c_idx].turning != 'S':
                            bound += (cfg.MAX_SPEED-cfg.TURN_SPEED)*(cfg.CCZ_DEC2_LEN)/(cfg.MAX_SPEED*(cfg.MAX_SPEED+cfg.TURN_SPEED))
                        bound = bound - all_cars[c_jdx].OT
                        tmp_conts = solver.Constraint(bound, solver.infinity())
                        tmp_conts.SetCoefficient(all_cars[c_jdx].D, 1)
                        #print("eq1-2 ", bound, all_cars[c_idx]['ID'], all_cars[c_jdx]['ID'])

                elif (type(all_cars[c_jdx].D)!=float or type(all_cars[c_idx].D)!=float):
                    if (all_cars[c_idx].OT > all_cars[c_jdx].OT):


                        bound = all_cars[c_jdx].length/all_cars[c_jdx].speed_in_intersection - all_cars[c_idx].OT+all_cars[c_jdx].OT
                        bound += cfg.HEADWAY/all_cars[c_jdx].speed_in_intersection
                        if all_cars[c_idx].turning == 'S' and all_cars[c_jdx].turning != 'S':
                            bound += (cfg.MAX_SPEED-cfg.TURN_SPEED)*(cfg.CCZ_DEC2_LEN)/(cfg.MAX_SPEED*(cfg.MAX_SPEED+cfg.TURN_SPEED))
                        tmp_conts = solver.Constraint(bound, solver.infinity())
                        tmp_conts.SetCoefficient(all_cars[c_idx].D, 1)
                        tmp_conts.SetCoefficient(all_cars[c_jdx].D, -1)
                        #print("eq1-3 ", bound, all_cars[c_idx]['ID'], all_cars[c_jdx]['ID'])
                    elif (all_cars[c_idx].OT < all_cars[c_jdx].OT):

                        bound = all_cars[c_idx].length/all_cars[c_idx].speed_in_intersection + all_cars[c_idx].OT-all_cars[c_jdx].OT
                        bound += cfg.HEADWAY/all_cars[c_idx].speed_in_intersection
                        if all_cars[c_jdx].turning == 'S' and all_cars[c_idx].turning != 'S':
                            bound += (cfg.MAX_SPEED-cfg.TURN_SPEED)*(cfg.CCZ_DEC2_LEN)/(cfg.MAX_SPEED*(cfg.MAX_SPEED+cfg.TURN_SPEED))
                        tmp_conts = solver.Constraint(bound, solver.infinity())
                        tmp_conts.SetCoefficient(all_cars[c_idx].D, -1)
                        tmp_conts.SetCoefficient(all_cars[c_jdx].D, 1)
                        #print("eq1-4 ", bound, all_cars[c_idx]['ID'], all_cars[c_jdx]['ID'])



    # part 5: set constrain (11)
    for c_idx in range(len(new_cars)):
        for c_jdx in range(c_idx+1, len(new_cars)):

            if (new_cars[c_idx].lane == new_cars[c_jdx].lane):
                continue


            #ans = data.getConflictRegion(new_cars[c_idx].lane, new_cars[c_idx].turning, new_cars[c_jdx].lane, new_cars[c_jdx].turning)
            ans = data.getConflictRegion(new_cars[c_idx], new_cars[c_jdx])


            if (len(ans) > 0):
                tau_S1_S2 = ans[0]
                tau_S2_S1 = ans[1]





                flag = solver.IntVar(0, 1, 'flag'+str(c_idx)+'_'+str(c_jdx))

                bound = -new_cars[c_jdx].OT + new_cars[c_idx].OT + tau_S1_S2 - cfg.LARGE_NUM
                tmp_conts2 = solver.Constraint(bound, solver.infinity())
                tmp_conts2.SetCoefficient(new_cars[c_idx].D, -1)
                tmp_conts2.SetCoefficient(new_cars[c_jdx].D, 1)
                tmp_conts2.SetCoefficient(flag, -cfg.LARGE_NUM)
                #print("eq2-1 ", bound+ cfg.LARGE_NUM, new_cars[c_idx]['ID'], new_cars[c_jdx]['ID'], c_idx, c_jdx)

                bound = -new_cars[c_idx].OT + new_cars[c_jdx].OT + tau_S2_S1
                tmp_conts1 = solver.Constraint(bound, solver.infinity())
                tmp_conts1.SetCoefficient(new_cars[c_idx].D, 1)
                tmp_conts1.SetCoefficient(new_cars[c_jdx].D, -1)
                tmp_conts1.SetCoefficient(flag, cfg.LARGE_NUM)
                #print("eq2-2 ", bound, new_cars[c_idx]['ID'], new_cars[c_jdx]['ID'])

    #'''
    # part 6: set constrain (12)
    for nc_idx in range(len(new_cars)):
        for oc_idx in range(len(old_cars)):

            if (new_cars[nc_idx].lane == old_cars[oc_idx].lane):
                continue



            #ans = data.getConflictRegion(new_cars[nc_idx].lane, new_cars[nc_idx].turning, old_cars[oc_idx].lane, old_cars[oc_idx].turning)

            ans = data.getConflictRegion(new_cars[nc_idx], old_cars[oc_idx])





            if (len(ans) > 0):
                tau_S1_S2 = ans[0]
                tau_S2_S1 = ans[1]

                '''
                if (old_cars[oc_idx]['ID'] == 'L_27') and (new_cars[nc_idx]['ID'] == 'L_32'):
                    print(ans)
                    print(new_cars[nc_idx]['ID'], new_cars[nc_idx].lane, new_cars[nc_idx].turning, new_cars[nc_idx].OT)
                    print(old_cars[oc_idx]['ID'], old_cars[oc_idx].lane, old_cars[oc_idx].turning, old_cars[oc_idx].OT)
                '''

                flag = solver.IntVar(0, 1, 'flagg'+str(nc_idx)+"_"+str(oc_idx))

                bound = -old_cars[oc_idx].D-old_cars[oc_idx].OT + new_cars[nc_idx].OT + tau_S1_S2 - cfg.LARGE_NUM
                tmp_conts3 = solver.Constraint(bound, solver.infinity())
                tmp_conts3.SetCoefficient(new_cars[nc_idx].D, -1)
                tmp_conts3.SetCoefficient(flag, -cfg.LARGE_NUM)

                bound = old_cars[oc_idx].D + old_cars[oc_idx].OT - new_cars[nc_idx].OT + tau_S2_S1
                tmp_conts4 = solver.Constraint(bound, solver.infinity())
                tmp_conts4.SetCoefficient(new_cars[nc_idx].D, 1)
                tmp_conts4.SetCoefficient(flag, cfg.LARGE_NUM)
                #print("eq3-2 ", bound, new_cars[nc_idx]['ID'], old_cars[oc_idx]['ID'])
    #'''


    # part 7: set objective
    objective = solver.Objective()

    for c_idx in range(len(new_cars)):
        objective.SetCoefficient(new_cars[c_idx].D, 1)
    objective.SetMinimization()

    '''
    print('Car num', len(new_cars))
    print('Number of variables =', solver.NumVariables())
    print('Number of constraints =', solver.NumConstraints())
    #'''

    # part 8: Solve the problem
    sol_status = solver.Solve()


    if (sol_status == 2):
        # Unfeasible
        #print ([car.position for car in old_cars])
        #print ([car.position for car in new_cars])
        print("Error: no fesible solution")
        exit()

    for nc_idx in range(len(new_cars)):
        new_cars[nc_idx].D = new_cars[nc_idx].D.solution_value()

    #print('Solution:')
    avg_delay = 0
    for nc_idx in range(len(new_cars)):
        #print(new_cars[nc_idx]['ID'], " = ", new_cars[nc_idx].D.solution_value())
        avg_delay = avg_delay + new_cars[nc_idx].D
    if len(new_cars) > 0:
        avg_delay = avg_delay/len(new_cars)

    #print ("avg_delay ", ans/len(new_cars))


    #print("=============")


    '''
    for c_idx in range(len(all_cars)):
        for c_jdx in range(len(all_cars)):
            if (all_cars[c_idx]['ID'] == 'L_12') and (all_cars[c_jdx]['ID'] == 'R_13'):
                print(all_cars[c_jdx])
                print(all_cars[c_idx])
    '''


    return avg_delay


def Fcfs(old_cars, new_cars):
    sorted_new = sorted(new_cars, key=lambda k: k.position)

    for car in sorted_new:
        IcaccPlus(old_cars, [car])
        old_cars.append(car)



def FixedSignal(old_cars, new_cars, iteration):
    for car in old_cars:
        car.D = car.D+cfg.TIME_STEP

    for car in new_cars:
        car.OT = 999999999
        car.D = 0
        old_cars.append(car)
    del new_cars[:]

    dir = (iteration%28)//7
    for idx in range(cfg.LANE_NUM_PER_DIRECTION):
        lane_i = dir*cfg.LANE_NUM_PER_DIRECTION+idx

        car_list = [car for car in old_cars if car.lane == lane_i]

        car_pass_length = cfg.TIME_STEP*cfg.MAX_SPEED
        for car in car_list:
            car_pass_length = car_pass_length - car.length
            if car_pass_length <= 0:
                #Left over, not pass
                car.length = car.length+car_pass_length
                break
            else:
                car.OT = -car.D
