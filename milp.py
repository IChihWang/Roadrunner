
from get_inter_info import Data
from ortools.linear_solver import pywraplp
import config as cfg
import copy
import get_inter_length_info
import numpy

inter_length_data = get_inter_length_info.Data()

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

def IcaccPlus(old_cars, new_cars, advised_n_sched_car, pedestrian_time_mark_list, others_road_info, spillback_delay_record):
    # part 1: calculate OT
    for c_idx in range(len(new_cars)):
        OT = new_cars[c_idx].position/cfg.MAX_SPEED
        new_cars[c_idx].OT = OT + cfg.SUMO_TIME_ERR

    car_idd = [car.ID for car in new_cars]
    if "RL_556" in car_idd or "RS_568" in car_idd or "RR_579" in car_idd or "RR_601" in car_idd:
        print(car_idd)


    # part 2: build the solver
    solver = pywraplp.Solver('SolveIntegerProblem',pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)

    # part 3: claim parameters

    car_count_src_dst_lane = [ [0]*len(others_road_info) for i in range(len(others_road_info))]
    for car in advised_n_sched_car:
        lane_idx = car.lane
        dst_lane_idx = car.dst_lane
        for l_idx in range(len(others_road_info)):
            if l_idx != dst_lane_idx:
                car_count_src_dst_lane[lane_idx][l_idx] += 1

    # Sort new cars
    new_car_src_dst_lane = [[[] for i in range(len(others_road_info))] for i in range(len(others_road_info))]
    for car in new_cars:
        lane_idx = car.lane
        dst_lane_idx = car.dst_lane
        new_car_src_dst_lane[lane_idx][dst_lane_idx].append(car)


    accumulate_car_len_lane = [0]*(len(others_road_info))
    last_car_delay_lane = [0]*(len(others_road_info))
    for car in old_cars:
        lane_idx = car.dst_lane
        accumulate_car_len_lane[lane_idx] += (car.length + cfg.HEADWAY)

        current_delay = (car.OT+car.D) - car.position/cfg.MAX_SPEED
        if current_delay > last_car_delay_lane[lane_idx]:
            last_car_delay_lane[lane_idx] = current_delay


    sorted_new_cars = sorted(new_cars, key=lambda x: x.position, reverse=False)
    head_of_line_blocking_position = [999999]*cfg.LANE_NUM_PER_DIRECTION*4
    accumulate_car_len = [-999999]*len(others_road_info)
    recorded_delay = [0]*len(others_road_info)
    base_delay = [0]*len(others_road_info)
    max_dst_lane_idx_list = [[-999999, -1] for i in range(4)]
    for dst_lane_idx in range(len(others_road_info)):
        if others_road_info[dst_lane_idx] != None:
            accumulate_car_len[dst_lane_idx] = accumulate_car_len_lane[dst_lane_idx]-others_road_info[dst_lane_idx]['avail_len']+2*(cfg.CAR_MAX_LEN+cfg.HEADWAY)
            recorded_delay[dst_lane_idx] = max(others_road_info[dst_lane_idx]['delay'], spillback_delay_record[dst_lane_idx]) # To record the dispatch speed
            base_delay[dst_lane_idx] = recorded_delay

            direction = dst_lane_idx//4
            if max_dst_lane_idx_list[direction][0] < accumulate_car_len[dst_lane_idx]:
                max_dst_lane_idx_list[direction][0] = accumulate_car_len[dst_lane_idx]
                max_dst_lane_idx_list[direction][1] = dst_lane_idx

    max_dst_lane_idx_list = [data[1] for data in max_dst_lane_idx_list]

    #max_dst_lane_idx = numpy.argmax(accumulate_car_len)

    for car in sorted_new_cars:

        dst_lane_idx = car.dst_lane
        lane_idx = car.lane
        max_dst_lane_idx = max_dst_lane_idx_list[dst_lane_idx//4]
        if others_road_info[dst_lane_idx] != None:
            if car.position > head_of_line_blocking_position[lane_idx]:
                new_cars.remove(car)    # Blocked by the car at the front
                continue

            accumulate_car_len[max_dst_lane_idx] += (car.length + cfg.HEADWAY)
            spillback_delay = 0

            if accumulate_car_len[max_dst_lane_idx] > 0:
                spillback_delay_multiply_factor = accumulate_car_len[max_dst_lane_idx]/(cfg.CCZ_LEN)
                spillback_delay = recorded_delay[max_dst_lane_idx]*(spillback_delay_multiply_factor)

                '''
                if last_car_delay_lane[max_dst_lane_idx] > spillback_delay:    # To make space with front batch
                    base_delay[max_dst_lane_idx] = last_car_delay_lane[max_dst_lane_idx]
                    last_car_delay_lane[max_dst_lane_idx] = -1       # Ensure that this is only called once
                    accumulate_car_len[max_dst_lane_idx] = (car.length + cfg.HEADWAY)
                    spillback_delay = base_delay[max_dst_lane_idx]
                '''
                #print(car.ID, max_dst_lane_idx,accumulate_car_len[max_dst_lane_idx], dst_lane_idx, accumulate_car_len[dst_lane_idx])
                car.is_spillback = True
                if accumulate_car_len[max_dst_lane_idx] > (cfg.CCZ_LEN):
                    car.is_spillback_strict = True
                else:
                    car.is_spillback_strict = False

                spillback_delay_record[max_dst_lane_idx] = recorded_delay[max_dst_lane_idx]
            else:
                spillback_delay_record[max_dst_lane_idx] = 0

            if car.is_spillback_strict == True:
                new_cars.remove(car)
                if car.position < head_of_line_blocking_position[lane_idx]:
                    head_of_line_blocking_position[lane_idx] = car.position
            else:
                min_d_add = 0
                if car.position < cfg.CCZ_LEN:
                    min_d_add = (2*2*cfg.CCZ_ACC_LEN/(cfg.MAX_SPEED+0)) - (2*cfg.CCZ_ACC_LEN/cfg.MAX_SPEED)

                if car.turning == 'S':
                    car.D = solver.NumVar(max(0+min_d_add, spillback_delay), solver.infinity(), 'd'+str(car.ID))
                else:
                    min_d = (2*cfg.CCZ_DEC2_LEN/(cfg.MAX_SPEED+cfg.TURN_SPEED)) - (cfg.CCZ_DEC2_LEN/cfg.MAX_SPEED)
                    car.D = solver.NumVar(max(min_d+min_d_add, spillback_delay), solver.infinity(), 'd'+str(car.ID))


    '''
    for car in sorted_new_cars:

        dst_lane_idx = car.dst_lane
        lane_idx = car.lane
        if others_road_info[dst_lane_idx] != None:
            if car.position > head_of_line_blocking_position[lane_idx]:
                new_cars.remove(car)    # Blocked by the car at the front
                continue

            accumulate_car_len[dst_lane_idx] += (car.length + cfg.HEADWAY)
            spillback_delay = 0

            if accumulate_car_len[dst_lane_idx] > 0:
                spillback_delay_multiply_factor = accumulate_car_len[dst_lane_idx]/(cfg.CCZ_LEN)
                spillback_delay = recorded_delay[dst_lane_idx]*(1+spillback_delay_multiply_factor)

                if last_car_delay_lane[dst_lane_idx] > spillback_delay:    # To make space with front batch
                    base_delay[dst_lane_idx] = last_car_delay_lane[dst_lane_idx]
                    last_car_delay_lane[dst_lane_idx] = -1       # Ensure that this is only called once
                    accumulate_car_len[dst_lane_idx] = (car.length + cfg.HEADWAY)
                    spillback_delay = base_delay[dst_lane_idx]

                car.is_spillback = True
                if accumulate_car_len[dst_lane_idx] > (cfg.CCZ_LEN):
                    car.is_spillback_strict = True
                else:
                    car.is_spillback_strict = False

                spillback_delay_record[dst_lane_idx] = recorded_delay[dst_lane_idx]
            else:
                spillback_delay_record[dst_lane_idx] = 0

            if car.is_spillback_strict == True:
                new_cars.remove(car)
                if car.position < head_of_line_blocking_position[lane_idx]:
                    head_of_line_blocking_position[lane_idx] = car.position
            else:
                min_d_add = 0
                if car.position < cfg.CCZ_LEN:
                    min_d_add = (2*2*cfg.CCZ_ACC_LEN/(cfg.MAX_SPEED+0)) - (2*cfg.CCZ_ACC_LEN/cfg.MAX_SPEED)

                if car.turning == 'S':
                    car.D = solver.NumVar(max(0+min_d_add, spillback_delay), solver.infinity(), 'd'+str(car.ID))
                else:
                    min_d = (2*cfg.CCZ_DEC2_LEN/(cfg.MAX_SPEED+cfg.TURN_SPEED)) - (cfg.CCZ_DEC2_LEN/cfg.MAX_SPEED)
                    car.D = solver.NumVar(max(min_d+min_d_add, spillback_delay), solver.infinity(), 'd'+str(car.ID))

    '''




    '''
    head_of_line_blocking_position = [cfg.TOTAL_LEN*2]*cfg.LANE_NUM_PER_DIRECTION*4
    for dst_lane_idx in range(len(others_road_info)):
        # Connected to other intersections
        if others_road_info[dst_lane_idx] != None:
            affected_car_count_list = [car_count_src_dst_lane[lane_idx][dst_lane_idx] for lane_idx in range(len(others_road_info))]
            sorted_lane_idx_list = numpy.argsort(affected_car_count_list)

            accumulate_car_len = accumulate_car_len_lane[dst_lane_idx]-others_road_info[dst_lane_idx]['avail_len']+(cfg.CAR_MAX_LEN+cfg.HEADWAY)
            recorded_delay = max(others_road_info[dst_lane_idx]['delay'], spillback_delay_record[dst_lane_idx]) # To record the dispatch speed
            base_delay = recorded_delay

            affected_car_list = []
            for lane_idx in range(len(others_road_info)):
                for car in new_car_src_dst_lane[lane_idx][dst_lane_idx]:
                    affected_car_list.append(car)

            sorted_affected_car_list = sorted(affected_car_list, key=lambda x: x.position, reverse=False)


            for car in sorted_affected_car_list:
                accumulate_car_len += (car.length + cfg.HEADWAY)
                spillback_delay = 0


                if "RL_556" == car.ID or "RS_568" == car.ID or "RR_579" == car.ID or "RR_601" == car.ID:
                    print(car.ID, car.position, dst_lane_idx, accumulate_car_len, head_of_line_blocking_position[lane_idx])


                if accumulate_car_len > 0:
                    spillback_delay_multiply_factor = accumulate_car_len/(cfg.CCZ_LEN+cfg.BZ_LEN)
                    spillback_delay = recorded_delay*spillback_delay_multiply_factor

                    if last_car_delay_lane[dst_lane_idx] > spillback_delay:    # To make space with front batch
                        base_delay = last_car_delay_lane[dst_lane_idx]
                        last_car_delay_lane[dst_lane_idx] = -1       # Ensure that this is only called once
                        accumulate_car_len = (car.length + cfg.HEADWAY)
                        spillback_delay = base_delay

                    car.is_spillback = True
                    if accumulate_car_len > (cfg.CCZ_LEN+cfg.BZ_LEN):
                        car.is_spillback_strict = True
                    else:
                        car.is_spillback_strict = False

                    spillback_delay_record[dst_lane_idx] = recorded_delay
                else:
                    spillback_delay_record[dst_lane_idx] = 0

                if car.is_spillback_strict == True:
                    new_cars.remove(car)
                    if car.position < head_of_line_blocking_position[lane_idx]:
                        head_of_line_blocking_position[lane_idx] = car.position
                else:
                    min_d_add = 0
                    if car.position < cfg.CCZ_LEN:
                        min_d_add = (2*2*cfg.CCZ_ACC_LEN/(cfg.MAX_SPEED+0)) - (2*cfg.CCZ_ACC_LEN/cfg.MAX_SPEED)

                    if car.turning == 'S':
                        car.D = solver.NumVar(max(0+min_d_add, spillback_delay), solver.infinity(), 'd'+str(car.ID))
                    else:
                        min_d = (2*cfg.CCZ_DEC2_LEN/(cfg.MAX_SPEED+cfg.TURN_SPEED)) - (cfg.CCZ_DEC2_LEN/cfg.MAX_SPEED)
                        car.D = solver.NumVar(max(min_d+min_d_add, spillback_delay), solver.infinity(), 'd'+str(car.ID))

    '''

    '''
            for lane_idx in sorted_lane_idx_list:
                for car in new_car_src_dst_lane[lane_idx][dst_lane_idx]:
                    accumulate_car_len += (car.length + cfg.HEADWAY)
                    spillback_delay = 0


                    if "RL_556" == car.ID or "RS_568" == car.ID or "RR_579" == car.ID:
                        print(car.ID, car.position, accumulate_car_len, head_of_line_blocking_position[lane_idx])


                    if accumulate_car_len > 0:

                        #spillback_delay_multiply_factor = accumulate_car_len//(cfg.CCZ_LEN+cfg.BZ_LEN)
                        #spillback_delay = base_delay + recorded_delay*spillback_delay_multiply_factor

                        #spillback_delay_multiply_factor = accumulate_car_len//(cfg.TOTAL_LEN/2)
                        #spillback_delay_multiply_factor = accumulate_car_len/(cfg.TOTAL_LEN/2)
                        spillback_delay_multiply_factor = accumulate_car_len/(cfg.CCZ_LEN+cfg.BZ_LEN)
                        #spillback_delay = base_delay + recorded_delay*spillback_delay_multiply_factor
                        #spillback_delay = base_delay+recorded_delay*spillback_delay_multiply_factor
                        spillback_delay = recorded_delay*spillback_delay_multiply_factor

                        if last_car_delay_lane[dst_lane_idx] > spillback_delay:    # To make space with front batch
                            base_delay = last_car_delay_lane[dst_lane_idx]
                            last_car_delay_lane[dst_lane_idx] = -1       # Ensure that this is only called once
                            accumulate_car_len = (car.length + cfg.HEADWAY)
                            spillback_delay = base_delay
                        car.is_spillback = True
                        if accumulate_car_len > (cfg.CCZ_LEN+cfg.BZ_LEN):
                            car.is_spillback_strict = True
                        else:
                            car.is_spillback_strict = False

                        spillback_delay_record[dst_lane_idx] = recorded_delay
                    else:
                        spillback_delay_record[dst_lane_idx] = 0

                    if car.is_spillback_strict == True:
                        new_cars.remove(car)
                        if car.position < head_of_line_blocking_position[lane_idx]:
                            head_of_line_blocking_position[lane_idx] = car.position
                    else:
                        min_d_add = 0
                        if car.position < cfg.CCZ_LEN:
                            min_d_add = (2*2*cfg.CCZ_ACC_LEN/(cfg.MAX_SPEED+0)) - (2*cfg.CCZ_ACC_LEN/cfg.MAX_SPEED)

                        if car.turning == 'S':
                            car.D = solver.NumVar(max(0+min_d_add, spillback_delay), solver.infinity(), 'd'+str(car.ID))
                        else:
                            min_d = (2*cfg.CCZ_DEC2_LEN/(cfg.MAX_SPEED+cfg.TURN_SPEED)) - (cfg.CCZ_DEC2_LEN/cfg.MAX_SPEED)
                            car.D = solver.NumVar(max(min_d+min_d_add, spillback_delay), solver.infinity(), 'd'+str(car.ID))
    '''

    for dst_lane_idx in range(len(others_road_info)):
        # Doesn't connected to other intersections
        if others_road_info[dst_lane_idx] == None:
            for lane_idx in range(len(others_road_info)):
                for car in new_car_src_dst_lane[lane_idx][dst_lane_idx]:

                    if car.position > head_of_line_blocking_position[lane_idx]:
                        new_cars.remove(car)    # Blocked by the car at the front
                    else:
                        if car.turning == 'S':
                            car.D = solver.NumVar(0, solver.infinity(), 'd'+str(car.ID))
                        else:
                            min_d = (2*cfg.CCZ_DEC2_LEN/(cfg.MAX_SPEED+cfg.TURN_SPEED)) - (cfg.CCZ_DEC2_LEN/cfg.MAX_SPEED)
                            car.D = solver.NumVar(min_d, solver.infinity(), 'd'+str(car.ID))


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


    # part 7: pedestrian
    for car in new_cars:
        in_dir = car.in_dir
        out_dir = car.out_dir

        # Set pedestrian constraint if "in" is not none
        if pedestrian_time_mark_list[in_dir] != None:
            avoid_start_AT = pedestrian_time_mark_list[in_dir] - car.length/car.speed_in_intersection
            avoid_end_AT = pedestrian_time_mark_list[in_dir]+cfg.PEDESTRIAN_TIME_GAP

            if avoid_start_AT - car.OT <= 0:
                bound = avoid_end_AT - car.OT
                tmp_conts2 = solver.Constraint(bound, solver.infinity())
                tmp_conts2.SetCoefficient(car.D, 1)
            else:
                flag = solver.IntVar(0, 1, 'flag_ped_in_'+str(car.ID))

                bound = avoid_end_AT - car.OT - cfg.LARGE_NUM
                tmp_conts2 = solver.Constraint(bound, solver.infinity())
                tmp_conts2.SetCoefficient(car.D, 1)
                tmp_conts2.SetCoefficient(flag, -cfg.LARGE_NUM)

                bound = -(avoid_start_AT - car.OT)
                tmp_conts1 = solver.Constraint(bound, solver.infinity())
                tmp_conts1.SetCoefficient(car.D, -1)
                tmp_conts1.SetCoefficient(flag, cfg.LARGE_NUM)

        #'''
        # Set pedestrian constraint if "out" is not none
        if pedestrian_time_mark_list[out_dir] != None:
            avoid_start_AT = pedestrian_time_mark_list[out_dir] - car.length/cfg.MAX_SPEED
            avoid_end_AT = pedestrian_time_mark_list[out_dir]+cfg.PEDESTRIAN_TIME_GAP

            travel_in_inter_time = inter_length_data.getIntertime(car.lane, car.turning)


            if avoid_start_AT - car.OT - travel_in_inter_time <= 0:
                bound = avoid_end_AT - car.OT - travel_in_inter_time
                tmp_conts2 = solver.Constraint(bound, solver.infinity())
                tmp_conts2.SetCoefficient(car.D, 1)
            else:
                flag = solver.IntVar(0, 1, 'flag_ped_out_'+str(car.ID))

                bound = avoid_end_AT - car.OT - travel_in_inter_time - cfg.LARGE_NUM
                tmp_conts2 = solver.Constraint(bound, solver.infinity())
                tmp_conts2.SetCoefficient(car.D, 1)
                tmp_conts2.SetCoefficient(flag, -cfg.LARGE_NUM)

                bound = -(avoid_start_AT - car.OT - travel_in_inter_time)
                tmp_conts1 = solver.Constraint(bound, solver.infinity())
                tmp_conts1.SetCoefficient(car.D, -1)
                tmp_conts1.SetCoefficient(flag, cfg.LARGE_NUM)
        #'''



    # part 8: set objective
    objective = solver.Objective()

    for c_idx in range(len(new_cars)):
        objective.SetCoefficient(new_cars[c_idx].D, 1)
    objective.SetMinimization()

    '''
    print('Car num', len(new_cars))
    print('Number of variables =', solver.NumVariables())
    print('Number of constraints =', solver.NumConstraints())
    #'''

    # part 9: Solve the problem
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
