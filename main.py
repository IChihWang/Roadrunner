from __future__ import absolute_import
from __future__ import print_function


import os
import sys

sys.path.append('/usr/share/sumo/tools/')
# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import optparse
import random
import numpy
import threading
import time
import matplotlib.pyplot as plt
import numpy as np

from ortools.linear_solver import pywraplp
from sumolib import checkBinary
import traci
import traceback

import config as cfg

from gen_route import generate_routefile_with_src_dst, generate_routefile_with_src_dst_2
import socket
import json
import csv



# For debug
#from playsound import playsound

from IntersectionManager import IntersectionManager
#from myGraphic import Gui
#import myGraphic


#myGraphic.gui = Gui()

car_dst_dict = dict()
car_status_dict = dict()
car_intersection_id_dict = dict()

# Creating variables for theads
car_src_dict = dict()
car_path_dict = dict() # (car_id, path(node_turn_dict) )
car_path_to_write = dict()
send_str = ""

src_dst_dict = None     # Load from the file (car_id, (src_idx, dst_idx))
###################

simulation_time = 600

def worker(sock, server_send_str):
    global car_path_dict
    global car_path_to_write
    try:
        send_str = server_send_str

        # Send request
        sock.sendall(send_str + "@")

        # Receive the result
        data = ""
        while len(data) == 0 or data[-1] != "@":
            get_str = sock.recv(8192)
            if get_str == None:
                return
            data += get_str

        if data == None:
            is_continue = False

        # Parse data to path_dict
        data = data[0:-2]
        cars_data_list = data.split(";")
        if len(data) > 0:

            for car_data in cars_data_list:
                car_data_list = car_data.split(",")
                car_id = car_data_list[0]
                route_str = car_data_list[1][0:-1]
                nodes_turn = route_str.split("/")

                node_turn_dict = dict()

                if len(route_str)>0:
                    for node_turn in nodes_turn:
                        intersection_id, turn = node_turn.split(":")
                        node_turn_dict[intersection_id] = turn


                    if (car_id in car_path_dict):
                        current_intersection = car_intersection_id_dict[car_id]
                        if current_intersection in car_path_dict[car_id]:
                            node_turn_dict[current_intersection] = car_path_dict[car_id][current_intersection]

                    car_path_to_write[car_id] = node_turn_dict
    except Exception as e:
        traceback.print_exc()


def run():
    global car_path_dict
    global send_str
    global car_src_dict
    global car_status_dict
    global car_path_to_write


    car_enter_time = dict()
    car_travel_time = []
    car_delay_time = []


    """execute the TraCI control loop"""
    simu_step = 0
    total_car_num = 0
    worker_thread = None

    # Create a list with intersection managers
    intersection_manager_list = []
    intersection_manager_dict = dict()
    for idx in range(1, cfg.INTER_SIZE+1):
        for jdx in range(1, cfg.INTER_SIZE+1):
            intersection_manager_id = "%3.3o"%(idx) + "_" + "%3.3o"%(jdx)
            intersection_manager = IntersectionManager(intersection_manager_id)
            intersection_manager_list.append(intersection_manager)
            intersection_manager_dict[(idx, jdx)] = intersection_manager

    for idx in range(1, cfg.INTER_SIZE+1):
        for jdx in range(1, cfg.INTER_SIZE+1):
            if idx <= cfg.INTER_SIZE-1:
                intersection_manager_dict[(idx, jdx)].connect(1, intersection_manager_dict[(idx+1, jdx)], 3)

            if jdx <= cfg.INTER_SIZE-1:
                intersection_manager_dict[(idx, jdx)].connect(2, intersection_manager_dict[(idx, jdx+1)], 0)


    try:
        while traci.simulation.getMinExpectedNumber() > 0:

            #Get timestamp
            TiStamp1 = time.time()

            #if (simu_step*10)//1/10.0 >= simulation_time:
                #break

            traci.simulationStep()
            all_c = traci.vehicle.getIDList()
            server_send_str = ""
            # Update the position of each car
            for car_id in all_c:

                # Generate source/destination
                if car_id not in car_dst_dict:
                    car_status_dict[car_id] = "NEW"

                    # Get source & destination

                    src_node_idx, dst_node_idx = src_dst_dict[car_id]
                    car_dst_dict[car_id] = dst_node_idx


                    # Record entering time
                    car_enter_time[car_id] = simu_step

                    total_car_num += 1


                lane_id = traci.vehicle.getLaneID(car_id)

                is_handled = False
                for intersection_manager in intersection_manager_list:
                    if intersection_manager.check_in_my_region(lane_id):
                        is_handled = True
                        car_turn = "S"  # by default

                        if car_id in car_path_dict:
                            if intersection_manager.ID in car_path_dict[car_id]:
                                car_turn = car_path_dict[car_id][intersection_manager.ID]
                            elif car_status_dict[car_id] == "OLD":
                                print(car_id + " didn't find a proper path at " + intersection_manager.ID)
                                car_status_dict[car_id] = "NEW"

                        data = intersection_manager.update_car(car_id, lane_id, simu_step, car_turn)
                        if data != None:
                            car_length = data[0]
                            time_offset = data[1]
                            intersection_id = data[2]
                            intersection_from_direction = data[3]

                            car_intersection_id_dict[car_id] = intersection_id

                            server_send_str += car_id + ","

                            server_send_str += car_status_dict[car_id] + ","

                            server_send_str += str(car_length) + ","
                            server_send_str += str(car_dst_dict[car_id]) + ","
                            server_send_str += "%1.4f"%(time_offset) + "," + str(intersection_id) + "," + str(intersection_from_direction) + ";"
                            car_src_dict[car_id] = intersection_id

                        break

                if not is_handled:
                    traci.vehicle.setSpeed(car_id, cfg.MAX_SPEED)

            del_car_id_list = []
            for car_id in car_dst_dict:
                if car_id not in all_c:
                    # The car exits the system
                    car_status_dict[car_id] = "Exit"
                    del car_path_dict[car_id]
                    del car_src_dict[car_id]
                    del car_intersection_id_dict[car_id]
                    del_car_id_list.append(car_id)

            for car_id in del_car_id_list:
                travel_time = simu_step-car_enter_time[car_id]
                car_travel_time.append(travel_time)


                src_node_idx, dst_node_idx = src_dst_dict[car_id]

                dst_node_idx = (dst_node_idx - ((src_node_idx // cfg.INTER_SIZE) * cfg.INTER_SIZE)) % (cfg.INTER_SIZE*4)
                src_node_idx = src_node_idx % cfg.INTER_SIZE

                node_idx_diff = 0
                if dst_node_idx < cfg.INTER_SIZE:
                    node_idx_diff = abs(src_node_idx-dst_node_idx)
                elif dst_node_idx < cfg.INTER_SIZE*2:
                    node_idx_diff = dst_node_idx - src_node_idx -1
                elif dst_node_idx < cfg.INTER_SIZE*3:
                    dst_node_idx = cfg.INTER_SIZE*3-dst_node_idx -1
                    node_idx_diff = abs(src_node_idx-dst_node_idx) + cfg.INTER_SIZE-1
                else:
                    src_node_idx = cfg.INTER_SIZE - src_node_idx -1
                    dst_node_idx = cfg.INTER_SIZE*4-dst_node_idx +2
                    node_idx_diff = dst_node_idx - src_node_idx -1

                optimal_time = node_idx_diff*(float(cfg.TOTAL_LEN)/cfg.MAX_SPEED) + (float(cfg.TOTAL_LEN)*2/cfg.MAX_SPEED)*2

                car_delay_time.append(travel_time - optimal_time)

                del car_dst_dict[car_id]
                del car_enter_time[car_id]


            # Early wait

            if (simu_step-1)%cfg.ROUTING_PERIOD < cfg.TIME_STEP:
                if worker_thread != None:
                    worker_thread.join()
                    car_path_dict = car_path_to_write

            # Send string to the server handler
            if simu_step%cfg.ROUTING_PERIOD < cfg.TIME_STEP:

                delete_key_list = []
                for car_id, car_status in car_status_dict.items():
                    if car_status == "Exit":
                        delete_key_list.append(car_id)
                        server_send_str += car_id + "," + car_status + ";"
                    elif car_status == "NEW":
                        car_status_dict[car_id] = "OLD"

                for car_id in delete_key_list:
                    del car_status_dict[car_id]

                worker_thread = threading.Thread(target=worker, args=(sock, server_send_str,))
                worker_thread.start()

                worker_thread.join()
                car_path_dict = car_path_to_write






            for intersection_manager in intersection_manager_list:
                intersection_manager.run(simu_step)


            simu_step += cfg.TIME_STEP



            #Synchronize time
            deltaT = cfg.TIME_STEP
            TiStamp2 = time.time() - TiStamp1
            if TiStamp2 > deltaT:
                pass
            else:
                #time.sleep(deltaT-TiStamp2)
                pass
    except Exception as e:
        traceback.print_exc()

    car_travel_time = car_travel_time	# Skip first 500 cars
    car_delay_time = car_delay_time

    avg_travel_time = (sum(car_travel_time)/len(car_travel_time))
    avg_delay_time = (sum(car_delay_time)/len(car_delay_time))
    served_car_num = (len(car_travel_time))
    actual_departure_rate = (float(served_car_num)/simu_step)
    actual_arrival_rate = (float(total_car_num)/simu_step)
    #print("Average delay: %f" % avg_travel_time)
    #print("Car number: %i" % (len(car_travel_time)))
    #print("Arrival rate: %f" % (len(car_travel_time)/600))
    sys.stdout.flush()
    sock.sendall("END@")

    traci.close()

    with open('../result/traveling_time.csv', 'a') as csvfile:
        file_writer = csv.writer(csvfile, lineterminator='\n')
        to_write_list = [sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6], sys.argv[7]]
        to_write_list.append(avg_travel_time)
        to_write_list.append(avg_delay_time)
        to_write_list.append(total_car_num)
        to_write_list.append(actual_arrival_rate)
        to_write_list.append(served_car_num)
        to_write_list.append(actual_departure_rate)
        file_writer.writerow(to_write_list)




##########################
# Setup running options for sumo
def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options





###########################
# Main function
if __name__ == "__main__":
    print("Usage: python code.py <arrival_rate (0~1.0)> <seed> <grid size>")
    sys.argv[7]

    cfg.INTER_SIZE = int(sys.argv[3])

    #HOST, PORT = "128.238.147.124", 9909
    HOST, PORT = "localhost", 9909
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((HOST, PORT))

    seed = int(sys.argv[2])
    random.seed(seed)  # make tests reproducible
    numpy.random.seed(seed)


    options = get_options()

    # this script has been called from the command line. It will start sumo as a server, then connect and run
    sumoBinary = checkBinary('sumo')

    # 0. Generate the intersection information files
    #os.system("bash gen_intersection/gen_data.sh " + str(cfg.LANE_NUM_PER_DIRECTION))

    # 1. Generate the route file for this simulation
    arrival_rate = sys.argv[1]

    generate_routefile_with_src_dst(cfg.INTER_SIZE, float(arrival_rate), seed, simulation_time)


    # Load from the file
    src_dst_file_name = "%i_%s_%i_src_dst.json" % (cfg.INTER_SIZE, arrival_rate, seed)
    with open('data/routes/'+src_dst_file_name) as json_file:
        src_dst_dict = json.load(json_file)


    try:
        # 3. This is the normal way of using traci. sumo is started as a subprocess and then the python script connects and runs

        net_name = "lane%iby%i.net.xml" % (cfg.INTER_SIZE, cfg.INTER_SIZE)
        route_name = "%i_%s_%i.rou.xml" % (cfg.INTER_SIZE, arrival_rate, seed)
        traci.start([sumoBinary, "-c", "data/UDTA.sumocfg",
                                 "--tripinfo-output", "tripinfo.xml","--step-length", str(cfg.TIME_STEP),
                                 "--collision.mingap-factor", "0",
                                 "-n", "data/net/" + net_name,
                                 "-r", "data/routes/" + route_name])




        # Echo and tell the size of the network
        init_message = "My_grid_size:" + str(cfg.INTER_SIZE)
        init_message += ":My_schedule_period:" + str(int(cfg.GZ_LEN/cfg.MAX_SPEED))
        init_message += ":My_routing_period:" + str(cfg.ROUTING_PERIOD_NUM)
        sock.sendall(init_message)

        print("Server replies: ", sock.recv(1024))

        #server_thread = threading.Thread(target=server_handler, args=(sock,))
        #server_thread.start()

        # 4. Start running SUMO
        run()
    except Exception as e:
        traceback.print_exc()

    sock.close()
