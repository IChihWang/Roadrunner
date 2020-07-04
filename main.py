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

from gen_route import generate_routefile
import socket



# For debug
#from playsound import playsound

from IntersectionManager import IntersectionManager
#from myGraphic import Gui
#import myGraphic


#myGraphic.gui = Gui()

car_dst_dict = dict()
car_status_dict = dict()

# Creating variables for theads
car_src_dict = dict()
car_path_dict = dict() # (car_id, path(node_turn_dict) )
send_str = ""
###################


def run():
    global car_path_dict
    global send_str
    global car_src_dict
    global car_status_dict

    """execute the TraCI control loop"""
    simu_step = 0

    # Create a list with intersection managers
    intersection_manager_list = []
    for idx in range(1, cfg.INTER_SIZE+1):
        for jdx in range(1, cfg.INTER_SIZE+1):
            intersection_manager_id = "%3.3o"%(idx) + "_" + "%3.3o"%(jdx)
            intersection_manager = IntersectionManager(intersection_manager_id)
            intersection_manager_list.append(intersection_manager)



    try:
        while traci.simulation.getMinExpectedNumber() > 0:

            #Get timestamp
            TiStamp1 = time.time()
            
            if (simu_step*10)//1/10.0 == 500:
                break
                
            traci.simulationStep()
            all_c = traci.vehicle.getIDList()
            server_send_str = ""
            # Update the position of each car
            for car_id in all_c:
                
                # Generate source/destination
                if car_id not in car_dst_dict:
                    car_status_dict[car_id] = "NEW"
                    
                    
                
                    # Get source
                    sink_id = traci.vehicle.getRoadID(car_id) # The route ID is the sink ID in MiniVnet
                    src_node_idx = sink_id
                    
                    # Genterate destination
                    dst_node_idx = src_node_idx
                    while src_node_idx == dst_node_idx:
                        dst_node_idx = random.randrange(0,cfg.INTER_SIZE*4)
                    car_dst_dict[car_id] = dst_node_idx
            
            
                lane_id = traci.vehicle.getLaneID(car_id)
                    # TODO: send request
                    # TODO time lower bound
                
                is_handled = False
                for intersection_manager in intersection_manager_list:
                    if intersection_manager.check_in_my_region(lane_id):
                        is_handled = True
                        car_turn = "S"  # by default
                        
                        if car_id in car_path_dict:
                            car_turn = car_path_dict[car_id][intersection_manager.ID]
                            
                        data = intersection_manager.update_car(car_id, lane_id, simu_step, car_turn)
                        if data != None:
                            car_length = data[0]
                            time_offset = data[1]
                            intersection_id = data[2]
                            intersection_from_direction = data[3]
                            
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
                    del_car_id_list.append(car_id)

            for car_id in del_car_id_list:
                del car_dst_dict[car_id]
                    
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
            
                send_str = server_send_str
                
                # Send request
                sock.sendall(send_str + "@")
                
                # Receive the result
                data = ""
                while len(data) == 0 or data[-1] != "@":
                    data += sock.recv(8192)
                    
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
                        for node_turn in nodes_turn:
                            intersection_id, turn = node_turn.split(":")
                            node_turn_dict[intersection_id] = turn

                        if (car_id not in car_path_dict) or (car_src_dict[car_id] in node_turn_dict):
                            # Update the new path
                            #print(car_id, "Update new path", (car_id not in car_path_dict))
                            car_path_dict[car_id] = node_turn_dict
                        else:
                            # The car and expected path is not synchronized
                            if car_status_dict[car_id] == "OLD":
                                # Force reroute
                                car_status_dict[car_id] = "NEW"
                                #print(car_id, "Force reroute")
                    
                    
                    
                    
            

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


    print(sys.argv[1], int(sys.argv[2]), int(sys.argv[3]))

    # Print out the measurements
    #print("Average total delay: ", total_delays/car_num)
    #print("Average delay by scheduling: ", total_delays_by_sche/car_num)
    print(intersection_manager.total_delays/intersection_manager.car_num, intersection_manager.total_delays_by_sche/intersection_manager.car_num, intersection_manager.car_num)

    print("avg_fuel = ",intersection_manager.total_fuel_consumption/intersection_manager.fuel_consumption_count)

    sys.stdout.flush()

    traci.close()
    send_lock.release()


    
    
def server_handler(sock):
    is_continue = True
    global car_path_dict
    global car_src_dict
    global send_str
    global car_status_dict
    
    try:
        while is_continue:
        
            # Send request
            send_lock.acquire() # Use the lock to block here
            string_write_lock.acquire()
            sock.sendall(send_str + "@")
            string_write_lock.release()
            
            # Receive the result
            data = ""
            while len(data) == 0 or data[-1] != "@":
                data += sock.recv(8192)
                
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
                    for node_turn in nodes_turn:
                        intersection_id, turn = node_turn.split(":")
                        node_turn_dict[intersection_id] = turn

                    if (car_id not in car_path_dict) or (car_src_dict[car_id] in node_turn_dict):
                        # Update the new path
                        #print(car_id, "Update new path", (car_id not in car_path_dict))
                        car_path_dict[car_id] = node_turn_dict
                    else:
                        # The car and expected path is not synchronized
                        if car_status_dict[car_id] == "OLD":
                            # Force reroute
                            car_status_dict[car_id] = "NEW"
                            #print(car_id, "Force reroute")
           
    except Exception as e:
        traceback.print_exc()
    


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
    print("Usage: python code.py <arrival_rate (0~1.0)> <seed> <schedular>")
    
    #HOST, PORT = "128.238.147.124", 9999
    HOST, PORT = "localhost", 9999
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((HOST, PORT))

    seed = int(sys.argv[2])
    random.seed(seed)  # make tests reproducible
    numpy.random.seed(seed)

    options = get_options()

    # this script has been called from the command line. It will start sumo as a server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # 0. Generate the intersection information files
    os.system("bash gen_intersection/gen_data.sh " + str(cfg.LANE_NUM_PER_DIRECTION))

    # 1. Generate the route file for this simulation
    arrival_rate = float(sys.argv[1])
    generate_routefile(arrival_rate)





    try:
        # 3. This is the normal way of using traci. sumo is started as a subprocess and then the python script connects and runs

        traci.start([sumoBinary, "-c", "data/icacc+.sumocfg",
                                 "--tripinfo-output", "tripinfo.xml","--step-length", str(cfg.TIME_STEP),
                                 "--collision.mingap-factor", "0"])


        
        string_write_lock = threading.Lock()
        send_lock = threading.Lock()
        send_lock.acquire()
        
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
    except:
        None

    try:
        send_lock.release()
    except:
        None
    sock.close()
