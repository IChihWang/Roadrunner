
import csv
import os
import config as cfg

omnet_csv_file_path = '/media/seneca/Files/NYU-UNIZA-workspace/simu5G/simulations/NR/cars/'
omnet_csf_file_name = 'sumo_output.fifo'
omnet_csv_file = omnet_csv_file_path + omnet_csf_file_name

# Global variable to store the delay
communication_delay_dict = dict()    # {(src, dst): {'delay': delay, 'report_time': <report time>}}
csvfile = None

def read_delay_from_csv():
    global csvfile

    # Create file if file does not exist
    if not os.path.exists(omnet_csv_file):
        os.mknod(omnet_csv_file)
        
    else:
        if csvfile == None:
            csvfile = open(omnet_csv_file, 'r', newline = '')
    
        while True:
            csv_row = csvfile.readline()
            csv_row = csv_row.strip()
            
            if csv_row == 'DONE' or csv_row == '':
                break
            if csv_row == 'SimulationTime;SrcItem;DstItem;CommDelay;':
                continue
                
            # Read each row
            row = csv_row.split(';')
            
            if (len(row) < 4):
                print(row)
                break
            
            report_time = float(row[0])
            src = row[1];
            dst = row[2];
            delay = float(row[3])*0.001;	# In ms

            # Store the data into the dictionary
            if (src, dst) not in communication_delay_dict:
                communication_delay_dict[(src, dst)] = dict()

            communication_delay_dict[(src, dst)]['delay'] = delay
            communication_delay_dict[(src, dst)]['report_time'] = report_time
        
    
# Input: two items (cars, facilities "AIM")
# Output: communication delays in ms (INF for drop)

def get_communication_delay(src_item, dst_item, simu_time = None):
    delay = 0

    # Try to get delay data
    if (src_item, dst_item) in communication_delay_dict:
        # Get delay info
        delay_info = communication_delay_dict[(src_item, dst_item)]
        delay = delay_info['delay']
        report_time = delay_info['report_time']
        
        # Check the report time (check if simu_time is reported)
        if simu_time != None:
            if simu_time + 1 < report_time:	# Warning if the report time is >1s earlier
                print('Warning: OMNET reports earlier than simulation time. OMNET time: ', report_time, '; vehicle time: ', simu_time, 'src: ', src_item, 'dst: ', dst_item)
            elif simu_time - report_time > cfg.TIME_STEP:    # Inteprate "late message" as drop
                #print('Msg drop: OMNET time: ', report_time, '; vehicle time: ', simu_time, 'src: ', src_item, 'dst: ', dst_item)
                delay = float('inf')
            
    else:
        print('Msg drop: the delay is not found for ', (src_item, dst_item))
        delay = float('inf')
        
    return delay
        
# Temp value for communication retry
def get_retry_timeout(src_item, dst_item):
    return 0.5     # 10 ms

'''
def read_delay_from_csv():
    csvfile = open(omnet_csv_file, 'r', newline = '')
    csv_reader = csv.reader(csvfile, delimiter=';')
    next(csv_reader, None)    # Skip the header
    
    # Read each row
    for row in csv_reader:
        # Read the data
        report_time = float(row[0])
        src = row[1];
        dst = row[2];
        delay = float(row[3]);

        # Store the data into the dictionary
        if (src, dst) not in communication_delay_dict:
            communication_delay_dict[(src, dst)] = dict()
            
        communication_delay_dict[(src, dst)]['delay'] = delay
        communication_delay_dict[(src, dst)]['report_time'] = report_time
        
    csvfile.close()
    
# Input: two items (cars, facilities "AIM")
# Output: communication delays in ms (INF for drop)

def get_communication_delay(src_item, dst_item, simu_time = None):
    delay = 0

    # Try to get delay data
    if (src_item, dst_item) in communication_delay_dict:
        # Get delay info
        delay_info = communication_delay_dict[(src_item, dst_item)]
        delay = delay_info['delay']
        report_time = delay_info['report_time']
        
        # Check the report time (check if simu_time is reported)
        if simu_time != None:
            if simu_time < report_time:
                print('Warning: OMNET reports earlier than simulation time')
            elif simu_time - report_time > 1:	# error > 1 sec
                print('Error: the delay is reported 1 second ago')
                
    else:
        print('Error: the delay is not found for ', (src_item, dst_item))
        
    return delay
        
# Temp value for communication retry
def get_retry_timeout(src_item, dst_item):
    return 0.01     # 10 ms
'''


'''
# Interface for communication delay and drop
# Temporary implementation: random delay
import random
random.seed(0)

# Function to get delay/drop
# Input: two items (cars, facilities "AIM")
# Output: communication delays in ms (INF for drop)
def get_communication_delay(src_item, dst_item):
    if random.choice([False]*19 + [True]):
        return float('inf')
    else:
        return random.randrange(0, 100, 1)*0.001 # ms

def get_retry_timeout(src_item, dst_item):
    return 0.01     # 10 ms
'''
