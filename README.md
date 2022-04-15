# Roadrunner+
An Autonomous Intersection Management Cooperating with Connected Autonomous Vehicles and Pedestrians with Spillback Considered

## Requirements:  
ortools (pip install ortools)   
matplotlib (pip install matplotlib)
numpy (pip install numpy)

## Platform: 
Run with SUMO 18.0, python 3

## Quick walkthrough with files: 
**1. main.py**
  - Start SUMO with gui or without gui (Line 146sumo-gui/ sumo in checkBinary)
  - Generate intersection information (Line 149, call gen_data.sh to generate intersection infos)
  - Generate traffic with given arrival rate and random seed (Line 153, call the function in gen_route.py)
  - "Usage: python main.py <arrival_rate (0~1.0)> <seed> <schedular> <is_slowdown_control T/F>"
  - The <scheduler> is integers from 0~3, used in (Line 384 in IntersectionManager.py)
    - 0: algorithm in Roadrunner+ (it was temporarily named IcaccPlus)
    - 1: ICACC (which didn't implement pedestrian)
    - 2: FCFS reservation (also consider pedestrian)
    - 3: First-come first-pass (FCFS_no_reservation, which didn't implement pedestrian)
  
**2. config.py**
  - Records all the configurations
    - LANE_NUM_PER_DIRECTION: number of lane per direction in a four-legged intersection
    - MAX_SPEED/TURN_SPEED: the speed defined for Roadrunner+ to control near the intersection (in m/s)
    - SUMO_TIME_ERR: this is to calibrate the time difference between SUMO simulation and Roadrunner+ algorithm (Look into this when collision happens at speed control near the intersection)
    - *_LEN: length of each zone
    - LANE_WIDTH: the width of lane (this number needs to match the setting of SUMO network)
    - HEADWAY: a defult headway between cars (3 meters)
    - CCZ_*_LEN, MAX_ACC: the detailed setting for the cuise control zone (i.e., the length needs to be smaller than CCZ_LEN)
    - CAR_*: the length of the cars
    - N_TIME_STEP: time of simulation that SUMO has to conduct (in seconds in SUMO time)
    - TIME_STEP: size of time step that SUMO takes for the simulation
    - PEDESTRIAN_POSSIBILITY: probability of generating a pedestrian in each time step (the pedestrian is not printed in SUMO)
    - PEDESTRIAN_TIME_GAP: the time gap that researved for the pedestrian
  
**3. data/**
  - Record the settings for the SUMO simulation, including the network (.net.xml)
  - Generated traffic information (.rou.xml) is saved here
  
**4. gen_route.py**
  - The traffic generating function (generate_routefile) is called in main.py
  - The traffic data is saved in data/

**5. inter_info/**
  - The recorded safe gap data within the intersection
  - Files: lane_info*.json. The number is the lane number per direction of the intersection
  - Files are generated with the programs in gen_intersection.
  - **Needs to generate the according files before the simulation starts (i.e., calling main.py)**.
  

**6. inter_length_info/**
  - The recorded trajectory length within the intersection for each turn and lane. (The information is used for lane advising)
  - Files: lane_info*.json. The number is the lane number per direction of the intersection
  - Files are generated with the programs in gen_intersection.
  - **Needs to generate the according files before the simulation starts (i.e., calling main.py)**.

**7. advise_info/**
  - The recorded blocks that needs to be reserved in the intersection. (The information is used for lane advising)
  - Files: advise_info*.json. The number is the lane number per direction of the intersection
  - Files are generated with the programs in gen_intersection.
  - **Needs to generate the according files before the simulation starts (i.e., calling main.py)**.
  
**8. gen_intersection/**
  - Programs that generate files in 5-7.
  - Use $make to compile the .cpp files (These .cpp generates the basic information of the intersection, regardless the config.py settings)
  - Call "$bash gen_data.sh" to convert the raw information to a usible data with config.py settings.
  - After running make and bash, the results should be found in 5-7.
  
**9. milp.py**
  - The MILP formulations for each scheduling algorithm
  - Note that FCFS is implemented with Roadrunner+ algorithm (IcaccPlus) by sending one car at a time
  - Note that FCFS_no_reservation is implemented with ICACC by sending one car at a time (because it force the later group to wait for the first-arrived group)
  - Note that FixedSignal is deprecated and replaced with a more reasonable traffic-signal model
  
**10. Cars.py**
  - The car control stages and behaviors, mainly the cruise control behaviors
  
**11. IntersectionManager.py**
  - The "main" of Roadrunner+, which includes the lane advising, scheduling, communicating with cars, and pedestrian handling
