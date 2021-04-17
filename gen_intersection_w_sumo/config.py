LANE_NUM_PER_DIRECTION = 3


MAX_SPEED = 11.18    #25mph
TURN_SPEED = 10.0    #25mph
# REMEMBER!!!! Set this as a FLOATING NUMBER!!!!!!

SUMO_TIME_ERR = 0

AZ_LEN = 75.0
PZ_LEN = 25.0
GZ_LEN = 25.0
BZ_LEN = 25.0
CCZ_LEN = 50.0

LANE_WIDTH = 3.2

LARGE_NUM = 1e4

HEADWAY = 3 # 3 meter
CCZ_ACC_LEN = 5.0
CCZ_DEC2_LEN = 2.5
MAX_ACC = (MAX_SPEED*MAX_SPEED)/(2*CCZ_ACC_LEN)

CCZ_CATCHUP_MIN_SPEED = 3

CAR_MAX_LEN = 15
CAR_MIN_LEN = 5
CAR_AVG_LEN = (CAR_MAX_LEN+CAR_MIN_LEN)/2

N_TIME_STEP = 3600  # number of time steps
TIME_STEP = 0.05

DISTANCE = 1.5 # 1.5 lane

PEDESTRIAN_POSSIBILITY = 0 # 0~1
PEDESTRIAN_TIME_GAP = 0 # 60 second
