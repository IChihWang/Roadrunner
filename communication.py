# Interface for communication delay and drop
# Temporary implementation: random delay
import random
random.seed(0)

# Function to get delay/drop
# Input: two items (cars, facilities "AIM")
# Output: communication delays in ms (INF for drop)
def get_communication_delay(src_item, dst_item):
    '''
    if random.choice([False]*19 + [True]):
        return float('inf')
    else:
        return random.randrange(0, 100, 1)*0.001 # ms
    '''
    return 0

def get_retry_timeout(src_item, dst_item):
    return 0.01     # 10 ms
