import json
import sys




if __name__ == "__main__":


    with open(sys.argv[1]) as json_file:
        data_list = json.load(json_file)
        

    # Constuct training data
    x_batch = []
    y_batch = []
    
    for data in data_list:
        x = []
        y = []  # TODO: decide whether y include lane assignment
        sched_car = data['sched_car']
        n_sched_car = data['n_sched_car']
        
        for car_tuple in sched_car:
            x.append(car_tuple)
            
        for car_tuple in n_sched_car:
            x.append((car_tuple[0],car_tuple[1]))
            y.append((car_tuple[2],car_tuple[3]))
        
        x_batch.append(x)
        y_batch.append(y)
        
        
        
        
    print(x_batch)
    print(y_batch)