from utils.data_collection import DataLogger
import pickle
import matplotlib.pyplot as plt

def load_data_logger(path:str):
    """
    Helper for loading pickled data logger from a previous data collection session.
    """
    with open(path, 'rb') as f:
        data_logger_obj = pickle.load(f)
    if type(data_logger_obj) != DataLogger:
        raise TypeError("Please load data logger object")
    
    return data_logger_obj


if __name__=="__main__":
    path_to_data_file = "data/subl_test_1.pkl"

    data_logger = load_data_logger(path_to_data_file)


    data_arr = data_logger.data_arr
    headers = data_logger.headers

    print(data_arr.shape)
    print(headers)
    # from here we can work with data, should be of format:
    # 0 --> time, 1--> current_angle, 2--> goal_angle, 3--> servo_command, 4--> vibe_command_1, ... 11--> vibe_command_7
    # data_arr[time_index, desired_data_index]
