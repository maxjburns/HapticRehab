import numpy as np
import pickle

class DataLogger:
    def __init__(self, headers:list[str]):
        self.data_ls = []
        self.data_len = len(headers)
        self.headers = headers
        self.data_arr = None
    
    def log_data(self, new_data_ls:list):
        if len(new_data_ls) != self.data_len:
            print("DATA COLLECTION ERROR: IMPROPER LENGTH")
            return False
        
        self.data_ls.append(new_data_ls)

        return True

    def save_data(self, save_path:str):
        """
        save data_ls as a pickle file, for later loading and visualization. converts to numpy first.
        """
        
        with open(save_path, 'wb') as f:
            
            if type(self.data_arr) != np.ndarray and self.data_arr == None:
                self.data_arr = np.array(self.data_ls)
                del self.data_ls

            pickle.dump(self, f)

    def export_as_csv(self, save_path:str):
        """
        save data_ls as a csv 
        """
        if type(self.data_arr) != np.ndarray and self.data_arr == None:
            self.data_arr = np.array(self.data_ls)
            del self.data_ls
        header_str = ",".join(self.headers)
        np.savetxt(save_path, self.data_arr, delimiter=",", header=header_str, comments='', fmt="%.6f")
    
    # TODO: add data visualization methods