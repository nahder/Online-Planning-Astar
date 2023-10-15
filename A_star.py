import numpy as np 
import matplotlib.pyplot as plt 


def read_dat_file(file_path):
    data = []
    with open(file_path, 'r') as file:
        for line in file:
            if line.startswith('#'):
                continue

            columns = line.strip().split()
            data_row = [float(col) for col in columns]
            data.append(data_row)
    return np.array(data)

class A_star: 
    def __init__(self, m, x_range, y_range, cell_size): 

        self.m = m 
        self.x_range = x_range 
        self.y_range = y_range
        self.cell_size = cell_size 
        
        self.generate_grid() 
    
    def generate_grid(self): 
        num_rows = int((self.y_range[1]-self.y_range[0])/self.cell_size)
        num_cols = int((self.x_range[1]-self.x_range[0])/self.cell_size)
        self.grid = np.zeros((num_rows,num_cols))
    
    def populate_landmarks(self): 
        landmark_data = read_dat_file('ds1/ds1_Landmark_Groundtruth.dat')
        landmarks_x, landmarks_y = landmark_data[:,1], landmark_data[:,2] 

        for l_x, l_y in zip(landmarks_x, landmarks_y): 
            pass


    def display_grid(self): 
        print(self.grid) 



def main(): 
    test = A_star(1,(-2,5),(-6,6),1) 
    # test.display_grid()
    test.populate_landmarks() 

if __name__=='__main__': 
    main() 
