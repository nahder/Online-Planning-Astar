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
        num_rows = int((self.y_range[1] - self.y_range[0]) / self.cell_size)
        num_cols = int((self.x_range[1] - self.x_range[0]) / self.cell_size)
        self.grid = np.zeros((num_rows,num_cols))
        self.populate_landmarks() 
    
    def populate_landmarks(self): 
        landmark_data = read_dat_file('ds1/ds1_Landmark_Groundtruth.dat')
        landmarks_x, landmarks_y = landmark_data[:,1], landmark_data[:,2] #obtain world positions for landmarks, need to discretize to grid locations

        for l_x, l_y in zip(landmarks_x, landmarks_y): 
            l_idx, l_idy = self.position_to_index(l_x,l_y)
            self.grid[l_idy,l_idx] = 1 #[row,col]

    def plan_path(self, start, goal): 
        #plan path from start to goal
        pass 

    #converts between continuous position to grid index
    def position_to_index(self,pos_x,pos_y): 
        #1) subtract position from minimum range (to find relative distance from beginning) 
        #2) divide by cell size 
        idx_x, idx_y = int((pos_x - self.x_range[0]) / self.cell_size) , int((pos_y - self.y_range[0]) / self.cell_size)
        return idx_x, idx_y

    def display_grid(self): 
        print(self.grid) 


def main(): 
    test = A_star(1,(-2,5),(-6,6),1) 
    test.display_grid()


if __name__=='__main__': 
    main() 
