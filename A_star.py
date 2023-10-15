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


class Node: 
    def __init__(self, position, parent=None): 
        self.position = position 
        self.parent = parent 

        self.f = 0 # g+h, total estimated cost
        self.g = 0 # cost from start node to current node
        self.h = 0 # heuristic cost (estimated cost to end)

    def __eq__(self, other_node): #equals method to compare if nodes are at same pos
        return self.position == other_node.position


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

    #TODO: ADD ERROR CHECKING FOR VALID START,GOAL
    
    def plan_path(self, start, goal): 
        #plan path from start to goal
        start = Node(self.position_to_index(*start))
        goal = Node(self.position_to_index(*goal))

        open_set = [start] 
       
        
    #manhattan distance 
    def heuristic(self,start,goal): 
        return np.abs(goal[0]-start[0]) + np.abs(goal[1]-start[0])
    
    #
    def get_neighbors(self,node): 
        neighbors = []
        pos_x, pos_y = node.position 

        for dx in [-1,0,1]: 
            for dy in [-1,0,1]: 
                new_x, new_y = pos_x+dx, pos_y+dy 

                if self.within_grid(new_x,new_y) and (dx,dy) != (0,0): 
                    if self.grid[new_y,new_x] == 1: #if wall there cost is 1k
                        cost = 1000
                    else: 
                        cost = 1
                    neighbors.append((Node(position=(new_x,new_y)),cost)) #returns neighbor and movement cost to that node 
        return neighbors 


    def within_grid(self,x,y): 
        return 0<=x<self.grid.shape[1] and 0 <=y < self.grid.shape[0]


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
    test.plan_path((0.5,-1.5),(5,5)) 


if __name__=='__main__': 
    main() 
