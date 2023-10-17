import numpy as np 
import matplotlib.pyplot as plt 
#test
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
        start_node = Node(self.position_to_index(*start))
        goal_node = Node(self.position_to_index(*goal))

        frontier = [start_node] #nodes to explore
        visited = [] 
        
        while frontier: 
            cur_node = min(frontier, key=lambda x:x.f) #select node from neighbors with lowest total cost, f

            visited.append(cur_node) 
            frontier.remove(cur_node) 

            if cur_node == goal_node: 
                path = [] 
                while cur_node: 
                    path.append(cur_node.position) 
                    cur_node = cur_node.parent 
                return path[::-1]
            
            neighbors = self.get_neighbors(cur_node) 

            for neigh,cost in neighbors: 
                neigh.g = cur_node.g + cost 
                neigh.h = self.manhattan_heuristic(neigh.position,goal_node.position)
                neigh.f = neigh.g + neigh.h 

                if neigh not in visited and neigh not in frontier: 
                    neigh.parent = cur_node 
                    frontier.append(neigh) #want to explore neighbors of neigh 

                elif neigh in frontier: #this node was reachable from another node. we want the shorter cost one to it
                    prev_found_neigh = frontier[frontier.index(neigh)] #find idx of neigh 
                    if neigh.g < prev_found_neigh.g:
                        prev_found_neigh.g = neigh.g
                        prev_found_neigh.f = prev_found_neigh.g + prev_found_neigh.h
                        prev_found_neigh.parent = cur_node

        return None

    #manhattan distance 
    def manhattan_heuristic(self,start,goal): 
        return np.abs(goal[0]-start[0]) + np.abs(goal[1]-start[1])

    
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
    
    def visualize_results(self, path):
        plt.figure(figsize=(5,10))
        plt.grid(color='black', linewidth=0.5)

        plt.xticks(np.arange(-0.5, self.grid.shape[1], 1), [])
        plt.yticks(np.arange(-0.5, self.grid.shape[0], 1), [])

        plt.imshow(self.grid, cmap='gray_r')

        if path:
            path_x, path_y = zip(*path) 
            plt.plot(path_x, path_y, 'ro-', markersize=5)

        plt.title("A* Pathfinding Results")
        plt.show()


def main(): 
    test = A_star(1,(-2,5),(-6,6),1) 
    test.display_grid()
    path = test.plan_path(start=(0.5,-1.5),goal=(.5,1.5))   
    test.visualize_results(path)
    path = test.plan_path(start=(4.5,3.5),goal=(4.5,-1.5))   
    test.visualize_results(path)
    path = test.plan_path(start=(-0.5,5.5),goal=(1.5,3.5))   
    test.visualize_results(path)

    print(path)


if __name__=='__main__': 
    main() 
