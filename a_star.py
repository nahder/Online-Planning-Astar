import numpy as np 
import matplotlib.pyplot as plt 

#reads in .dat file into a np array
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

    def __hash__(self): #makes it hashable for set
        return hash(self.position)

class A_star: 
    def __init__(self, m, x_range, y_range, cell_size): 
        self.m = m 
        self.x_range = x_range 
        self.y_range = y_range
        self.cell_size = cell_size 
        self.generate_grid() 
    
    #creates grid based on provided x and y ranges
    def generate_grid(self): 
        num_rows = int((self.y_range[1] - self.y_range[0]) / self.cell_size)
        num_cols = int((self.x_range[1] - self.x_range[0]) / self.cell_size)
        self.grid = np.zeros((num_rows,num_cols))
        self.populate_landmarks() 
    
    #fills in landmarks and scales them based on cell size
    def populate_landmarks(self): 
        landmark_data = read_dat_file('ds1/ds1_Landmark_Groundtruth.dat')
        landmarks_x, landmarks_y = landmark_data[:,1], landmark_data[:,2]

        inflate_cells = int(0.3 / self.cell_size)

        for l_x, l_y in zip(landmarks_x, landmarks_y): 
            l_idx, l_idy = self.world_to_grid(l_x,l_y)
            
            for dx in range(-inflate_cells, inflate_cells + 1):
                for dy in range(-inflate_cells, inflate_cells + 1):
                    if self.within_grid(l_idx + dx, l_idy + dy):
                        self.grid[l_idy + dy, l_idx + dx] = 1

    #offline a* path planner
    def plan_path(self, start, goal): 
        self.s_v, self.g_v = start, goal

        start_node = Node(self.world_to_grid(*start))
        goal_node = Node(self.world_to_grid(*goal))

        frontier = {start_node} 
        node_dict = {start_node.position: start_node}  # dictionary for O(1) node lookup by position
        visited = set() #set for o(1) lookup to see if it exists
        
        while frontier: 
            cur_node = min(frontier, key=lambda x:x.f)
            if cur_node == goal_node: 
                path = [] 
                while cur_node: 
                    path.append(cur_node.position) 
                    cur_node = cur_node.parent 
                for x, y in path:
                    self.grid[y, x] = 0.35  # Set the path cells to light gray
                return path[::-1]

            visited.add(cur_node) 
            frontier.remove(cur_node)

            neighbors = self.get_neighbors(cur_node) 
            for neigh, cost in neighbors: 
                neigh.g = cur_node.g + cost 
                neigh.h = self.euclidean_distance(neigh.position,goal_node.position)
                neigh.f = neigh.g + neigh.h 

                if neigh not in visited:
                    if neigh not in frontier:
                        neigh.parent = cur_node 
                        frontier.add(neigh)
                        node_dict[neigh.position] = neigh
                    else:
                        existing_neigh = node_dict[neigh.position]
                        if neigh.g < existing_neigh.g:
                            existing_neigh.g = neigh.g
                            existing_neigh.f = existing_neigh.g + existing_neigh.h
                            existing_neigh.parent = cur_node

    #online astar path planner
    def online_plan_path(self, start, goal):
        self.s_v, self.g_v = start, goal
        start_node = Node(self.world_to_grid(*start))
        goal_node = Node(self.world_to_grid(*goal))

        frontier = {start_node}
        visited = set()
        path = []

        while frontier:
            cur_node = min(frontier, key=lambda x: x.f)
            visited.add(cur_node.position)
            path.append(cur_node.position)
            frontier.remove(cur_node)

            if cur_node == goal_node:
                for x, y in path:
                    self.grid[y, x] = 0.35  # Set the path cells to light gray
                return path

            neighbors = self.get_neighbors(cur_node)
            best_neighbor = None
            lowest_f = float('inf')
            
            for neigh, cost in neighbors:
                if neigh.position not in visited:
                    neigh.g = cur_node.g + cost
                    neigh.h = self.euclidean_distance(neigh.position, goal_node.position)
                    neigh.f = neigh.g + neigh.h
                    if neigh.f < lowest_f:
                        lowest_f = neigh.f
                        best_neighbor = neigh
                        
            if best_neighbor: 
                frontier.add(best_neighbor)

        return None

    
    def manhattan_heuristic(self,start,goal): 
        return np.abs(goal[0]-start[0]) + np.abs(goal[1]-start[1])
    
    def chebyshev_heuristic(self,start, goal):
        dx = abs(goal[0] - start[0])
        dy = abs(goal[1] - start[1])
        return max(dx, dy)


    def euclidean_distance(self,p1,p2): 
         return .5 * np.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
    

    def get_neighbors(self,node): 
        neighbors = []
        pos_x, pos_y = node.position 
        for dx in [-1,0,1]: 
            for dy in [-1,0,1]: 
                new_x, new_y = pos_x+dx, pos_y+dy 

                if self.within_grid(new_x,new_y) and (dx,dy) != (0,0): 
                    if self.grid[new_y,new_x] == 1:
                        cost = 1000
                    else: 
                        cost = 1
                    neighbors.append((Node(position=(new_x,new_y)),cost)) 
        return neighbors 

    def within_grid(self,x,y): 
        return 0<=x<self.grid.shape[1] and 0 <=y < self.grid.shape[0]

    def world_to_grid(self,pos_x,pos_y): 
        idx_x, idx_y = int((pos_x - self.x_range[0]) / self.cell_size) , int((pos_y - self.y_range[0]) / self.cell_size)
        return idx_x, idx_y
    
    def grid_to_world(self, idx_x, idx_y):
        pos_x = (idx_x * self.cell_size) + self.x_range[0]
        pos_y = (idx_y * self.cell_size) + self.y_range[0]
        return pos_x, pos_y

    def display_grid(self): 
        print(self.grid) 
    
    def visualize_results(self, path):
        print("start:",self.s_v, "goal:",self.g_v)

        plt.figure(figsize=(5,10))
        plt.grid(color='black', linewidth=0.5)
        plt.xticks(np.arange(-0.5, self.grid.shape[1], 1), [])
        plt.yticks(np.arange(-0.5, self.grid.shape[0], 1), [])

        plt.imshow(self.grid, cmap='gray_r')

        if path:
            path_x, path_y = zip(*path) 
            plt.plot(path_x, path_y, 'ro-', markersize=5.0)

        # Highlight start and goal points
        start_idx = self.world_to_grid(self.s_v[0], self.s_v[1])
        goal_idx = self.world_to_grid(self.g_v[0], self.g_v[1])
        
        plt.plot(start_idx[0], start_idx[1], 'go', markersize=8.0)  # Green color for start
        plt.plot(goal_idx[0], goal_idx[1], 'bo', markersize=8.0)    # Blue color for goal

        plt.title("A* Pathfinding Results")
        plt.subplots_adjust(left=0, bottom=0, right=1, top=1, wspace=0, hspace=0)  #remove padding
        plt.margins(0, 0)  # No margins
        plt.gca().xaxis.set_major_locator(plt.NullLocator())
        plt.gca().yaxis.set_major_locator(plt.NullLocator())
        plt.show()


