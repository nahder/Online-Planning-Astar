import numpy as np 
import matplotlib.pyplot as plt 
import a_star
from a_star import *


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

class IK_controller: 
    def __init__(self, astar_planner, start=None, goal=None, show_animation=True): 
    
        self.planner = astar_planner
        self.start = start #world coords
        self.goal = goal #world coords

        self.grid_path = self.planner.online_plan_path(self.start,self.goal) #grid path

        self.world_path = self.convert_path_to_world(self.grid_path) #world path

        self.grid_start = self.planner.world_to_grid(*self.start) #grid start

        self.grid_goal = self.planner.world_to_grid(*self.goal) #grid goal

        self.cur_pose = (start[0]-(self.planner.cell_size/2),start[1]-(self.planner.cell_size/2),-np.pi/2) #robot pose, world coords 
        self.cur_pose_grid = self.real_to_grid_rounded(*self.cur_pose[:2]) #robot pose, grid coords
        
        self.dt = .1
        self.max_linear_accel = 0.288
        self.max_angular_accel = 5.579

        self.prev_command = (0.0,0.0) 
        self.trajectory = [self.cur_pose]  
        self.show_animation = show_animation

        self.reached_goal = False 
        

    def motion_model(self, u, prev_state, dt):
        x_prev, y_prev, theta_prev = prev_state
        v, w = u

        sigma_v = 0.05
        sigma_w = 0.1

        v_noisy = v + np.random.normal(0, sigma_v)
        w_noisy = w + np.random.normal(0, sigma_w)

        x_new = x_prev + v_noisy * np.cos(theta_prev) * dt
        y_new = y_prev + v_noisy * np.sin(theta_prev) * dt
        theta_new = theta_prev + w_noisy * dt

        theta_new = self.wrap_angle(theta_new)

        return x_new, y_new, theta_new


    def distance(self,p1,p2): 
         return np.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
    
    def convert_path_to_world(self,path): 
        return [self.planner.grid_to_world(x,y) for x,y in path]
    
    def wrap_angle(self,angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def control_cmd(self, target_world):
        # compute movement command to go to target
        xt, yt = target_world
        x, y, heading = self.cur_pose
        angle_to_target = self.wrap_angle(np.arctan2(yt - y, xt - x))
        rel_bearing = self.wrap_angle(angle_to_target - heading)
        
        distance_error = self.distance((xt, yt), (x, y))

        kp_dist, kp_angle = .8,3.5

        v = kp_dist * distance_error  
        w = kp_angle * rel_bearing 

        linear_accel = np.clip((v - self.prev_command[0]) / self.dt, -self.max_linear_accel, self.max_linear_accel)
        angular_accel = np.clip((w - self.prev_command[1]) / self.dt, -self.max_angular_accel, self.max_angular_accel)

        if linear_accel > self.max_linear_accel or -linear_accel < -self.max_linear_accel:
            v = self.prev_command[0] + linear_accel * self.dt
        
        if angular_accel > self.max_angular_accel or -angular_accel < -self.max_angular_accel:
            w = self.prev_command[1] + angular_accel * self.dt
        
        self.prev_command = (v, w)
        return v, w
    
    def follow_waypoints(self):
        for i, target in enumerate(self.world_path):  # target in world coordinates
            while self.distance(self.cur_pose[:2], target) > .2:
                cmd = self.control_cmd(target)
                self.move_robot(cmd)
            if i == len(self.world_path) - 1:
                self.reached_goal = True
                # print("reached goal")

    def move_robot(self, command):
        new_pose = self.motion_model(command, self.cur_pose, self.dt)
        self.cur_pose = new_pose 
        self.cur_pose_grid = self.real_to_grid_rounded(*self.cur_pose[:2]) #robot pose, grid coords
        self.trajectory.append(self.cur_pose)

    def follow_waypoint(self, target_world):
        
        while self.distance(self.cur_pose[:2], target_world) > .01:
            cmd = self.control_cmd(target_world)
            self.move_robot(cmd)
        # print("waypoint reached") 
    
    def plan_while_driving(self): 
        #the online planner from a_star.py returns the whole path from start to goal.
        #we want to drive the robot while planning, so we will use the online planner to get the next waypoint

        #the online planner will give us the first node to go to. we will drive there.
        #then, we will see where we actually ended up.
        #then, we will replan using the neighbors of the grid position we actually ended up at 
        #we will repeat this process until we reach the goal
        first_target = self.world_path[0] 
        self.follow_waypoint(first_target)

        start_node = Node(self.grid_start)
        goal_node = Node(self.grid_goal)

        frontier = {start_node}
        visited = set()
        path = []

        while frontier:
            cur_node = min(frontier, key=lambda x: x.f)
            self.follow_waypoint(self.planner.grid_to_world(*cur_node.position))
            #actual position: convert cur pose to grid with real_to_grid_rounded
            visited.add(cur_node.position)
            path.append(cur_node.position)
            frontier.remove(cur_node)

            cur_node.position = self.cur_pose_grid
            
            if cur_node == goal_node:
                # print("goal reached")
                self.reached_goal = True
                return path

            neighbors = self.planner.get_neighbors(cur_node)
            best_neighbor = None
            lowest_f = float('inf')
            
            for neigh, cost in neighbors:
                if neigh.position not in visited:
                    neigh.g = cur_node.g + cost
                    neigh.h = self.planner.euclidean_distance(neigh.position, goal_node.position)
                    neigh.f = neigh.g + neigh.h
                    if neigh.f < lowest_f:
                        lowest_f = neigh.f
                        best_neighbor = neigh
                        
            if best_neighbor: 
                frontier.add(best_neighbor)
        return None


    def real_to_grid(self, x_real, y_real):
        x_min, y_min = -2, -6
        x_grid = (x_real - x_min) / self.planner.cell_size
        y_grid = (y_real - y_min) / self.planner.cell_size
        return x_grid, y_grid
    
    def real_to_grid_rounded(self, x_real, y_real):
        x_min, y_min = -2, -6
        x_grid = round((x_real - x_min) / self.planner.cell_size)
        y_grid = round((y_real - y_min) / self.planner.cell_size)
        return int(x_grid), int(y_grid)
    
    def visualize_results(self):
        plt.figure(figsize=(5, 10))
        for x, y in self.grid_path:
            self.planner.grid[y, x] = 0.35  # Set the path cells to light gray

        if self.show_animation:
            for i, pose in enumerate(self.trajectory):
                plt.clf()  # Clear the figure to remove the previous arrow
                plt.imshow(self.planner.grid, cmap='gray_r')  # Redraw the grid
                self.redraw_static_elements()  # Redraw the static elements like the A* path, start, and goal

                traj_up_to_current = self.trajectory[:i+1]
                traj_x, traj_y, _ = zip(*traj_up_to_current)
                traj_x_grid, traj_y_grid = zip(*[self.real_to_grid(x, y) for x, y in zip(traj_x, traj_y)])
                plt.plot(traj_x_grid, traj_y_grid, 'o', color='blue', markersize=1.0, zorder=2)
                self.draw_heading_arrow(pose, scale=0.2, color='magenta', zorder=3)
                plt.pause(0.02)

                plt.title("A* Pathfinding with Robot's Trajectory")

            plt.show()  # Show the animation
        else:
            plt.imshow(self.planner.grid, cmap='gray_r')  # Redraw the grid

            self.redraw_static_elements()  # Redraw the static elements

            traj_x, traj_y, _ = zip(*self.trajectory)
            traj_x_grid, traj_y_grid = zip(*[self.real_to_grid(x, y) for x, y in zip(traj_x, traj_y)])
            plt.plot(traj_x_grid, traj_y_grid, 'o', color='blue', markersize=1.0, zorder=2)

            for i, pose in enumerate(self.trajectory):
                if i % 20 == 0:  # Adjust the timestep interval as needed
                    self.draw_heading_arrow(pose, scale=0.2, color='magenta', zorder=3)

            plt.title("A* Pathfinding with Robot's Trajectory")
            plt.show()  # Show the final static plot

    def redraw_static_elements(self):
        # Redraw static elements like the A* path, start, and goal points
        path_x, path_y = zip(*self.grid_path)
        plt.plot(path_x, path_y, color='grey', marker='o', linestyle='-', markersize=5.0, zorder=1)
        start_idx = self.planner.world_to_grid(*self.start)
        goal_idx = self.planner.world_to_grid(*self.goal)
        plt.plot(start_idx[0], start_idx[1], 'o', color='red', markersize=8.0, zorder=2)
        plt.plot(goal_idx[0], goal_idx[1], 'o', color='green', markersize=8.0, zorder=2)

    def draw_heading_arrow(self, pose, scale=1.0, color='magenta', zorder=3):
        x, y, theta = pose
        dx = scale * np.cos(theta)  # Calculate the change in x
        dy = scale * np.sin(theta)  # Calculate the change in y
        x_grid, y_grid = self.real_to_grid(x, y)
        plt.arrow(x_grid, y_grid, dx, dy, head_width=0.2, head_length=0.2, fc=color, ec=color, zorder=zorder)
        

def main(): 
    a_star = A_star(1, (-2, 5), (-6, 6), 0.1)

    # set1 = [
    #     {"start": [0.5, -1.5], "goal": [0.5, 1.5]},
    #     {"start": [4.5, 3.5], "goal": [4.5, -1.5]},
    #     {"start": [-0.5, 5.5], "goal": [1.5, -3.5]}
    # ]

    # set2 = [
    #     {"start": [2.45, -3.55], "goal": [0.95, -1.55]},
    #     {"start": [4.95, -0.05], "goal": [2.45, 0.25]},
    #     {"start": [-0.55, 1.45], "goal": [1.95, 3.95]}
    # ]

    # start = (4.95,-.05)
    # goal = (2.45, 0.25)

    # start = (-.55, 1.45) 
    # goal = (1.95, 3.95) 

    start = (2.45, -3.55)
    goal = (.95, -1.55) 

    controller = IK_controller(a_star,start,goal,False) #start, goal in world coordinates 
    # controller.follow_waypoints()
    controller.plan_while_driving()

    # for target in path: 
    #     controller.follow_waypoint(controller.planner.grid_to_world(*target))
    controller.visualize_results()


if __name__ == "__main__":
    main()
