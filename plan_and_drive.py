from controller import IK_controller
from a_star import A_star 
import numpy as np


'''
Putting it all together: Plan the paths while driving them. That is, no longer use your paths from Step 7 (which
assumed perfect world dynamics). Instead, at each step of your online planning, execute the next chosen step
using your inverse kinematic controller. Provide a visual display of your results, that shows the occupied cells,
planned paths, robot position, and robot heading. Discuss any challenges, inconsistent or surprising behavior.
'''

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
        
def main():
    planner = A_star(1, (-2, 5), (-6, 6), 0.1)
    set1 = [
    {"start": [0.5, -1.5], "goal": [0.5, 1.5]},
    {"start": [4.5, 3.5], "goal": [4.5, -1.5]},
    {"start": [-0.5, 5.5], "goal": [1.5, -3.5]}
    ]

    set2 = [
    {"start": [2.45, -3.55], "goal": [0.95, -1.55]},
    {"start": [4.95, -0.05], "goal": [2.45, 0.25]},
    {"start": [-0.55, 1.45], "goal": [1.95, 3.95]}
    ]

    start = (-.5, 5.5)
    goal = (1.5, -3.5)
    controller = IK_controller(astar_planner=planner,start=start,goal=goal,
                            show_animation=False) #start, goal in world coordinates 
    plan_while_driving(planner, controller, start, goal)

if __name__ == "__main__":
    main() 