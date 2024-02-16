from a_star import A_star
from controller import IK_controller

def run_test(set, algorithm, a_star):
    a_star.generate_grid()
    for row in set:
        if algorithm == "online":
            path = a_star.online_plan_path(row["start"], row["goal"])

        elif algorithm == "offline":
            path = a_star.plan_path(row["start"], row["goal"])

        a_star.visualize_results(path)

def main():
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

    #3 plan paths from set 1 with offline A* 
    # a_star = A_star(1, (-2, 5), (-6, 6), 1.0)
    # run_test(set1,"offline",a_star) 

    #5: plan paths from set 1 with online A*
    # a_star = A_star(1, (-2, 5), (-6, 6), 1.0)
    # run_test(set1,"online",a_star)

    #7: plan paths on set2 with online A*
    # a_star = A_star(1, (-2, 5), (-6, 6), 0.1)
    # run_test(set2,"online",a_star)

    #9: drive paths generated in 7 (set2) with IK controller
    # for path in set2:
    #     start = path["start"]
    #     goal = path["goal"]
    #     planner = A_star(1, (-2, 5), (-6, 6), 0.1)
    #     controller = IK_controller(astar_planner=planner, start=start, goal=goal, show_animation=False)
    #     controller.follow_waypoints()
    #     controller.visualize_results()

    # 10: plan set2 paths while driving them
    # for path in set2:
    #     start = path["start"]
    #     goal = path["goal"]
    #     planner = A_star(1, (-2, 5), (-6, 6), 0.1)
    #     controller = IK_controller(astar_planner=planner, start=start, goal=goal, show_animation=False)
    #     controller.plan_while_driving()
    #     controller.visualize_results()

    #11: plan while driving, set 1, on the finer grid
    for path in set1:
        start = path["start"]
        goal = path["goal"]
        planner = A_star(1, (-2, 5), (-6, 6), 1.0)
        controller = IK_controller(astar_planner=planner, start=start, goal=goal, show_animation=True)
        controller.plan_while_driving()
        controller.visualize_results()

    #11b: plan while driving set 1, on the coarser grid 
    for path in set1:
        start = path["start"]
        goal = path["goal"]
        planner = A_star(1, (-2, 5), (-6, 6), 0.1)
        controller = IK_controller(astar_planner=planner, start=start, goal=goal, show_animation=False)
        controller.plan_while_driving()
        controller.visualize_results()

if __name__ == "__main__":
    main()
