from a_star import A_star

def run_test(set, algorithm):
    a_star = A_star(1, (-2, 5), (-6, 6), .01)
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

    test_set = "set2"  
    algorithm = "online"  

    if test_set == "set1":
        run_test(set1, algorithm)
    elif test_set == "set2":
        run_test(set2, algorithm)

if __name__ == "__main__":
    main()
