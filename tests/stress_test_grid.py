import time
import random
from collections import defaultdict

from w9_pathfinding import Grid, BFS, Dijkstra, AStar

SHORTEST_PATH_ALGORITHMS = [AStar, BFS, Dijkstra]


def create_random_map(width, height, obstacle_percentage):
    obstacle_map = []
    for _ in range(height):
        row = [random.random() < obstacle_percentage for x in range(width)]
        obstacle_map.append(row) 
    
    return obstacle_map


def create_random_grid(width, height, obstacle_percentage):
    obstacle_map = create_random_map(width, height, obstacle_percentage)
    grid = Grid(obstacle_map)
    grid.diagonal_movement = random.randint(0, 3)
    grid.passable_left_right_border = random.randint(0, 1)
    grid.passable_up_down_border = random.randint(0, 1)
    return grid


def find_path(finder, start, end):
    t = time.time()
    path = finder.find_path(start, end)
    return path, time.time() - t


def find_free_point(grid):
    while True:
        x = random.randint(0, grid.width - 1)
        y = random.randint(0, grid.height - 1)
        if not grid.has_obstacle(x, y):
            return x, y


def check_path(grid, path, start, end):
    if not path:
        return True
    
    if path[0] != start or path[-1] != end:
        return False

    for i in range(len(path) - 1):
        p = path[i]
        next_p = path[i + 1]
        neighbours = grid.get_neighbours(*p)
        if next_p not in neighbours:
            return False

    return True


def show_grid_info(grid, start, end):
    print("obstacle_map =", grid.obstacle_map)
    print("diagonal_movement =", grid.diagonal_movement)
    print("passable_left_right_border =", grid.passable_left_right_border)
    print("passable_up_down_border =", grid.passable_up_down_border)
    print(f"start, end = {start}, {end}")


def run_random_grid(width=12, height=8, obstacle_percentage=0.3, num_runs=1, timer=None):
    grid = create_random_grid(width, height, obstacle_percentage)
    algrithms = [(a.__name__, a(grid)) for a in SHORTEST_PATH_ALGORITHMS] 

    for _ in range(num_runs):
        
        start = find_free_point(grid)
        end = find_free_point(grid)

        results = []
        for alg_name, finder in algrithms:
            path, time = find_path(finder, start, end)
            if timer is not None:
                timer[alg_name] += time

            if not check_path(grid, path, start, end):
                print(f"Error algorithm {alg_name} return wrong path {path}")
                grid.show_path(path)
                show_grid_info(grid, start, end)
                return False

            results.append((alg_name, path))

        if len({len(x[1]) for x in results}) > 1:
            print("Error, different results: ")
            for alg_name, path in results:
                print(f" - {alg_name} : {len(path)}, {path}")
                grid.show_path(path)
            show_grid_info(grid, start, end)
            return False
        
    return True


def stress_test():
    num_tests = 100

    timer = defaultdict(int)
    
    for i in range(num_tests):
        print(f"run {i + 1}/{num_tests}")
        r = run_random_grid(
            width=100,
            height=100,
            obstacle_percentage=0.2,
            num_runs=10,
            timer=timer
        )
        if not r:
            break

    print("Total time:")
    for alg_name, time in timer.items():
        print(f" - {alg_name} {time}")


if __name__ == "__main__":
    stress_test()
