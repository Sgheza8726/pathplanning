import argparse
from src.graph import GridGraph, Cell
from src.graph_search import a_star_search, breadth_first_search, depth_first_search, \
                             astar_with_visited, bfs_with_visited, dfs_with_visited
from src.utils import generate_plan_file

def parse_args():
    p = argparse.ArgumentParser(description="HelloRob Path Planning Client.")
    p.add_argument("-m", "--map", type=str, required=True, help="Path to the map file.")
    p.add_argument("--start", type=int, nargs=2, required=True, help="Start cell (i j).")
    p.add_argument("--goal", type=int, nargs=2, required=True, help="Goal cell (i j).")
    p.add_argument("--algo", type=str, default="bfs", choices=["bfs", "dfs", "astar"], help="Algorithm.")
    p.add_argument("-r", "--collision-radius", type=float, default=0.0, help="Collision radius (meters).")
    return p.parse_args()

if __name__ == "__main__":
    args = parse_args()

    # Build graph (with collision inflation)
    graph = GridGraph(args.map, collision_radius=args.collision_radius)
    start, goal = Cell(*args.start), Cell(*args.goal)

    # Run chosen algorithm and collect visited for visualization
    if args.algo == "astar":
        path, visited = astar_with_visited(graph, start, goal)
    elif args.algo == "bfs":
        path, visited = bfs_with_visited(graph, start, goal)
    else:
        path, visited = dfs_with_visited(graph, start, goal)

    # Write the EXACT JSON the web app expects
    generate_plan_file(graph, start, goal, path, visited)

    # Friendly console output
    name = args.algo.upper()
    print(f"[{name}] path length: {len(path)} -> out.planner")
