from typing import Dict, List, Tuple
from collections import deque
import heapq
from .graph import GridGraph, Cell

def _reconstruct(came_from: Dict[Cell, Cell], start: Cell, goal: Cell) -> List[Cell]:
    if goal not in came_from and goal != start:
        return []
    cur = goal
    path = [cur]
    while cur != start:
        cur = came_from[cur]
        path.append(cur)
    path.reverse()
    return path

# -------- BFS --------
def bfs_with_visited(g: GridGraph, start: Cell, goal: Cell) -> Tuple[List[Cell], List[Cell]]:
    if not g.cell_is_free(start) or not g.cell_is_free(goal):
        return [], []
    q = deque([start])
    came_from: Dict[Cell, Cell] = {}
    visited_set = {start}
    visited_order: List[Cell] = [start]
    while q:
        v = q.popleft()
        if v == goal:
            break
        for n in g.neighbors(v):
            if n not in visited_set:
                visited_set.add(n)
                visited_order.append(n)
                came_from[n] = v
                q.append(n)
    return _reconstruct(came_from, start, goal), visited_order

# -------- DFS --------
def dfs_with_visited(g: GridGraph, start: Cell, goal: Cell) -> Tuple[List[Cell], List[Cell]]:
    if not g.cell_is_free(start) or not g.cell_is_free(goal):
        return [], []
    stack = [start]
    came_from: Dict[Cell, Cell] = {}
    visited_set = {start}
    visited_order: List[Cell] = [start]
    while stack:
        v = stack.pop()
        if v == goal:
            break
        for n in g.neighbors(v):
            if n not in visited_set:
                visited_set.add(n)
                visited_order.append(n)
                came_from[n] = v
                stack.append(n)
    return _reconstruct(came_from, start, goal), visited_order

def _manhattan(a: Cell, b: Cell) -> int:
    return abs(a.i - b.i) + abs(a.j - b.j)

# -------- A* --------
def astar_with_visited(g: GridGraph, start: Cell, goal: Cell) -> Tuple[List[Cell], List[Cell]]:
    if not g.cell_is_free(start) or not g.cell_is_free(goal):
        return [], []
    pq: List[Tuple[int,int,int,Cell]] = []  # (f, h, tie, node)
    came_from: Dict[Cell, Cell] = {}
    g_cost: Dict[Cell, int] = {start: 0}
    tie = 0
    h0 = _manhattan(start, goal)
    heapq.heappush(pq, (h0, h0, tie, start))
    closed = set()
    visited_order: List[Cell] = []

    while pq:
        _, _, _, v = heapq.heappop(pq)
        if v in closed:
            continue
        closed.add(v)
        visited_order.append(v)
        if v == goal:
            break
        for n in g.neighbors(v):
            new_g = g_cost[v] + 1
            if n not in g_cost or new_g < g_cost[n]:
                g_cost[n] = new_g
                tie += 1
                h = _manhattan(n, goal)
                heapq.heappush(pq, (new_g + h, h, tie, n))
                came_from[n] = v

    return _reconstruct(came_from, start, goal), visited_order

# Backwards-compatible simple APIs
def breadth_first_search(g: GridGraph, start: Cell, goal: Cell) -> List[Cell]:
    path, _ = bfs_with_visited(g, start, goal)
    return path

def depth_first_search(g: GridGraph, start: Cell, goal: Cell) -> List[Cell]:
    path, _ = dfs_with_visited(g, start, goal)
    return path

def a_star_search(g: GridGraph, start: Cell, goal: Cell) -> List[Cell]:
    path, _ = astar_with_visited(g, start, goal)
    return path
