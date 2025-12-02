import json
from typing import List, Optional
from .graph import GridGraph, Cell

def _to_pair(c: Cell) -> list:
    return [int(c.i), int(c.j)]

def generate_plan_file(graph: GridGraph,
                       start: Cell,
                       goal: Cell,
                       path: List[Cell],
                       visited: Optional[List[Cell]] = None,
                       out_path: str = "out.planner") -> None:
    payload = {
        "start": _to_pair(start),
        "goal": _to_pair(goal),
        "path": [_to_pair(c) for c in (path or [])],
        "visited": [_to_pair(c) for c in (visited or [])]
    }
    with open(out_path, "w") as f:
        json.dump(payload, f)
