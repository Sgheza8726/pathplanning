from dataclasses import dataclass
from typing import Iterable, List, Tuple
import math
from collections import Counter

@dataclass(frozen=True, eq=True)
class Cell:
    i: int
    j: int

class GridGraph:
    """
    Map format (first line):
        xmin ymin width height resolution
    Followed by width*height integers (commonly 0/1). Some maps invert meaning.
    This loader auto-detects which value means FREE vs OCCUPIED.
    """
    def __init__(self, map_path: str, collision_radius: float = 0.0):
        self.map_path = map_path
        with open(map_path, "r") as f:
            header = f.readline().strip().split()
            if len(header) < 5:
                raise ValueError("Bad map header (need: xmin ymin width height resolution)")
            self.xmin, self.ymin = float(header[0]), float(header[1])
            self.width, self.height = int(header[2]), int(header[3])
            self.res = float(header[4])

            vals: List[int] = []
            for line in f:
                parts = line.strip().split()
                for p in parts:
                    try:
                        vals.append(int(p))
                    except ValueError:
                        pass

        expected = self.width * self.height
        if len(vals) < expected:
            vals += [0] * (expected - len(vals))
        vals = vals[:expected]

        # Auto-detect which label is "free"
        counts = Counter(vals)
        if 0 in counts and 1 in counts:
            self.free_value = 0 if counts[0] >= counts[1] else 1
            self.occ_value = 1 - self.free_value
        else:
            # fallback: majority label = free
            self.free_value = max(counts, key=counts.get)
            self.occ_value = 0 if self.free_value != 0 else 1

        # Build 2D grid [height x width] in row-major (i=row, j=col)
        self.grid = [vals[r * self.width:(r + 1) * self.width] for r in range(self.height)]

        # Inflate obstacles by collision radius
        self.inflation = int(round(max(0.0, collision_radius) / self.res))
        if self.inflation > 0:
            self._inflate()

    def _inflate(self):
        base = [row[:] for row in self.grid]
        for r in range(self.height):
            for c in range(self.width):
                if base[r][c] == self.occ_value:
                    for dr in range(-self.inflation, self.inflation + 1):
                        for dc in range(-self.inflation, self.inflation + 1):
                            rr, cc = r + dr, c + dc
                            if 0 <= rr < self.height and 0 <= cc < self.width:
                                self.grid[rr][cc] = self.occ_value

    # --- helpers ---
    def in_bounds(self, cell: Cell) -> bool:
        return 0 <= cell.i < self.height and 0 <= cell.j < self.width

    def cell_is_free(self, cell: Cell) -> bool:
        return self.in_bounds(cell) and self.grid[cell.i][cell.j] == self.free_value

    def neighbors(self, cell: Cell) -> Iterable[Cell]:
        for di, dj in [(1,0), (-1,0), (0,1), (0,-1)]:
            n = Cell(cell.i + di, cell.j + dj)
            if self.cell_is_free(n):
                yield n

    # grid <-> metric
    def cell_to_pos(self, i: int, j: int) -> Tuple[float, float]:
        x = self.xmin + (j + 0.5) * self.res
        y = self.ymin + (i + 0.5) * self.res
        return (x, y)

    def pos_to_cell(self, x: float, y: float) -> Cell:
        j = int(math.floor((x - self.xmin) / self.res))
        i = int(math.floor((y - self.ymin) / self.res))
        return Cell(i, j)
