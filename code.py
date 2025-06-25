import heapq
import matplotlib.pyplot as plt
import numpy as np


def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# A* algorithm
def astar(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal), 0, start, [start]))
    visited = set()

    while open_set:
        _, cost, current, path = heapq.heappop(open_set)
        if current in visited:
            continue
        visited.add(current)
        if current == goal:
            return path

        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            nx, ny = current[0] + dx, current[1] + dy
            neighbor = (nx, ny)
            if 0 <= nx < rows and 0 <= ny < cols:
                if grid[nx][ny] == 0 and neighbor not in visited:
                    heapq.heappush(open_set, (
                        cost + 1 + heuristic(neighbor, goal),
                        cost + 1,
                        neighbor,
                        path + [neighbor]
                    ))
    return None
