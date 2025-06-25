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



# Visualization
def visualize(grid, path, start, goal):
    img = np.array(grid)
    img = np.where(img == 1, 100, 255)

    fig, ax = plt.subplots()
    ax.imshow(img, cmap='gray')

    if path:
        px, py = zip(*path)
        ax.plot(py, px, color='blue', linewidth=2, label="Path")

    ax.scatter(start[1], start[0], c='green', s=100, label='Start', marker='o')
    ax.scatter(goal[1], goal[0], c='red', s=100, label='Goal', marker='X')

    ax.set_xticks(np.arange(-0.5, len(grid[0]), 1))
    ax.set_yticks(np.arange(-0.5, len(grid), 1))
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.grid(True, color='black', linewidth=0.5)
    ax.legend(loc='upper right')
    plt.title("A* Pathfinding - Room Layout")
    plt.show()




# Room Definitions
grid_room1 = [
    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    [1, 1, 0, 1, 1, 0, 1, 1, 1, 0],
    [0, 1, 0, 0, 0, 0, 0, 0, 1, 0],
    [0, 1, 1, 1, 1, 1, 1, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 1, 0, 1, 0],
    [0, 1, 1, 1, 1, 0, 1, 0, 1, 0],
    [0, 1, 0, 0, 0, 0, 0, 0, 1, 0],
    [0, 1, 0, 1, 1, 1, 1, 1, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
]

grid_room2 = [
    [0, 0, 1, 0, 0, 0],
    [0, 1, 1, 1, 1, 0],
    [0, 1, 0, 0, 1, 0],
    [0, 1, 0, 1, 1, 0],
    [0, 0, 0, 1, 0, 0],
    [1, 1, 0, 0, 0, 0],
]

grid_room3 = [
    [0, 0, 0, 1, 0, 0, 0],
    [1, 1, 0, 1, 0, 1, 0],
    [0, 1, 0, 0, 0, 1, 0],
    [0, 1, 1, 1, 1, 1, 0],
    [0, 0, 0, 0, 0, 0, 0],
]

# Room selection
room_options = {
    "1": grid_room1,
    "2": grid_room2,
    "3": grid_room3
}

print("\nChoose a room layout:")
print("1. Room 1 (10x10)")
print("2. Room 2 (6x6)")
print("3. Room 3 (5x7)")
room_choice = input("Enter room number (1/2/3): ")

grid = room_options.get(room_choice)
if not grid:
    print("Invalid choice. Exiting.")
    exit()

rows, cols = len(grid), len(grid[0])
print(f"Grid size: {rows} rows x {cols} cols")

# Get start and goal from user
try:
    start_x = int(input("Enter START X (0-based row): "))
    start_y = int(input("Enter START Y (0-based col): "))
    goal_x = int(input("Enter GOAL X (0-based row): "))
    goal_y = int(input("Enter GOAL Y (0-based col): "))

    start = (start_x, start_y)
    goal = (goal_x, goal_y)

    if grid[start_x][start_y] == 1 or grid[goal_x][goal_y] == 1:
        print("Start or goal is on a wall. Exiting.")
        exit()

    path = astar(grid, start, goal)

    if path:
        print("✅ Path found:")
        print(path)
    else:
        print("❌ No path found.")

    
    visualize(grid, path, start, goal)


except Exception as e:
    print("Error:",e)

