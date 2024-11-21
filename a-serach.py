import heapq
import matplotlib.pyplot as plt
import numpy as np


# Define the A* algorithm
def a_star(grid, start, goal):
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    #possible moves:
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    open_list = []
    heapq.heappush(open_list, (0, start))

    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    came_from = {}

    while open_list:
        _, current = heapq.heappop(open_list)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        #find neighbours
        for direction in directions:
            neighbor = (current[0] + direction[0], current[1] + direction[1])

            if 0 <= neighbor[0] < len(grid) and 0 <= neighbor[1] < len(grid[0]) and grid[neighbor[0]][neighbor[1]] == 0:
                tentative_g_score = g_score[current] + 1

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))

    return None


# Visualization function with standard colors
def visualize_path(grid, path, start, goal):
    grid = np.array(grid)
    for (x, y) in path:
        grid[x, y] = 2  # Mark the path
    grid[start[0], start[1]] = 3  # Mark the start
    grid[goal[0], goal[1]] = 4  # Mark the goal

    # Define a custom color map
    cmap = plt.cm.get_cmap('Accent', 5)
    bounds = [0, 1, 2, 3, 4]
    norm = plt.Normalize(vmin=0, vmax=4)

    plt.imshow(grid, cmap=cmap, norm=norm)
    plt.colorbar(ticks=[0, 1, 2, 3, 4],
                 format=plt.FuncFormatter(lambda val, loc: ['Empty', 'Obstacle', 'Path', 'Start', 'Goal'][int(val)]))
    plt.title("A* Pathfinding Visualization")
    plt.show()


#Define a grid
grid = [[8, 3, 2],
        [4, 0, 5],
        [6, 7, 1]]

#Define start and goal positions
start = (range(9, 9))
goal = [[0, 1, 2],
        [3, 4, 5],
        [6, 7, 8]]

# Run the A* algorithm
path = a_star(grid, start, goal)

if path:
    print("Path found:", path)
    visualize_path(grid, path, start, goal)
else:
    print("No path found")