import numpy as np

grid_range = (-2, 2)
grid_size = grid_range[1] - grid_range[0] + 1
max_neighbors = 8
infinity = float('inf')

# Create the adjacency matrix filled with 'infinity'
adj_matrix = np.full((grid_size, grid_size, max_neighbors), infinity)

# Set the edge weights for neighboring nodes
for x in range(grid_range[0], grid_range[1] + 1):
    for y in range(grid_range[0], grid_range[1] + 1):
        neighbors = [(x + dx, y + dy) for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]]
        for i, (nx, ny) in enumerate(neighbors):
            if grid_range[0] <= nx <= grid_range[1] and grid_range[0] <= ny <= grid_range[1]:
                adj_matrix[x - grid_range[0], y - grid_range[0], i] = 1

print(adj_matrix.shape[0],adj_matrix.shape[2])