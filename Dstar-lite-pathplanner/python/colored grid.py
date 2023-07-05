import pandas as pd
from collections import Counter
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap



file_paths = ['nono.csv', 'politebusy.csv', 'politenotbusy.csv', 'notpolitebusy.csv']
# Calculate the number of rows and columns
num_files = len(file_paths)
num_rows = int(np.ceil(np.sqrt(num_files)))
num_cols = int(np.ceil(num_files / num_rows))

# Find the maximum count all files
max_count = 0
for file_path in file_paths:
    data = pd.read_csv(file_path)
    counts = Counter(data['Robot position'])
    max_count = max(max_count, max(counts.values()))

# Create a custom colormap from white to blue to black
colors = [(1, 1, 1), (0, 0, 1), (0, 0, 0)]
cmap = LinearSegmentedColormap.from_list('white_blue_black', colors)

# Create a figure with subplots
fig, axs = plt.subplots(num_rows, num_cols, figsize=(10, 10))

for i, (file_path, ax) in enumerate(zip(file_paths, axs.flat)):
    coordinate_counts = {}

    # Load the CSV file into a DataFrame
    data = pd.read_csv(file_path)

    #Extract the coordinates column
    coordinates = data['Robot position']

    #Count the occurrences of each coordinate
    coordinate_counts = Counter(coordinates)

    grid = np.zeros((40, 60))

    for coordinate, count in coordinate_counts.items():
        x, y = coordinate.strip('()').split(',')  # Remove parentheses and split by comma
        x = int(x)
        y = int(y)
        grid[39 - x, y] = count  # Invert the y-axis by subtracting from 39

    # Plot the grid in the current subplot with the custom color scale
    im = ax.imshow(grid, cmap=cmap, interpolation='nearest', origin='lower', vmin=0, vmax=max_count)  # Use custom colormap and set origin to lower
    fig.colorbar(im, ax=ax)

fig.tight_layout()
plt.show()