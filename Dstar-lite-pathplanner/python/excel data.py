import pandas as pd
from collections import Counter
import matplotlib.pyplot as plt
import numpy as np

# Load the CSV file
data = pd.read_csv('politebusy.csv')

coordinates = data['Robot position']

# Count the occurrences of each coordinate
coordinate_counts = Counter(coordinates)

# Get the coordinate with the highest count
most_common_coordinate = coordinate_counts.most_common(1)[0][0]

grid = np.zeros((40, 60))

#update the grid values
for coordinate, count in coordinate_counts.items():
    x, y = coordinate.strip('()').split(',')  # Remove parentheses and split by comma
    x = int(x)
    y = int(y)
    grid[39 - x, y] = count  # Invert the y-axis by subtracting from 39

plt.imshow(grid, cmap='Blues', interpolation='nearest', origin='lower')  # Use 'Blues' colormap and set origin to lower
plt.colorbar()

# Mark the coordinate with the highest count
x, y = most_common_coordinate.strip('()').split(',')
x = int(x)
y = int(y)
plt.scatter(int(y), 39 - int(x), marker='o', color='red', s=100)

plt.show()