import matplotlib.pyplot as plt
import random
import Global_tomerge as global_nav
from matplotlib.patches import Polygon

def plot_expended_obstacles(object_corners, SandG, expended_corners):
    point_names = global_nav.name2coord(expended_corners, SandG)
    point_names2 = global_nav.name2coord(object_corners, SandG)
    point_names.pop('S', None)
    point_names.pop('G', None)
    point_names2.pop('S', None)
    point_names2.pop('G', None)
    plt.figure(figsize=(8, 8))

    # Merge both dictionaries while skipping the 'S' and 'G' keys
    i = 0
    for pt, coord in point_names2.items():
        point_names[f"P{i}_init"] = coord
        i += 1
        
    # Extract shapes formed by 4 consecutive points
    shapes = [list(point_names.values())[i:i+4] for i in range(0, len(point_names) - 3, 4)]
    
    # Create a dictionary to map shapes to unique colors
    color_dict = {}
    for shape in shapes:
        if str(shape) not in color_dict:
            color_dict[str(shape)] = plt.cm.get_cmap('tab10')(random.random())

    # Plot the shapes with assigned colors
    for shape in shapes:
        plt.gca().add_patch(Polygon(shape, closed=True, fill=True, facecolor=color_dict[str(shape)], edgecolor='black'))

    # Plot the points with their names
    x_values = [point[0] for point in point_names.values()]
    y_values = [point[1] for point in point_names.values()]
    plt.scatter(x_values, y_values)
    for point_name, point_coords in point_names.items():
        plt.text(point_coords[0], point_coords[1], point_name, ha='right', va='bottom')

    # Set plot labels and title
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Expended Obstacles')

    # Show plot
    plt.grid(True)
    plt.show()