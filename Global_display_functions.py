import matplotlib.pyplot as plt
import random
import Global_tomerge as global_nav
from matplotlib.patches import Polygon
import math

def plot_expended_obstacles(object_corners, SandG, expended_corners):
    point_names = global_nav.name2coord(expended_corners, SandG)
    point_names2 = global_nav.name2coord(object_corners, SandG)
    point_names.pop('R', None)
    point_names.pop('G', None)
    point_names2.pop('R', None)
    point_names2.pop('G', None)
    plt.figure(figsize=(8, 8))

    # Merge both dictionaries and skips  'R' and 'G' 
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





def display_segments(p1, p2, p3, p4):

    # Extracting coordinates for the segments
    x_values_segment1 = [p1[0], p2[0]]
    y_values_segment1 = [p1[1], p2[1]]

    x_values_segment2 = [p3[0], p4[0]]
    y_values_segment2 = [p3[1], p4[1]]

    # Create a plot
    plt.figure(figsize=(6, 6))

    # Plotting the segments
    plt.plot(x_values_segment1, y_values_segment1, label='Segment 1', marker='o')
    plt.plot(x_values_segment2, y_values_segment2, label='Segment 2', marker='o')

    # Set plot labels and title
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Displaying Two Segments on a Grid')

    # Add legend
    plt.legend()

    # Show grid
    plt.grid(True)

    # Show plot
    plt.show()


def plot_visibility_graph(adjacency_list, point_names):
    plt.figure(figsize=(8, 8))

    # Plotting points with their names
    for point_name, coordinates in point_names.items():
        plt.scatter(coordinates[0], coordinates[1], color='blue', s=100)
        plt.text(coordinates[0], coordinates[1], point_name, color='black', ha='right', va='bottom')

    # Connecting points based on adjacency list
    for point, connected_points in adjacency_list.items():
        for connected_point in connected_points:
            # Get coordinates for the connecting points
            x_values = [point_names[point][0], point_names[connected_point][0]]
            y_values = [point_names[point][1], point_names[connected_point][1]]

            # Plot line between connected points
            plt.plot(x_values, y_values, color='gray')

    # Set plot labels and title
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Visibility Graph')

    # Show grid
    plt.grid(True)

    # Show plot
    plt.show()


def plot_graph_with_shapes(adjacent_list, point_names, distances, shortest_path, object_edges):
    plt.figure(figsize=(8, 8))
     
    # Plotting all points and their names
    for point, coordinates in point_names.items():
        plt.scatter(coordinates[0], coordinates[1], color='blue', s=100)
        plt.text(coordinates[0], coordinates[1], point, ha='right')

    # Plotting edges with distances
    for start_point, connected_points in adjacent_list.items():
        for end_point in connected_points:
            if (start_point, end_point) in distances:
                distance = distances[(start_point, end_point)]
                start_coord = point_names[start_point]
                end_coord = point_names[end_point]
                plt.plot([start_coord[0], end_coord[0]], [start_coord[1], end_coord[1]], 'k-')
                plt.text((start_coord[0] + end_coord[0]) / 2, (start_coord[1] + end_coord[1]) / 2, f'{distance:.2f}', color='black')

    # Plotting shapes
    for obj_id, points in object_edges.items():
        if isinstance(points, list):
            # Edge points: List of tuples
            x_values = [point[0] for point in points]
            y_values = [point[1] for point in points]

            # Check if the first and last points are the same to close the shape
            if points[0] != points[-1]:
                points.append(points[0])  # Append the first point at the end to close the shape

            # Update x and y values after closing the shape
            x_values = [point[0] for point in points]
            y_values = [point[1] for point in points]

            # Plot the points representing the edges of the object
            plt.plot(x_values, y_values, label=f'Shape {obj_id}', marker='o')

            # Fill the shape defined by the points for each object
            plt.fill(x_values, y_values, alpha=0.3)
        
    # Highlighting shortest path in red
    for i in range(len(shortest_path) - 1):
        start_point = point_names[shortest_path[i]]
        end_point = point_names[shortest_path[i + 1]]
        plt.plot([start_point[0], end_point[0]], [start_point[1], end_point[1]], 'r-', linewidth=2)


    # Set plot labels and title
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Graph Visualization with Shapes')

    # Add legend
    plt.legend()

    # Show grid
    plt.grid(True)

    # Show plot
    plt.show()
