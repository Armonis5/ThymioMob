import random
import math
from math import sqrt

#ALL FUNCTIONS

############################################################################################################
#Formatage des données de la caméra

#Takes points given by camera and creates a dictionnary with object 
#name as key associated to a list of points
def create_dictionnary(objects):
    object_dict = {}
    for i, pts_list in enumerate(objects):
        if len(pts_list) == 4:
            object_dict[f'object_{i+1}'] = pts_list
    return object_dict

#Takes goal and start position coordinates from camera and creates
#a dictionnary with object
def create_RandG_dict(robot, goal):
    RandG = {
        'robot': robot,
        'goal': goal
    }
    return RandG


#To use in grow_obsracles function
#It calculates angle between point p1 and p2 and finds the distance
#we should move the points in x and y
def get_delta(p1, p2, size_robot):
    alpha = math.atan2((p1[1]-p2[1]),(p1[0]-p2[0]))
    deltax= abs(size_robot*math.cos(alpha))
    deltay= abs(size_robot*math.sin(alpha))
    return deltax, deltay

#Grow obstacles along diagonals to avoid collision with the robot
def grow_obstacles(start_obj, size_robot):
    # Calculate the adjusted coordinates for each vertex
    expended_object = {}

    for obj_id, points in start_obj.items():
        expended_object[obj_id] = points[:]
        for i in range(0, 2):
            x1, y1 = points[i]
            x2, y2 = points[i+2]

            #first point on top left from second point
            if (x1<x2 and y1>y2):
                p1=(x1,y1)
                p2=(x2,y2)
                case=1
            #second point on top left from first point
            elif (x2<x1 and y2>y1):
                p1=(x2,y2)
                p2=(x1,y1)
                case=1
            #first point on top right from second point
            elif (x1>x2 and y1>y2):
                p1=(x1,y1)
                p2=(x2,y2)
                case=2
            #second point on top right from first point
            elif (x2>x1 and y2>y1):
                p1=(x2,y2)
                p2=(x1,y1)
                case=2
            #first above second
            elif (x2==x1 and y1>y2):
                p1=(x1,y1)
                p2=(x2,y2)
                case=3
            #second above first
            elif (x2==x1 and y2>y1):
                p1=(x2,y2)
                p2=(x1,y1)
                case=3
            #seconde on left of first
            elif (x2>x1 and y2==y1):
                p1=(x2,y2)
                p2=(x1,y1)
                case=4
            #first on left of second
            elif (x1>x2 and y2==y1):
                p1=(x1,y1)
                p2=(x2,y2)
                case=4
            else:
                print("error")

            deltax, deltay=get_delta(p1,p2,size_robot)
    
            if case==1:
                p1=(p1[0]-deltax,p1[1]+deltay)
                p2=(p2[0]+deltax,p2[1]-deltay)

            elif case==2:
                p1=(p1[0]+deltax,p1[1]+deltay)
                p2=(p2[0]-deltax,p2[1]-deltay)

            elif case==3:
                p1=(p1[0],p1[1]+deltay)
                p2=(p2[0],p2[1]-deltay)

            elif case==4:
                p1=(p1[0]+deltax,p1[1])
                p2=(p2[0]-deltax,p2[1])

            else:
                print("error")

            expended_object[obj_id][i]=p1
            expended_object[obj_id][i+2]=p2

    return expended_object


#############################################################################################################
#From the list of point I need to establish which points are connected to each other.
#The points are connected if they don't cross a line connecting points from the same object.

#function that defines the orientation of three given points
def compute_orientation(p1,p2,p3):
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    orientation_value = (y2-y1)*(x3-x2)-(y3-y2)*(x2-x1)
    # Check for collinear, clockwise, or counterclockwise orientation
    if orientation_value == 0:
        return 0  # Collinear
    elif orientation_value > 0:
        return -1  # Clockwise orientation
    else:
        return 1  # Counterclockwise orientation

#For 3 collinear points a,b,c, we search if point c lies on segment ab    
def onSegment(a, b, c): 
    xa, ya = a
    xb, yb = b
    xc, yc = c
    if ( (xc <= max(xa, xb)) and (xc >= min(xa, yb)) and 
           (yc <= max(ya, yb)) and (yc >= min(ya, yb))): 
        return True
    return False

#function that checks if two line segments intersect
def intersect(p1,q1,p2,q2):
    xp1, yp1 = p1
    xq1, yq1 = q1
    xp2, yp2 = p2
    xq2, yq2 = p2

    o1=compute_orientation(p1,q1,p2)
    o2=compute_orientation(p1,q1,q2)
    o3=compute_orientation(p2,q2,p1)
    o4=compute_orientation(p2,q2,q1)

    #If two points are equal, we return false, they are crossing on that point 
    #which is not considered here as an intersection.
    #This situation for points in the same object is handle in is_connected function.
    if p1==p2 or p1==q2 or q1==p2 or q1==q2:
        return False
    
    # General case: If orientation of point 1 and 2 are different 
    # and orientation of point 3 and 4 are different, then the points intersect
    if o1 != o2 and o3 != o4:
        return True
    
    # p1 , q1 and p2 are collinear and p2 lies on segment p1q1 
    if ((o1 == 0) and onSegment(p1, q1, p2)): 
        return True
  
    # p1 , q1 and q2 are collinear and q2 lies on segment p1q1 
    if ((o2 == 0) and onSegment(p1, q1, q2)): 
        return True
  
    # p2 , q2 and p1 are collinear and p1 lies on segment p2q2 
    if ((o3 == 0) and onSegment(p2, q2, p1)): 
        return True
  
    # p2 , q2 and q1 are collinear and q1 lies on segment p2q2 
    if ((o4 == 0) and onSegment(p2, q2, q1)): 
        return True
    return False 

#############################################################################################################
#Handle data for better lisibility

#Give a name to avery point. Creates a dictionnary with names as key associated to point coordinates
def name2coord(object_edges, GandS):
    point_names = {}
    j = 0

    for obj_id, points_list in object_edges.items():
        for point in points_list:
            point_names[f'P{j}'] = point
            j += 1
    point_names['R']=GandS['robot']
    point_names['G']=GandS['goal']
    return point_names

#Creates a dictionnary with object name as key associated to a list of points 
#names belonging to object
def object_ptsname(object_edges):
    point_names = {}  # Dictionary to map points to unique names
    j = 0 

    for obj_id, points_list in object_edges.items():
        point_names[obj_id] = []
        for _ in points_list:
            point_names[obj_id].append(f'P{j}')
            j += 1

    return point_names

#############################################################################################################
#Finding the shortest path

#Check if two points are connected
def is_connected(point1, point2, expended_corners, RandG, object_corners):
    point_names = name2coord(expended_corners, RandG)
    coordinate_to_name = {v: k for k, v in point_names.items()} #New dictionary with coordinates as key and point name as value
    point_objects = object_ptsname(expended_corners)
    
    #if point1 and point2 are in same object but not adjacent return  false
    P1= coordinate_to_name[point1] #P1 is the name of the point1
    P2= coordinate_to_name[point2] #P2 is the name of the point2
    for object, points in point_objects.items():
        if P1 in points and P2 in points:
            #If the points are adjacent, return true
            if abs(points.index(P1)-points.index(P2))==1:
                return True
            #If points are adjacent with one at the end and the other at the beginning of the list
            if points.index(P1)==len(points)-1 and points.index(P2)==0:
                return True
            if points.index(P2)==len(points)-1 and points.index(P1)==0:
                return True
            #If points in same object not adjacent, return false
            return False

    #We go through all object and check if the line between point1 and point2 
    #intersect with any of the vertices of an object
    for object, points in expended_corners.items():
        for j in range(len(points)):
            if j == (len(points) - 1):
                #intersection with last vertice
                if intersect(point1, point2, points[j], points[0]):
                    return False
            else:
                #We skip the points if they are equal to point1 or point2
                if points[j] == point1 or points[j+1] == point2:
                    continue
                    
                else:
                    #intersection with other vertices
                    if intersect(point1, point2, points[j], points[j + 1]):
                        return False

    inside = False
    for obj_id, corners in expended_corners.items():
        if is_point_inside_quad(RandG['robot'], corners):    
            inside = True
              
    if (P1 == 'R' or P2 == 'R') and inside:
        for object, points in object_corners.items():
            for j in range(len(points)):
                if j == (len(points) - 1):
                    #intersection with last vertice
                    if intersect(point1, point2, points[j], points[0]):
                        return False
                else:
                    #We skip the points if they are equal to point1 or point2
                    if points[j] == point1 or points[j+1] == point2:
                        continue
                        
                    else:
                        #intersection with other vertices
                        if intersect(point1, point2, points[j], points[j + 1]):
                            return False
    return True




#Creates a dictionnary with point name as key associated to a list of points names connected to it
def generate_adjacency_list(object_edges, RandG, object_corners):
    point_names = name2coord(object_edges, RandG)
    adjacency_list = {}

    for P1, coord1 in point_names.items():
        adjacency_list[P1] = []
        for P2, coord2 in point_names.items():
            if P1 != P2:
                if is_connected(coord1, coord2, object_edges, RandG, object_corners):
                    adjacency_list[P1].append(P2)
    return adjacency_list



#Eucledian distance
def compute_dist(point1, point2):
    return sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)


#Creates a dictionnary with 2 connected points as key associated with their distance
def calculate_distances(adjacency_list, point_names):
    distances = {}  # Dictionary to store distances between connected points

    for point, connected_points in adjacency_list.items():
        for connected_point in connected_points:
            # Calculate distance using Euclidean distance formula
            distance = compute_dist(point_names[point], point_names[connected_point])
            
            # Store the distance in the distances dictionary
            distances[(point, connected_point)] = distance
            distances[(connected_point, point)] = distance
    
    return distances

#Finds distance stored in dictionnary distances between two points
def get_distance(distances, point1, point2):
    for points, dist in distances.items():
        if points[0] == point1 and points[1] == point2:
            return dist


def dijkstra(adjacency_list, point_names):
    shortest_dist = {} #store the best-known cost of visiting each point in the graph starting from start
    previous_nodes = {} #store the previous node of the current best known path for each node
    unvisited_nodes = list(point_names.keys())
    distances = calculate_distances(adjacency_list, point_names)
    # We need to set every distance to infinity. We will simulate that using a very large value     
    infinity = 10e10
    for node in point_names.keys():
        shortest_dist[node] = infinity
    shortest_dist['R'] = 0

    while unvisited_nodes:
        current_min_node = unvisited_nodes[0]
        for node in unvisited_nodes: # Iterate over the nodes
            if shortest_dist[node] < shortest_dist[current_min_node]:
                current_min_node = node
        # The code block below retrieves the current node's neighbors and updates their distances
        neighbors = adjacency_list[current_min_node]
        for neighbor in neighbors:
            test = shortest_dist[current_min_node] + get_distance(distances,current_min_node, neighbor)
            if test < shortest_dist[neighbor]:
                shortest_dist[neighbor] = test
                # We also update the best path to the current node
                previous_nodes[neighbor] = current_min_node
        unvisited_nodes.remove(current_min_node)

    return previous_nodes


def find_path(adjacency_list, point_names):
    previous_nodes = dijkstra(adjacency_list, point_names)
    path = ['G']
    current_node = 'G'
    while current_node != 'R':
        current_node = previous_nodes[current_node]
        path.append(current_node)
    path.reverse()
    return path

#############################################################################################################

def is_point_inside_quad(point, quad_coords):
    x, y = point
    x_coords = [coord[0] for coord in quad_coords]
    y_coords = [coord[1] for coord in quad_coords]
    n = len(quad_coords)
    inside = False

    for i in range(n):
        j = (i + 1) % n
        if (((y_coords[i] <= y and y < y_coords[j]) or (y_coords[j] <= y and y < y_coords[i])) and
            (x < (x_coords[j] - x_coords[i]) * (y - y_coords[i]) / (y_coords[j] - y_coords[i]) + x_coords[i])):
            inside = not inside
    
    return inside


def find_closest_point (P, corners):
    Infinity = 10e10
    min_dist = Infinity
    for point in corners:
        dist = compute_dist(P, point)
        if dist < min_dist:
            min_dist = dist
            closest_point = point
    return closest_point


def shortest_path(RandG, object_corners, expended_corners):
    for obj_id, corners in object_corners.items():
        if is_point_inside_quad(RandG['robot'], corners):
            start = find_closest_point(RandG['robot'], corners)
            initial_pose = RandG['robot']
            RandG['robot'] = start
            adjacent_list = generate_adjacency_list(expended_corners, RandG, object_corners)
            points_name2coord = name2coord(expended_corners, RandG)
            shortest_path = find_path(adjacent_list, points_name2coord)
            shortest_path.insert(0, 'init')
            points_name2coord['init'] = initial_pose
            return shortest_path, adjacent_list, points_name2coord
    points_name2coord = name2coord(expended_corners, RandG)
    adjacent_list = generate_adjacency_list(expended_corners, RandG, object_corners)
    shortest_path = find_path(adjacent_list, points_name2coord)
    return shortest_path, adjacent_list, points_name2coord