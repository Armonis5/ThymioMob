import numpy as np
import cv2
import time
import vision_utils as utils
import matplotlib.colors as colors
import math

# Constants
MAP_HEIGHT = 100
MAP_WIDTH = 75

HEIGHT_DELTA = 5
WIDTH_DELTA = 5
ORIGIN_DELTA = 5

GREEN_BGR = np.uint8([[[0, 255, 0]]])
BLUE_BGR = np.uint8([[[255, 0, 0]]])
BLACK_BGR = np.uint8([[[0, 0, 0]]])
RED_BGR = np.uint8([[[0, 0, 255]]])


# Based parameters for the color detection
COLOR_THRESHOLD = 20
SATURATION_THRESHOLD = 110
BRIGHTNESS_THRESHOLD = 70

CIRCLE_AREA_THRESHOLD = 100
RECTANGLE_AREA_THRESHOLD = 500
RED_AREA_THRESHOLD = 50


def color_to_hsv(color):
    """
    Convert a color from BGR to HSV format
    """
    hsv_color = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
    return hsv_color
    

def hsv_range(base_color, color_threshold,saturation_threshold, brightness_threshold):
    """
    Calculate the lower and upper range of the color in HSV format based on the base color and the color threshold
    """
    # Convert base color to HSV
    hsv_base_color = color_to_hsv(base_color)
    # Extract hue, saturation and value from base color
    lower = hsv_base_color[0][0][0] - color_threshold, saturation_threshold, brightness_threshold
    upper = hsv_base_color[0][0][0] + color_threshold, 255, 255
    if lower[0] < 0:
        lower = 0, lower[1], lower[2]
    if upper[0] > 360:
        upper = 360, upper[1], upper[2]
    
    if lower[1] <0:
        lower = lower[0], 0, lower[2]
    if upper[1] >255:
        upper = upper[0], 255, upper[2]

    if lower[2] <0:
        lower = lower[0], lower[1], 0
    if upper[2] >255:
        upper = upper[0], upper[1], 255

    lower = np.array(lower)
    upper = np.array(upper)

    return lower, upper

def compute_dimensions(blue_coordinates,old_height=0,old_origin=(0,0)):
    """
    Compute the dimensions of the map based on the blue squares
    """
    if len(blue_coordinates) != 4:
        return 0,old_height,old_origin
    min_x = min(blue_coord[0][0] for blue_coord in blue_coordinates)
    max_x = max(blue_coord[0][0] for blue_coord in blue_coordinates)
    min_y = min(blue_coord[0][1] for blue_coord in blue_coordinates)
    max_y = max(blue_coord[0][1] for blue_coord in blue_coordinates)
    width = max_x - min_x
    height = max_y - min_y
    origin = (max_x, max_y)
    return width, height, origin

def compute_transform_dimensions(dots):
    """
    Compute the dimensions of the map afeter the perspective transform
    """
    min_x = min(coord[0] for coord in dots)
    max_x = max(coord[0] for coord in dots)
    min_y = min(coord[1] for coord in dots)
    max_y = max(coord [1] for coord in dots)
    width = max_x - min_x
    height = max_y - min_y
    origin = (max_x, max_y)
    return width, height, origin


def black_range_hsv(brightness_threshold):
    """
    Calculate the lower and upper range of the black color in HSV format based on the brightness threshold
    """
    lower = np.array([0, 0, 0])
    upper = np.array([360, 255, brightness_threshold-1])
    return lower, upper

def find_coordinates(contours, color,width=0, height=0,origin=(0,0)):
    """
    Find the coordinates of the shapes in the frame, differentiating between black, green and blue shapes
    """
    coordinates = []
    obstacle_counter = 0
    blue_circle_counter = 0
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.009 * cv2.arcLength(cnt, True), True)
        rectangle = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rectangle)
        box = np.intp(box)
        new_order = np.array([box[1], box[2], box[3], box[0]])
        box = new_order

        if color == 'Black':
            rectangle_corners = []
            for point in box:
                corner = (point[0], point[1])
                corner = coordinate_to_IRL(corner,width,height,origin)
                if not coordinate_in_map(corner,width,height):
                    break
                rectangle_corners.append(corner)
            if rectangle_corners != []:
                coordinates.append(rectangle_corners)
       
        elif color == 'Green':
            # Code for green goal (existing logic)
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                coor = coordinate_to_IRL((cX,cY),width,height,origin)
                if coordinate_in_map(coor,width,height):
                    coordinates.append(coor)

        elif color == 'Blue':
            # Code for blue circles
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            center = (int(x), int(y))
            radius = int(radius)
            if radius >5:
                coordinates.append((center, radius))

    return coordinates

def midpoint_robot(contours,width, height,origin=(0,0)):
    """
    Find the midpoint between the centroids of a rectangle and a triangle
    """
    # Filter rectangles and triangles
    red_rectangles = []
    red_triangles = []
    for cnt in contours:
        if cv2.contourArea(cnt) > RED_AREA_THRESHOLD:
            approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
            if len(approx) == 4:

                    red_rectangles.append(cnt)
                
            elif len(approx) == 3:

                    red_triangles.append(cnt)
    
    # Check if both shapes are found
    if len(red_rectangles) >= 1 and len(red_triangles) >= 1:
        # Centroid of the rectangle (square)
        rect = cv2.minAreaRect(red_rectangles[0])
        rect_center = rect[0]
        
        
        # Centroid of the triangle
        tri = cv2.minEnclosingTriangle(red_triangles[0])[1]
        tri_center = np.mean(tri, axis=0)[0]
        # Convert to IRL coordinates
        rect_center = coordinate_to_IRL(rect_center,width,height,origin)
        tri_center = coordinate_to_IRL((tri_center[0],tri_center[1]),width,height,origin)        
        # Calculate midpoint between centroids
        midpoint = ((rect_center[0] + tri_center[0]) / 2, (rect_center[1] + tri_center[1]) / 2)
        return rect_center, tri_center, midpoint
    
    return None, None, None  # Return None if shapes are not found or condition not met

def detection(frame,mode,color_type,color_threshold=COLOR_THRESHOLD,saturation_threshold=SATURATION_THRESHOLD,
              brightness_threshold=BRIGHTNESS_THRESHOLD,width=75,height=100,origin=(0,0)):
    """
    Function that processes the frame by applying the color filters and the perspective transform to detect thes shapes in the frame
    Detects the different colors as:    blue: references
                                        green: goals
                                        black: obstacles
                                        red: robot
    Returns the frame, the robot position, the robot angle, the goal position, the obstacle position, the reference position, 
    the width, the height and the origin of the map
    """


    match color_type:
        case 'BGR':
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        case _:
            frame = frame

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv_filtered = cv2.bilateralFilter(hsv,9, 80, 80)

    # Define color ranges for blue squares
    blue_lower, blue_upper = hsv_range(BLUE_BGR, color_threshold, saturation_threshold, brightness_threshold)
    # Define color ranges for blue squares
    blue_mask = cv2.inRange(hsv_filtered, blue_lower, blue_upper)
    # Find contours for blue squares
    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # Keep only countours with area greater than 100
    blue_contours = [cnt for cnt in blue_contours if cv2.contourArea(cnt) > CIRCLE_AREA_THRESHOLD]
    # Yellow color for blue circles
    cv2.drawContours(frame, blue_contours, -1, (0, 255, 255), 3);

    # Blue circles management
    blue_coordinates = []
    if blue_contours is not None:
        # Find the coordinates of the blue circles
        blue_coordinates = find_coordinates(blue_contours, 'Blue')
        # Store the old values
        old_height = height
        old_origin = origin
        old_width = width
        # Compute the dimensions of the map
        width, height,origin = compute_dimensions(blue_coordinates,height,origin) 
        # Keep height always greater than width
        if height < width:
                tmp = height
                height = width
                width = tmp
        # If the dimensions of the map are not changed, keep the old values
        if abs(old_height-height) < HEIGHT_DELTA and abs(old_width-width) < WIDTH_DELTA and abs(old_origin[0]-origin[0]) < ORIGIN_DELTA and abs(old_origin[1]-origin[1]) < ORIGIN_DELTA:
            height = old_height
            width = old_width
            origin = old_origin
        else: 

            if len(blue_coordinates) == 4:
                # Compute the perspective transform matrix and then apply it
                bcoor = [b[0] for b in blue_coordinates]
                src = np.array(bcoor, dtype="float32")
                dst = np.array([(height,width), (0,width), (height,0), (0,0)], dtype="float32")
                M = cv2.getPerspectiveTransform(src, dst)
                frame = cv2.warpPerspective(frame, M, (height, width))
                bc = [transform_point(b[0],M) for b in blue_coordinates]
                width,height,origin = compute_transform_dimensions(bc)
                if height < width:
                    tmp = height
                    height = width
                    width = tmp

        # Recompute the color for the warped image
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv_filtered = cv2.bilateralFilter(hsv,9, 80, 80)

    # Define color ranges for green squares
    green_lower, green_upper = hsv_range(GREEN_BGR, color_threshold, saturation_threshold, brightness_threshold)
    # Define color ranges for black squares or rectangles
    black_lower, black_upper = black_range_hsv(brightness_threshold-1)
    # Define color ranges for red shapes
    red_lower, red_upper = hsv_range(RED_BGR, color_threshold, saturation_threshold, brightness_threshold)

    # Green mask
    green_mask = cv2.inRange(hsv_filtered, green_lower, green_upper)
    # Black mask
    black_mask = cv2.inRange(hsv_filtered, black_lower, black_upper)
    # Red mask
    red_mask = cv2.inRange(hsv_filtered, red_lower, red_upper)

    # Find contours for each colour and keep only the ones with area greater than a certain threshold
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    green_contours = [cnt for cnt in green_contours if cv2.contourArea(cnt) > RECTANGLE_AREA_THRESHOLD]

    black_contours, _ = cv2.findContours(black_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    black_contours = [cnt for cnt in black_contours if cv2.contourArea(cnt) > RECTANGLE_AREA_THRESHOLD]

    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    red_contours = [cnt for cnt in red_contours if cv2.contourArea(cnt) > RED_AREA_THRESHOLD]

    # Draw contours on the warped frame
    cv2.drawContours(frame, black_contours, -1, (0, 0, 255), 3);    # Red   color for   black   squares
    cv2.drawContours(frame, green_contours, -1, (0, 255,0), 3);     # Green color for   green   squares
    cv2.drawContours(frame, red_contours, -1, (0, 0, 0), 3);        # Black color for   red    squares

    # Green squares
    green_coordinates = []
    midpoints = None
    if green_contours is not None and height != 0 and width != 0:
        green_coordinates = find_coordinates(green_contours, 'Green',width,height,origin)         
    # Black rectangles
    black_coordinates = []
    if black_contours is not None and   height != 0 and width != 0:
        black_coordinates = find_coordinates(black_contours, 'Black',width,height,origin)
    if red_contours is not None and height != 0 and width != 0:
        midpoints = midpoint_robot(red_contours, width, height,origin)
    
    if midpoints is not None and midpoints[0] is not None and midpoints[1] is not None and midpoints[2] is not None:
        angle_robot = robot_angle(midpoints[0], midpoints[1])
        robot_midpoint = midpoints[2]
    else:
        angle_robot = None
        robot_midpoint = None

    if height != None:
        coor = IRL_to_coordinate(robot_midpoint,width,height,origin)
        if coor != None:
            cv2.circle(frame, (int(coor[0]), int(coor[1])), 5, (255, 0, 255),5)
        
    if len(green_coordinates)>0:
        green_coordinates = green_coordinates[0]

    # Show the different masks
    match mode:
        case 'blue':
            frame = blue_mask
        case 'green':
            frame = green_mask
        case 'black':
            frame = black_mask
        case 'red':
            frame = red_mask
        case 'hsv':
            frame = hsv
        case 'all':
            frame = frame
        case _:
            print('Invalid mode')
            frame = frame


    return frame, robot_midpoint,angle_robot, green_coordinates, black_coordinates, blue_coordinates,width,height,origin

def is_camera_hidden(black_mask):
    """
    Detect if the camera is hidden by checking if the black mask is empty
    """
    if np.sum(black_mask) == 0:
        return True
    else:
        return False

def coordinate_to_IRL(coordinate,width,height,origin=(0,0)):
    """
    Transform the coordinates from the frame to the IRL coordinates
    """
    coordinate = (origin[1]-coordinate[1], origin[0]-coordinate[0])
    h_factor = MAP_HEIGHT/height
    w_factor = MAP_WIDTH/width
    x = coordinate[0]*w_factor
    y = coordinate[1]*h_factor
    return (x,y)

def IRL_to_coordinate(coordinate,width,height,origin=(0,0)):
    """
    Transform the coordinates from the IRL coordinates to the frame
    """
    if coordinate == None:
        return None
    h_factor = height/MAP_HEIGHT
    w_factor = width/MAP_WIDTH
    x = coordinate[0]*w_factor
    y = coordinate[1]*h_factor
    coordinate = (int(origin[0]-y), int(origin[1]-x))
    return coordinate

def robot_angle(square_centroid, triangle_centroid):
    """
    Calculate the angle of the robot based on the centroid of the square and the centroid of the triangle
    """
    # Calculate vector from square centroid to triangle centroid
    vec = (triangle_centroid[0] - square_centroid[0], triangle_centroid[1] - square_centroid[1])
    # Calculate angle using arctan2
    angle_rad = math.atan2(vec[1], vec[0])

    return angle_rad

def coordinate_in_map(coordinate,width,height):
    """
    Check if the coordinate is in the map
    """
    x = coordinate[0]
    y = coordinate[1]
    if x < 0 or x > width or y < 0 or y > height:
        return False
    else:
        return True

def zoom_frame(frame, zoom_factor=2):
    """
    Zoom in the frame
    """
    height, width = frame.shape[:2]
    new_height, new_width = int(height / zoom_factor), int(width / zoom_factor)

    # Cropping the center of the frame
    start_row, start_col = height // 2 - new_height // 2, width // 2 - new_width // 2
    end_row, end_col = start_row + new_height, start_col + new_width
    cropped_frame = frame[start_row:end_row, start_col:end_col]

    # Resizing cropped frame back to original frame size
    zoomed_frame = cv2.resize(cropped_frame, (width, height), interpolation=cv2.INTER_LINEAR)
    return zoomed_frame

def transform_point(point, matrix):
    """
    Transform a point using the perspective transform matrix
    """
    point_homogeneous = np.array([point[0], point[1], 1])
    transformed_point = np.dot(matrix, point_homogeneous)
    transformed_point = transformed_point / transformed_point[2]
    return transformed_point[:2]
