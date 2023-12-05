import numpy as np
import cv2
import time
import vision_utils as utils
import matplotlib.colors as colors
import math


GREEN_BGR = np.uint8([[[0, 255, 0]]])
BLUE_BGR = np.uint8([[[255, 0, 0]]])
BLACK_BGR = np.uint8([[[0, 0, 0]]])
RED_BGR = np.uint8([[[0, 0, 255     ]]])


### Based parameters for the color detection
COLOR_THRESHOLD = 20
SATURATION_THRESHOLD = 110
BRIGHTNESS_THRESHOLD = 70


# GREEN_HSV_LOWER = np.array([35, 80, 80])
# GREEN_HSV_UPPER = np.array([145, 255, 255])
# BLUE_HSV_LOWER = np.array([170, 80, 80])
# BLUE_HSV_UPPER = np.array([270, 255, 255])
# BLACK_HSV_LOWER = np.array([0, 0, 0])
# BLACK_HSV_UPPER = np.array([360, 255, 255])


def color_to_hsv(color):
    hsv_color = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
    return hsv_color
    

# Calculate the saturation of each rgb channel and return the average, knowing that the image is in HSV format
def image_saturation(image):
    saturation = np.mean(image, axis=2)
    return saturation

def hsv_range(base_color, color_threashold,saturation_threshold, brightness_threshold):
    # Convert base color to HSV
    hsv_base_color = color_to_hsv(base_color)
    # Extract hue, saturation and value from base color
    lower = hsv_base_color[0][0][0] - color_threashold, saturation_threshold, brightness_threshold
    upper = hsv_base_color[0][0][0] + color_threashold, 255, 255
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


def black_range_hsv(brightness_threshold):
    lower = np.array([0, 0, 0])
    upper = np.array([360, 255, brightness_threshold-1])
    return lower, upper

def find_coordinates(contours, color,width=0, height=0,origin=(0,0)):
    coordinates = []
    obstacle_counter = 0
    blue_circle_counter = 0
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.009 * cv2.arcLength(cnt, True), True)
        #if len(approx) == 4:
            # Code for rectangles (existing logic)
        rectangle = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rectangle)
        box = np.intp(box)
        new_order = np.array([box[1], box[2], box[3], box[0]])
        box = new_order

        if color == 'Black':
            rectangle_corners = []
            for point in box:
                corner = (point[0], point[1])
                corner = coordinate_to_IRL(corner,height,origin)
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
                coor = coordinate_to_IRL((cX,cY),height,origin)
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
    # Filter rectangles and triangles
    red_rectangles = []
    red_triangles = []
    for cnt in contours:
        if cv2.contourArea(cnt) > 100:
            approx = cv2.approxPolyDP(cnt, 0.08 * cv2.arcLength(cnt, True), True)
            if len(approx) == 4:
                    red_rectangles.append(cnt)
                
            elif len(approx) == 3:
                    red_triangles.append(cnt)
    
    # Check if both shapes are found
    if len(red_rectangles) > 1 and len(red_triangles) > 1:
        # Centroid of the rectangle (square)
        rect = cv2.minAreaRect(red_rectangles[0])
        rect_center = rect[0]
        
        
        # Centroid of the triangle
        tri = cv2.minEnclosingTriangle(red_triangles[0])[1]
        tri_center = np.mean(tri, axis=0)[0]
        # Convert to IRL coordinates
        rect_center = coordinate_to_IRL(rect_center,height,origin)
        tri_center = coordinate_to_IRL((tri_center[0],tri_center[1]),height,origin)        
        # Calculate midpoint between centroids
        if rect_center != None and tri_center != None:
            midpoint = ((rect_center[0] + tri_center[0]) / 2, (rect_center[1] + tri_center[1]) / 2)
        else:
            midpoint = None
        return rect_center, tri_center, midpoint
    
    return None, None, None  # Return None if shapes are not found or condition not met

def detection(frame,mode,color_type,color_threashold=COLOR_THRESHOLD,saturation_threshold=SATURATION_THRESHOLD,brightness_threshold=BRIGHTNESS_THRESHOLD,width=75,height=100,origin=(0,0)):
    match color_type:
        case 'BGR':
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        case _:
            frame = frame
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv_filtered = cv2.bilateralFilter(hsv,9, 80, 80)

    blue_lower, blue_upper = hsv_range(BLUE_BGR, color_threashold, saturation_threshold, brightness_threshold)
    # Define color ranges for blue squares
    blue_mask = cv2.inRange(hsv_filtered, blue_lower, blue_upper)
    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #Only countours with area greater than 1000
    blue_contours = [cnt for cnt in blue_contours if cv2.contourArea(cnt) > 200]
    cv2.drawContours(frame, blue_contours, -1, (0, 255, 255), 3);  # Yellow    color for   blue    squares
    # Blue circles
    blue_coordinates = []
    if blue_contours is not None:
        blue_coordinates = find_coordinates(blue_contours, 'Blue')
        # Wraping of the frame, to make sure the blues circles is always on the corner of the frame
        width, height,origin = compute_dimensions(blue_coordinates,height,origin)
        height = max(height,width)
        cv2.circle(frame, origin, 5, (0, 255, 255))
        if len(blue_coordinates) == 4:
            # compute the perspective transform matrix and then apply it
            bcoor = [b[0] for b in blue_coordinates]
            src = np.array(bcoor, dtype="float32")
            dst = np.array([(height,width),(0,width),(height,0),(0,0)], dtype="float32")
            M = cv2.getPerspectiveTransform(src, dst)
            frame = cv2.warpPerspective(frame, M, (width, height))
            #Update the coordinates of the blue circles
            bc = find_coordinates(blue_contours, 'Blue',width,height,origin)
            width,height,origin = compute_dimensions(bc,height,origin)
            height = max(height,width)


    green_lower, green_upper = hsv_range(GREEN_BGR, color_threashold, saturation_threshold, brightness_threshold)
    black_lower, black_upper = black_range_hsv(brightness_threshold-1)
    red_lower, red_upper = hsv_range(RED_BGR, color_threashold, saturation_threshold, brightness_threshold)

    # Define color ranges for green squares
    green_mask = cv2.inRange(frame, green_lower, green_upper)
   
    # Define color ranges for black squares or rectangles
    black_mask = cv2.inRange(frame, black_lower, black_upper)
    # Define color ranges for red shapes
    red_mask = cv2.inRange(frame, red_lower, red_upper)

    # Find contours for each colour
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #Only countours with area greater than 1000
    green_contours = [cnt for cnt in green_contours if cv2.contourArea(cnt) > 1000]
    black_contours, _ = cv2.findContours(black_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #Only countours with area greater than 1000
    black_contours = [cnt for cnt in black_contours if cv2.contourArea(cnt) > 1000]
    #Only countours with area greater than 1000
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    red_contours = [cnt for cnt in red_contours if cv2.contourArea(cnt) > 200]

    cv2.drawContours(frame, black_contours, -1, (0, 0, 255), 3);    # Red       color for   black   squares
    cv2.drawContours(frame, green_contours, -1, (0, 255,0), 3);   # Green    color for   green   squares
    cv2.drawContours(frame, red_contours, -1, (0, 0, 0), 3);  # Black    color for   red    squares

    # Define the coordinates of the map
    # Green squares
    green_coordinates = []
    if green_contours is not None and height != 0:
        green_coordinates = find_coordinates(green_contours, 'Green',width,height,origin)         
    # Black rectangles
    black_coordinates = []
    if black_contours is not None and   height != 0:
        black_coordinates = find_coordinates(black_contours, 'Black',width,height,origin)
        midpoints = [None, None, None]
    if red_contours is not None and height != 0:
        midpoints = midpoint_robot(red_contours, width, height,origin)
    if midpoints is not None and midpoints[0] is not None and midpoints[1] is not None and midpoints[2] is not None:
        angle_robot = robot_angle(midpoints[0], midpoints[1])
        robot_midpoint = midpoints[2]
    else:
        angle_robot = None
        robot_midpoint = None

    if height != None:
        coor = IRL_to_coordinate(robot_midpoint,height,origin)
        if coor != None:
            cv2.circle(frame, (int(coor[0]), int(coor[1])), 5, (255, 0, 255),5)
        
    if len(green_coordinates)>0:
        green_coordinates = green_coordinates[0]


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


    return frame, robot_midpoint,angle_robot, green_coordinates, black_coordinates, blue_coordinates,height,origin

# Check if the camera is hidden (i.e. if the mask detecting the black squares is empty)
def is_camera_hidden(black_mask):
    if np.sum(black_mask) == 0:
        return True
    else:
        return False

def coordinate_to_IRL(coordinate,height,origin=(0,0)):
    if coordinate == None:
        return None
    coordinate = (origin[1]-coordinate[1], origin[0]-coordinate[0])
    factor = 100/height
    x = coordinate[0]*factor
    y = coordinate[1]*factor
    return (x,y)

def IRL_to_coordinate(coordinate,height,origin=(0,0)):
    if coordinate == None:
        return None
    factor = height/100
    x = coordinate[0]*factor
    y = coordinate[1]*factor
    coordinate = (origin[1]-y, origin[0]-x)
    return coordinate       

def robot_angle(square_centroid, triangle_centroid):
    # Calculate vector from square centroid to triangle centroid
    vec = (triangle_centroid[0] - square_centroid[0], triangle_centroid[1] - square_centroid[1])
    # Calculate angle using arctan2
    angle_rad = math.atan2(vec[1], vec[0])
    # Ensure angle is between 0 and 360 degrees
    # if angle_deg < 0:
    #     angle_deg += 360

    return angle_rad

def coordinate_in_map(coordinate,width,height):
    x = coordinate[0]
    y = coordinate[1]
    if x < 0 or x > width or y < 0 or y > height:
        return False
    else:
        return True

def zoom_frame(frame, zoom_factor=2):
    height, width = frame.shape[:2]
    new_height, new_width = int(height / zoom_factor), int(width / zoom_factor)

    # Cropping the center of the frame
    start_row, start_col = height // 2 - new_height // 2, width // 2 - new_width // 2
    end_row, end_col = start_row + new_height, start_col + new_width
    cropped_frame = frame[start_row:end_row, start_col:end_col]

    # Resizing cropped frame back to original frame size
    zoomed_frame = cv2.resize(cropped_frame, (width, height), interpolation=cv2.INTER_LINEAR)
    return zoomed_frame




def capture_data(camera_index, num_frames=500):
    cap = cv2.VideoCapture(camera_index)
    data = []

    while len(data) < num_frames:
        ret, frame = cap.read()
        if not ret:
            break

        # Process frame to extract measurements
        # Example: Extracting a central pixel value
        center_pixel = frame[frame.shape[0] // 2, frame.shape[1] // 2, :]
        data.append(center_pixel)

    cap.release()
    return np.array(data)

def calculate_covariance_matrix(data):
    return np.cov(data.T)

