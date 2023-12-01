import numpy as np
import cv2
import time
import vision_utils as utils
import matplotlib.colors as colors


GREEN_RGB = np.array([0, 255, 0], dtype=np.uint8)
BLUE_RGB = np.array([0, 0, 255], dtype=np.uint8)
BLACK_RGB = np.array([0, 0, 0], dtype=np.uint8)


### Based parameters for the color detection
COLOR_THREASHOLD = 20
SATURATION_THRESHOLD = 130
BRIGHTNESS_THRESHOLD = 60


# GREEN_HSV_LOWER = np.array([35, 80, 80])
# GREEN_HSV_UPPER = np.array([145, 255, 255])
# BLUE_HSV_LOWER = np.array([170, 80, 80])
# BLUE_HSV_UPPER = np.array([270, 255, 255])
# BLACK_HSV_LOWER = np.array([0, 0, 0])
# BLACK_HSV_UPPER = np.array([360, 255, 255])


def color_to_hsv(color):
    color = color/255
    hsv = colors.rgb_to_hsv(color)
    hsv[0] = hsv[0]*180
    hsv[1] = hsv[1]*255
    hsv[2] = hsv[2]*255
    return hsv.astype(np.uint8)

# Calculate the saturation of each rgb channel and return the average, knowing that the image is in HSV format
def image_saturation(image):
    saturation = np.mean(image, axis=2)
    return saturation

def hsv_range(base_color, color_threashold,saturation_threshold, brightness_threshold):
    # Convert base color to HSV
    hsv_base_color = color_to_hsv(base_color)
    # Extract hue, saturation and value from base color
    h, s, v = hsv_base_color[0], hsv_base_color[1], hsv_base_color[2]
    # Define lower and upper bounds for hue
    lower_hue = h - color_threashold
    upper_hue = h + color_threashold
    if lower_hue < 0:
        lower_hue = 180 + lower_hue
    if upper_hue > 180:
        upper_hue = upper_hue - 180

    if lower_hue > upper_hue:
        tmp = lower_hue
        lower_hue = upper_hue
        upper_hue = tmp
    lower = np.array([lower_hue, saturation_threshold, brightness_threshold])
    upper = np.array([upper_hue, 255, 255])

    return lower, upper

def black_range_hsv(brightness_threshold):
    lower = np.array([0, 0, 0])
    upper = np.array([360, 255, brightness_threshold-1])
    return lower, upper


def find_coordinates(contours, color):
    coordinates = {}
    obstacle_counter = 1
    blue_circle_counter = 1

    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.009 * cv2.arcLength(cnt, True), True)
        
        if len(approx) == 4:
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
                    rectangle_corners.append(corner)
                coordinates[f"Obstacle_{obstacle_counter}"] = rectangle_corners
                obstacle_counter += 1

            elif color == 'Blue':
                # Code for blue rectangles
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    coordinates["Position"] = (cX, cY)
    
                # Code to check if the red shape is a square
            if color == 'Red':
                # To filter out rectangles and keep only squares, we can check if all sides are approximately equal
                # First, find the width and height of the bounding rectangle
                width = cv2.norm(approx[0] - approx[1])
                height = cv2.norm(approx[1] - approx[2])
                # Check if width and height are similar (you may adjust the tolerance)
                if abs(width - height) <= max(width, height) * 0.2:  # 20% tolerance
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        coordinates[f"Red_Square_{red_square_counter}"] = (cX, cY)
                        red_square_counter += 1

        elif len(approx) == 3 and color == 'Red':
            # This is a triangle
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                coordinates[f"Red_Triangle_{red_triangle_counter}"] = (cX, cY)
                red_triangle_counter += 1

        elif color == 'Green':
            # Code for green goal (existing logic)
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                coordinates["Goal"] = (cX, cY)

        elif color == 'Blue':
            # Code for blue circles
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            center = (int(x), int(y))
            radius = int(radius)
            coordinates[f"Blue_Circle_{blue_circle_counter}"] = (center, radius)
            blue_circle_counter += 1

    return coordinates

def midpoint_robot(contours):
    # Filter rectangles and triangles
    red_rectangles = []
    red_triangles = []
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
        if len(approx) == 4:
            red_rectangles.append(cnt)
        elif len(approx) == 3:
            red_triangles.append(cnt)
    
    # Check if both shapes are found
    if len(red_rectangles) == 1 and len(red_triangles) == 1:
        # Centroid of the rectangle (square)
        rect = cv2.minAreaRect(red_rectangles[0])
        rect_center = rect[0]
        
        # Centroid of the triangle
        tri = cv2.minEnclosingTriangle(red_triangles[0])[0]
        tri_center = np.mean(tri, axis=0)
        
        # Calculate midpoint between centroids
        midpoint = ((rect_center[0] + tri_center[0]) / 2, (rect_center[1] + tri_center[1]) / 2)
        
        return rect_center, tri_center, midpoint
    
    return None, None, None  # Return None if shapes are not found or condition not met

def detection(frame,mode,color_type,color_threashold=COLOR_THREASHOLD,saturation_threshold=SATURATION_THRESHOLD,brightness_threshold=BRIGHTNESS_THRESHOLD):
    match color_type:
        case 'BGR':
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        case _:
            frame = frame
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    green_lower, green_upper = hsv_range(GREEN_RGB, color_threashold, saturation_threshold, brightness_threshold)
    blue_lower, blue_upper = hsv_range(BLUE_RGB, color_threashold, saturation_threshold, brightness_threshold)
    black_lower, black_upper = black_range_hsv(brightness_threshold-1)
    red_lower, red_upper = hsv_range(RED_RGB, color_threashold, saturation_threshold, brightness_threshold)

    # Define color ranges for green squares
    green_mask = cv2.inRange(hsv, green_lower, green_upper)
    # Define color ranges for blue squares
    blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
    # Define color ranges for black squares or rectangles
    black_mask = cv2.inRange(hsv, black_lower, black_upper)
    # Define color ranges for red shapes
    red_mask = cv2.inRange(hsv, red_lower, red_upper)

    # Find contours for each colour
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    black_contours, _ = cv2.findContours(black_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


    black_contours = black_contours[2:] # Exclude the map contour
    cv2.drawContours(frame, black_contours, -1, (0, 0, 255), 3);    # Red       color for   black   squares
    cv2.drawContours(frame, green_contours, -1, (0, 255,0), 3);   # Green    color for   green   squares
    cv2.drawContours(frame, blue_contours, -1, (255, 0, 0), 3);  # Blue    color for   blue    squares
    cv2.drawContours(frame, red_contours, -1, (0, 0, 0), 3);  # Black    color for   red    squares

    # Define the coordinates of the map
    # map_coordinates = [(0, 0), (0, 1000), (1000, 1000), (1000, 0)]

    # Blue squares
    blue_coordinates = []
    blue_coordinates = find_coordinates(blue_contours, 'Blue')
    # Green squares
    green_coordinates = []
    green_coordinates = find_coordinates(green_contours, 'Green')
    # Black rectangles
    black_coordinates = []
    black_coordinates = find_coordinates(black_contours, 'Black')
    # Red shapes
    red_coordinates = []
    red_coordinates = find_coordinates(red_contours, 'Red')

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

    return frame, blue_coordinates, green_coordinates, black_coordinates, red_coordinates

# Check if the camera is hidden (i.e. if the mask detecting the black squares is empty)
def is_camera_hidden(black_mask):
    if np.sum(black_mask) == 0:
        return True
    else:
        return False
        

def robot_angle(square_centroid, triangle_centroid):
    # Calculate vector from square centroid to triangle centroid
    vec = (triangle_centroid[0] - square_centroid[0], triangle_centroid[1] - square_centroid[1])

    # Calculate angle using arctan2
    angle_rad = math.atan2(vec[1], vec[0])
    angle_deg = math.degrees(angle_rad)
    
    # Ensure angle is between 0 and 360 degrees
    if angle_deg < 0:
        angle_deg += 360

    return robot_angle


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

