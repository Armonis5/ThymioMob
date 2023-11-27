import numpy as np
import cv2
import time
import vision_utils as utils
import matplotlib.colors as colors


GREEN_RGB = np.array([0, 255, 0], dtype=np.uint8)
BLUE_RGB = np.array([0, 0, 255], dtype=np.uint8)
BLACK_RGB = np.array([0, 0, 0], dtype=np.uint8)

COLOR_THREASHOLD = 20
SATURATION_THRESHOLD = 200
BRIGHTNESS_THRESHOLD = 70


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
    coordinates = []
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.009 * cv2.arcLength(cnt, True), True)
        if len(approx) == 4:
            rectangle = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rectangle)
            box = np.int0(box)
            if color == 'Black':
                coordinates.append(box)
            elif color in ['Green', 'Blue']:
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    coordinates.append([(cX, cY)])
    return coordinates

def write_colour_label(image, coordinates, color):
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    color_text_map = {
        'Black': (255, 255, 255),   # White text for Black contour
        'Green': (0, 155, 255),     # Orange text for Green contour 
        'Blue' : (255, 0, 0),       # Yellow text for Blue contour
    }

    text_color = color_text_map.get(color, (255, 255, 255))  # Default to white if no mapping found

    for coord in coordinates:
        if len(coord) == 1:
            x, y = coord[0]  # Extract x and y from the single coordinate
        else:
            x, y = coord  # Extract x and y from the tuple
        cv2.putText(image, color, (x, y), font, 1, text_color, 2, cv2.LINE_AA)

    return image

def sort_by_centroid(coordinates):
        coordinates.sort(key=lambda box: np.mean(box, axis=0)[0])  # Sort by x-coordinate of centroid
        return coordinates

def obstacle_detection(frame,mode,color_type):
    match color_type:
        case 'BGR':
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        case _:
            frame = frame
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)



    green_lower, green_upper = hsv_range(GREEN_RGB, COLOR_THREASHOLD, SATURATION_THRESHOLD, BRIGHTNESS_THRESHOLD)
    blue_lower, blue_upper = hsv_range(BLUE_RGB, COLOR_THREASHOLD, SATURATION_THRESHOLD, BRIGHTNESS_THRESHOLD)
    black_lower, black_upper = black_range_hsv(BRIGHTNESS_THRESHOLD-1)
    
    # Define color ranges for green squares
    green_mask = cv2.inRange(hsv, green_lower, green_upper)
    # Define color ranges for blue squares
    blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
    # Define color ranges for black squares or rectangles
    black_mask = cv2.inRange(hsv, black_lower, black_upper)

    # Find contours for each colour
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    black_contours, _ = cv2.findContours(black_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    black_contours = black_contours[2:] # Exclude the map contour
    cv2.drawContours(frame, black_contours, -1, (0, 0, 255), 3);    # Red       color for   black   squares
    cv2.drawContours(frame, green_contours, -1, (0, 255,0), 3);   # Orange    color for   green   squares
    cv2.drawContours(frame, blue_contours, -1, (255, 0, 0), 3);  # Pink    color for   blue    squares
    # Define the coordinates of the map
    map_coordinates = [(0, 0), (0, 1000), (1000, 1000), (1000, 0)]

    # Blue squares
    blue_coordinates = []
    blue_coordinates = find_coordinates(blue_contours, 'Blue')
    # Green squares
    green_coordinates = []
    green_coordinates = find_coordinates(green_contours, 'Green')
    # Black rectangles
    black_coordinates = []
    black_coordinates = find_coordinates(black_contours, 'Black')

    blue_coordinates.sort(key=lambda x: x[0][0])
    green_coordinates.sort(key=lambda x: x[0][0])
    black_coordinates.sort(key=lambda x: x[0][0])

    #Alternatively, if you want to ensure all the boxes are sorted in a specific manner, you might consider calculating their centroids and using these for sorting:

    blue_coordinates = sort_by_centroid(blue_coordinates)
    green_coordinates = sort_by_centroid(green_coordinates)
    black_coordinates = sort_by_centroid(black_coordinates)

    blue_coordinates = [tuple(coord) for coord in blue_coordinates]
    green_coordinates = [tuple(coord) for coord in green_coordinates]
    black_coordinates = [tuple(coord) for coord in black_coordinates]

    match mode:
        case 'blue':
            frame = blue_mask
        case 'green':
            frame = green_mask
        case 'black':
            frame = black_mask
        case 'hsv':
            frame = hsv
        case 'all':
            frame = frame
        case _:
            print('Invalid mode')
            frame = frame

    return frame, blue_coordinates, green_coordinates, black_coordinates


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

