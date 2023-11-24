import cv2
import numpy as np

ROBOT_COLOR = (0, 0, 255)
START_COLOR = (0, 255, 0)
END_COLOR = (255, 0, 0)
POINT_COLOR = (0,0,0)
PATH_COLOR = (0,0,150)

def draw_points(frame, points, color=(0, 255, 100), radius=5):
    for point in points:
        cv2.circle(frame, point, radius, color, -1)

def draw_arrow(frame, start_point, end_point, color=(0, 0, 255), thickness=2):
    cv2.arrowedLine(frame, start_point, end_point, color, thickness)

def draw_path(frame, path_points, color=(255, 0, 0), thickness=2):
    for i in range(len(path_points) - 1):
        cv2.line(frame, path_points[i], path_points[i + 1], color, thickness)
