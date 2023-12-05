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

def draw_point(frame, point, color=(0, 255, 100), radius=5):
    cv2.circle(frame, point, radius, color, -1)

def draw_arrow(frame, start_point, end_point, color=(0, 0, 255), thickness=2):
    cv2.arrowedLine(frame, start_point, end_point, color, thickness)

def draw_path(frame, start_point,end_point, path_points, color=(255, 0, 0), thickness=2):
    cv2.line(frame, start_point, path_points[0], color, thickness)
    for i in range(len(path_points) - 1):
        cv2.line(frame, path_points[i], path_points[i + 1], color, thickness)
    cv2.line(frame, path_points[-1], end_point, color, thickness)

def frame_draw(frame, robot_position, path_points, end_point):
    draw_point(frame, robot_position, ROBOT_COLOR)
    draw_path(frame, path_points[0],end_point,path_points, PATH_COLOR)
    draw_points(frame, path_points, POINT_COLOR)
    draw_point(frame, end_point, END_COLOR)

def on_trackbar(val, threshold, index):
    threshold[index] = val
