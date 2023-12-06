import cv2
import numpy as np

ROBOT_COLOR = (0, 0, 255)
START_COLOR = (0, 255, 0)
END_COLOR = (255, 0, 0)
POINT_COLOR = (0,0,0)
PATH_COLOR = (0,0,150)


def draw_point(frame, point, color=(0, 255, 100), radius=5):
    point = (int(point[0]), int(point[1]))
    cv2.circle(frame, point, radius, color, 2)

def draw_arrow(frame, start_point, end_point, color=(0, 0, 255), thickness=2):
    cv2.arrowedLine(frame, start_point, end_point, color, thickness)

def draw_path(frame,path_points, thickness=2):
    cv2.circle(frame, (int(path_points[0][0]), int(path_points[0][1])), 3, START_COLOR, -1)
    for i in range(len(path_points)-1):
        cv2.line(frame, (int(path_points[i][0]), int(path_points[i][1])), (int(path_points[i+1][0]),
                int(path_points[i+1][1])), PATH_COLOR, thickness)
    cv2.circle(frame, (int(path_points[-1][0]), int(path_points[-1][1])), 3, END_COLOR, -1)


def on_trackbar(val, threshold, index):
    threshold[index] = val
