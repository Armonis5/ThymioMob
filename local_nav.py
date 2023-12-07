import math
import time
import matplotlib.pyplot as plt
import numpy as np


def predict(state, input_val, covariance, Ts, theta, process_noise, thymio_b=11, thymio_r=2):
    # Prediction step
    A = np.array([[1, 0, Ts, 0, 0, 0],  # system matrix
                [0, 1, 0, Ts, 0, 0],
                [0, 0, 0,  0, 0, 0],
                [0, 0, 0,  0, 0, 0],
                [0, 0, 0,  0, 1, Ts],
                [0, 0, 0,  0, 0, 0]])

    B = np.array([[0, 0],       # input matrix
                [0, 0],
                [0.5*math.cos(theta), 0.5*math.cos(theta)],
                [0.5*math.sin(theta), 0.5*math.sin(theta)],
                [0, 0],
                [-1/thymio_b, 1/thymio_b]])
    

    new_state = np.dot(A, state) + np.dot(B, input_val)
    new_covariance = np.dot(A, np.dot(covariance, A.T)) + process_noise

    return new_state, new_covariance


def measure(state, covariance, cam_measurement, measurement_noise, position):
    position = np.array(position)
    if cam_measurement: # camera not hidden/covered
        C = np.array([[1, 0, 0, 0, 0, 0], # output matrix
                    [0, 1, 0, 0, 0, 0]])

        R = measurement_noise # measurement noise covariance
        i_t = position - np.dot(C, state) # innovation 
        S_t = np.dot(C, np.dot(covariance, C.T)) + R # variance of innovation
    
        K = np.dot(covariance, np.dot(C.T, np.linalg.inv(S_t))) # Kalman gain
    
        new_state = state + np.dot(K, i_t)
        new_covariance = np.dot((np.eye(6) - np.dot(K, C)), covariance)
    else: # camera covered
        new_state = state
        new_covariance = covariance
    
    return new_state, new_covariance


"""
A function that was not used anymore in the final report

def turn_to_target(pos_1, pos_2, thymio_angle): #turn towards the target when the angle difference is bigger than 10°, since otherwise the path will be inaccurate
    global motor_left_target, motor_right_target
    thymio_angle = 180*thymio_angle/math.pi
    x_dist = pos_2[0] - pos_1[0]
    y_dist = pos_2[1] - pos_1[1]
    target_angle = 180*math.atan2(y_dist, x_dist)/math.pi
    angle_diff = target_angle - thymio_angle   # result always between -180° and 180°, positive when target angle is greater than thymio angle
    return angle_diff
"""

def go_to_target(pos_1, pos_2, thymio_angle, angle_gain = 1): # code to keep Thymio going in the right direction
    global motor_left_target, motor_right_target
    thymio_angle = thymio_angle/math.pi*180
    x_dist = pos_2[0] - pos_1[0]
    y_dist = pos_2[1] - pos_1[1]
    global_distance = math.sqrt(x_dist**2 + y_dist**2)
    target_angle = math.atan2(y_dist, x_dist)/math.pi*180
    angle_diff = (target_angle - thymio_angle + 180) % 360 - 180 
    control_angle = angle_gain * angle_diff

    if abs(angle_diff) > 90: # only turning, since the angle difference is too great, no linear motion
        coef_angle = 0
    elif abs(angle_diff) > 10:
        coef_angle = 0.1+5/abs(angle_diff)
    else:
        coef_angle = 0.6

    
    if global_distance > 10:
        coef_distance = 1
    else:
        coef_distance = 0.1+0.09*global_distance

    
    speed_forward =  300* coef_angle * coef_distance
    
    motor_left_target = math.ceil(speed_forward - control_angle) 
    motor_right_target = math.ceil(speed_forward + control_angle) 
    
    return motor_left_target, motor_right_target


def check_target(state, coordinate, cor): # checks if the Thymio has reached its next target or is very close to its next target (hence the "cor")
    if (state[0]>(coordinate[0]+cor) or state[0]<(coordinate[0]-cor)) or (state[1]>(coordinate[1]+cor) or state[1]<(coordinate[1]-cor)):
        return True
    return False


def convert_speed(thymio_speed): # takes thymio speed and outputs real speed (cm/s), determined experimentally
    if thymio_speed == 0:
        return 0
    real_speed_7s = 0.2179*thymio_speed + 0.9714
    real_speed = real_speed_7s/7
    
    return real_speed