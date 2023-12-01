import math
import time
import matplotlib.pyplot as plt
import numpy as np

def predict(state, input_val, covariance, Ts, theta, process_noise, thymio_b=11, thymio_r=2):
    # Prediction step
    A = np.array([[1, 0, Ts, 0, 0, 0],
                [0, 1, 0, Ts, 0, 0],
                [0, 0, 0,  0, 0, 0],
                [0, 0, 0,  0, 0, 0],
                [0, 0, 0,  0, 1, Ts],
                [0, 0, 0,  0, 0, 0]])

    B = np.array([[0, 0],
                  [0, 0],
                  [thymio_r*0.5*math.cos(theta), thymio_r*0.5*math.cos(theta)],
                  [-thymio_r*0.5*math.sin(theta), -thymio_r*0.5*math.sin(theta)],
                  [0, 0],
                  [-thymio_r/thymio_b, thymio_r/thymio_b]])
    

    new_state = np.dot(A, state) + np.dot(B, input_val)
    new_covariance = np.dot(A, np.dot(covariance, A.T)) + process_noise

    return new_state, new_covariance


def measure(state, covariance, cam_measurement):
    # Update step based on camera measurement
    if cam_measurement = ...: # camera not covered
        C = np.array([[1, 0, 0, 0, 0, 0],
                    [0, 1, 0, 0, 0, 0]])  # Measurement matrix

        R = measurement_noise # Measurement noise covariance
        i_t = cam_measurement - np.dot(C, state) #innovation 
        S_t = np.dot(C, np.dot(covariance, C.T)) + R #innovation of variance
    
        K = np.dot(covariance, np.dot(C.T, np.linalg.inv(S_t))) #gain
    
        new_state = state + np.dot(K, i_t)
        new_covariance = np.dot((np.eye(2) - np.dot(K, C)), covariance)
    else: # camera covered
        new_state = state
        new_covariance = covariance
    
    return new_state, new_covariance

def turn_to_target(pos_1, pos_2, thymio_angle): #turn towards the target when the angle difference is bigger than 10°, since otherwise the path will be inaccurate
    global motor_left_target, motor_right_target
    x_dist = pos_2[0] - pos_1[0]
    y_dist = pos_2[1] - pos_1[1]
    target_angle = math.atan2(y_dist, x_dist)/math.pi*180
    angle_diff = (target_angle - thymio_angle + 180) % 360 - 180  # result always between -180° and 180°, positive when target angle is greater than thymio angle
    if abs(angle_diff) > 10:
        return angle_diff
    else:
        return 0

def go_to_target(pos_1, pos_2, thymio_angle, pos_gain = 10, angle_gain = 0.85):#code to bring Thymio back to the right direction
    global motor_left_target, motor_right_target
    x_dist = pos_2[0] - pos_1[0]
    y_dist = pos_2[1] - pos_1[1]
    target_angle = math.atan2(y_dist, x_dist)/math.pi*180
    control_x = pos_gain * x_dist
    control_y = pos_gain * y_dist
    angle_diff = (target_angle - thymio_angle + 180) % 360 - 180  # result always between -180° and 180°, positive when target angle is greater than thymio angle
    control_angle = angle_gain * angle_diff
    motor_left_target = math.ceil(control_x - control_angle)
    motor_right_target = math.ceil(control_x + control_angle)
    
    return motor_left_target, motor_right_target
