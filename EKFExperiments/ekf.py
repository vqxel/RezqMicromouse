#!/bin/python
import numpy as np
import math

state = np.array([[0.0], [0.0], [math.pi/2], [0.0], [0.0]]) # vector [pos_x, pos_y, theta, velo, omega]

cov_P = np.eye(5) * 1e-6 # We're very confident in where we start 
process_noise_Q = np.eye(5) * 0.4 
measurement_noise_R = np.eye(5) * 0.4 

wheelbase = 1

dt = 0.1

def pos_x(state):
    return state[0, 0]

def pos_y(state):
    return state[1, 0]

def theta(state):
    return state[2, 0]

def velo(state):
    return state[3, 0]

def omega(state):
    return state[4, 0]

def predict_state(state, dt):
    new_state = np.array([[pos_x(state) + velo(state) * np.cos(theta(state)) * dt], [pos_y(state) + velo(state) * np.sin(theta(state)) * dt], [theta(state) + omega(state) * dt], [velo(state)], [omega(state)]])
    """
    x' = x + v * cos(theta) * dt
    y' = y + v * sin(theta) * dt
    theta' = theta + w * dt
    v' = v
    w' = w
    """
    return new_state

def gen_transition_jacobian(state, dt):
    jacob = np.array([[1.0, 0.0, -velo(state) * np.sin(theta(state)) * dt, np.cos(theta(state)) * dt, 0.0], [0.0, 1.0, velo(state) * np.cos(theta(state)) * dt, np.sin(theta(state)) * dt, 0.0], [0.0, 0.0, 1.0, 0.0, dt], [0.0, 0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 0.0, 1.0]])
    '''
    1    0    -v * sin(theta)* dt    cos(theta) * dt    0
    0    1    v * cos(theta) * dt    sin(theta) * dt    0
    0    0    1                      0                  dt
    0    0    0                      0                  1
    '''
    return jacob

def kf(meas, state, cov_P, process_noise_Q, measurement_noise_R):
    '''
    state  [x, y, theta, v, w]
    meas   [l_velo, r_velo, theta, mouse_v, mouse_w]

    H: state to meas
    H @ state -> meas

    l_velo = v - w * d * 0.5
    r_velo = v + w * d * 0.5
    theta = theta
    mouse_v = v
    mouse_w = w
    '''
    state_to_meas_H = np.array([[0.0, 0.0, 0.0, 1, -wheelbase * 0.5], [0.0, 0.0, 0.0, 1, wheelbase * 0.5], [0.0, 0.0, 1, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 0.0, 1.0]])

    pred_state = predict_state(state, dt)
    transition_jacobian = gen_transition_jacobian(state, dt)

    pred_cov_P = transition_jacobian @ cov_P @ transition_jacobian.T + process_noise_Q

    innovation = meas - state_to_meas_H @ pred_state
    innovation_cov_P = state_to_meas_H @ pred_cov_P @ state_to_meas_H.T + measurement_noise_R

    kalman_gain = pred_cov_P @ state_to_meas_H.T  @ np.linalg.inv(innovation_cov_P)

    state = pred_state + kalman_gain @ innovation
    cov_P = (np.eye(5) - kalman_gain @ state_to_meas_H) @ pred_cov_P

    return (state, cov_P)

if __name__ == "__main__":
    for _ in range(10):
        meas = np.array([[1.0], [1.0], [math.pi/2], [1.0], [0.0]])
        state, cov_P = kf(meas, state, cov_P, process_noise_Q, measurement_noise_R)
