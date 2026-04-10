#!/bin/python

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import ekf

# Override globals in ekf module for consistency
ekf.dt = 0.1
ekf.wheelbase = 1.0

def run_simulation():
    # Simulation parameters
    total_time = 20.0
    dt = ekf.dt
    num_steps = int(total_time / dt)

    # Initial True State: [x, y, theta, v, omega]
    true_state = np.array([[0.0], [0.0], [0.0], [1.0], [0.1]]) # Moving in a circle
    
    # Initial Filter State
    state = np.array([[0.0], [0.0], [0.0], [0.0], [0.0]])
    cov = np.eye(5) * 1.0
    
    # Noises
    process_noise = np.eye(5) * 0.01
    measurement_noise = np.diag([0.1, 0.1, 0.05]) # [v_left, v_right, theta]

    # Storage for plotting
    true_history = []
    est_history = []
    meas_history = []

    for i in range(num_steps):
        # 1. Update True State (Ground Truth)
        true_state = ekf.predict_state(true_state, dt)
        
        # 2. Generate Noisy Measurements
        # H matrix maps state to [v_left, v_right, theta]
        # v_left = v - 0.5 * wheelbase * omega
        # v_right = v + 0.5 * wheelbase * omega
        v = true_state[3, 0]
        omega = true_state[4, 0]
        theta = true_state[2, 0]
        
        v_l = v - 0.5 * ekf.wheelbase * omega
        v_r = v + 0.5 * ekf.wheelbase * omega
        
        # Add noise
        meas = np.array([
            [v_l + np.random.normal(0, np.sqrt(measurement_noise[0, 0]))],
            [v_r + np.random.normal(0, np.sqrt(measurement_noise[1, 1]))],
            [theta + np.random.normal(0, np.sqrt(measurement_noise[2, 2]))]
        ])

        # 3. Run EKF
        state, cov = ekf.kf(meas, state, cov, process_noise, measurement_noise)

        # 4. Save data
        true_history.append(true_state.flatten())
        est_history.append(state.flatten())
        meas_history.append(meas.flatten())

    true_history = np.array(true_history)
    est_history = np.array(est_history)

    # Plotting
    plt.figure(figsize=(10, 8))
    plt.plot(true_history[:, 0], true_history[:, 1], 'g-', label='Ground Truth')
    plt.plot(est_history[:, 0], est_history[:, 1], 'r--', label='EKF Estimate')
    plt.xlabel('X position')
    plt.ylabel('Y position')
    plt.title('EKF Trajectory Tracking')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    
    # Save the plot
    plt.savefig('ekf_test_result.png')
    
    # Calculate RMSE
    pos_rmse = np.sqrt(np.mean(np.sum((true_history[:, :2] - est_history[:, :2])**2, axis=1)))
    theta_rmse = np.sqrt(np.mean((true_history[:, 2] - est_history[:, 2])**2))
    
    print(f"Simulation complete. Results saved to 'ekf_test_result.png'.")
    print(f"Final position RMSE: {pos_rmse:.4f} m")
    print(f"Final heading RMSE: {theta_rmse:.4f} rad")

if __name__ == "__main__":
    run_simulation()
