import numpy as np 

def sampling_motion_model(control_command, x_prev, delta_t): 
    """Motion model for a differential drive robot.
    
    Args:
    - control_command (tuple): Commanded speed as (v, w) where v is translational velocity and w is rotational velocity.
    - x_prev (tuple): Previous pose of the robot as (x, y, theta).
    - delta_t (float): Time step.
    
    Returns:
    - tuple: New pose of the robot as (x_new, y_new, theta_new).
    """
    
    # Noise parameters (alphas)
    a = [.0000000000, .0000000000, .0000000000, .000000000, .000000000, .0000000000] 
    
    v, w = control_command[0],control_command[1]

    # If no motion, return the previous pose
    if v ==0 and w==0: 
        return x_prev 
    
    # Add noise to the control commands
    v_hat = v + sample_normal_distribution(a[0]*v**2 + a[1]*w**2) 
    w_hat = w + sample_normal_distribution(a[2]*v**2 + a[3]*w**2) 
    gamma_hat = sample_normal_distribution(a[4]*v**2 + a[5]*w**2)

    # Motion equations for a differential drive robot
    x, y, theta = x_prev
    
    if np.abs(w_hat) < .000000001: 
        x_new = x  + v_hat*delta_t* np.cos(theta)
        y_new = y  + v_hat*delta_t* np.sin(theta)
        theta_new = theta 

    else: 
        x_new = x - (v_hat/w_hat) * np.sin(theta) + (v_hat/w_hat) * np.sin(theta + w_hat * delta_t)
        y_new = y + (v_hat/w_hat) * np.cos(theta) - (v_hat/w_hat) * np.cos(theta + w_hat * delta_t)
        theta_new = theta + w_hat * delta_t + gamma_hat * delta_t 

    if theta_new>np.pi: 
        theta_new -= 2*np.pi 
    else: 
        theta_new += 2*np.pi 

    return x_new, y_new, theta_new 

def sample_normal_distribution(b):
    """Sample from a normal distribution using the central limit theorem.
    
    Args:
    - b (float): A bound for the uniform distribution from which values will be sampled.
    
    Returns:
    - float: A value sampled from an approximate normal distribution.
    """
    # Sample 12 values from a uniform distribution and sum them up
    random_values = np.random.uniform(-b, b, 12)
    return 0.5 * np.sum(random_values)



