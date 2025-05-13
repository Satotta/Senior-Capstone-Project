import math

# Define general attitude control function (roll and pitch)
def attitudeControl(ang_cmd, delta_max, K1, K2, ang, ang_rate):
    
    # Convert control angle to radians for use in controller
    ang_cmd = math.radians(ang_cmd)
    
    # Roll error
    ang_error = ang_cmd - ang
    desired_rate = K1 * ang_error
    
    # Roll rate error
    delta_temp = desired_rate - K2 * ang_rate
    
    # Convert change in angle to degrees
    delta_temp = math.degrees(delta_temp)
    
    # Apply saturation element [deg]
    return max(-delta_max, min(delta_max, delta_temp))



# Define altitude controller (throttle)
def altitudeControl(h_cmd, K, h_db, d_T, h):
    
    # Altitude error and gain
    h_signal = K * (h_cmd - h)
    
    # Deadband logic
    # Airplane is low, increase % throttle
    if h_signal > h_db:
        return d_T
    
    # Airplane is high, decrease % throttle
    elif h_signal < -h_db:
        return -d_T
    
    # Airplane is at correct alt, keep % throttle the same
    else:
        return 0