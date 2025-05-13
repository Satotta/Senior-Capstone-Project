import math

# Define function for finding vehicle attitude
def attitude(mpu):
    
    # Pull acceleromter data
    ax, ay, az = mpu.acceleration
    
    # Pull gyro data
    gx, gy, gz = mpu.gyro
    
    # Correct y and z axis directions
    ay = -ay
    az = -az
    gy = -gy
    gz = -gz
    
    # Convert accelerometer data to pitch (theta) and roll (phi)
    theta = math.atan2(ax, math.sqrt(ay**2 + az**2))
    phi = math.atan2(-ay, -az)
    
    # Apply rates
    p = gx
    q = gy
    r = gz

    # Return values
    return theta, phi, p, q, r




# Define function for finding vehicle alt
def dist(vl53):
    
    return vl53 #vl53.range/1000




# Define filter to smooth attitude data
def smooth(data, data_f, a):
    
    return (1 - a) * data_f + a * data
    

    