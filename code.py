import time
import math
import busio
import pulseio
import board
import pwmio
import adafruit_mpu6050
import adafruit_vl53l0x
from adafruit_motor import servo

# Import neccessary functions for IMU sensors and aircraft controllers
from imu import attitude, dist, smooth
from controllers import attitudeControl, altitudeControl
from read_throttle import readThrottle

# ========= Aileron Controller Setup ========= #

# Set desired roll angle to maintain [deg]
phi_cmd = 0

# Initialize roll and roll rate gain values
K_phi = 1.5 # 1.5
K_p = 0.02 # 0.02

# Initialize max aileron deflection [deg]
d_A_limit = 40

# Initialize filtering constants
a_phi = 0.5
a_p = 0.9



# ========= Elevator Controller Setup ========= #

# Set desired pitch angle to maintain [deg]
theta_cmd = 0

# Initialize pitch and pitch rate gain values
K_theta = -0.5 #-0.5
K_q = -0.05 #-0.05

# Initialize max elevator deflection [deg]
d_el_limit = 60

# Initialize filtering constants
a_theta = 0.8
a_q = 0.2



# ========= Altitude Controller Setup ========= #

# Set desired altitude to maintain [m]
h_cmd = 0.3

# Define altitude deadband [m]
h_db = 0.05

# Initialize throttle gain
K_T = 1

# Initialize % throttle increment
d_T = 0.1

# Initialize pilot throttle
T_p = 0




# ========= Servo Setup ========= #

# Setup I2C (inter-integrated circuit)
# with clock (SCL) on pin GP5 and data (SDA) on pin GP4
i2c = busio.I2C(board.GP5, board.GP4)

# Create sensor objects for the IMU and distance sensor
mpu = adafruit_mpu6050.MPU6050(i2c)
#vl53 = adafruit_vl53l0x.VL53L0X(i2c)
vl53 = 30

# Initialize left and right aileron servos
pwm_AL = pwmio.PWMOut(board.GP7, frequency=50)
pwm_AR = pwmio.PWMOut(board.GP6, frequency=50)
AL = servo.Servo(pwm_AL, min_pulse=900, max_pulse=2100)
AR = servo.Servo(pwm_AR, min_pulse=900, max_pulse=2100)
AL_center = 125
AR_center = 85


# Initialize elevator servo
pwm_el = pwmio.PWMOut(board.GP8, frequency=50)
el = servo.Servo(pwm_el, min_pulse=900, max_pulse=2100)
el_center = 95


# ========= Throttle Setup ========= #
pwm_motor = pwmio.PWMOut(board.GP9 , frequency = 50)
motor = servo.Servo(pwm_motor, min_pulse=900, max_pulse=2000)

# Set up object to listen for pulses from the receiver on pin GP2
pulses = pulseio.PulseIn(board.GP10)



# ========= Main Loop ========= #

# Intitialize filtered values
phi_f = math.radians(phi_cmd)
p_f = 0
theta_f = math.radians(theta_cmd)
q_f = 0

while True:
    
    # ========= Aileron Control ========= #
    
    # Determine current attitude [rad or rad/s]
    theta, phi, p, q, r = attitude(mpu)
    
    # Filter the roll angle and rate readings
    phi_f = smooth(phi, phi_f, a_phi)
    p_f = smooth(p, p_f, a_p)
    
    # Call aileron controller and determine deflection [deg]
    d_A = attitudeControl(phi_cmd, d_A_limit, K_phi, K_p, phi_f, p_f)
    
    # Change left and right aileron angles and ensure servo saturation is avoided
    print(theta, d_A)
    AL.angle = AL_center - d_A
    AR.angle = AR_center - d_A
    
    
    
    # ========= Elevator Control ========= #
    
    # Filter the pitch angle and rate readings
    theta_f = smooth(theta, theta_f, a_theta)
    q_f = smooth(q, q_f, a_q)
    
    
    # Call aileron controller and determine deflection [deg]
    d_el = attitudeControl(theta_cmd, d_el_limit, K_theta, K_q, theta_f, q_f)
    #print(d_el)
    # Change elevator angle
    el.angle = el_center + d_el
    print(theta_f, d_el)
    
    
    # ========= Altitude Control ========= #
    
    # Pull pilot throttle
    T_p = readThrottle(T_p, pulses)
        
    # Determine current distance from ground [m]
    h = dist(vl53)
    
    # Call altitude controller and determine throttle change
    d_T_temp = altitudeControl(h_cmd, K_T, h_db, d_T, h)
    
    # Correct throttle
    # print(time.monotonic(), h_f, max(0, min(T_p + d_T_temp, 1)), (el_center + d_el))
    motor.fraction = max(0, min(T_p + d_T_temp, 1))
    
