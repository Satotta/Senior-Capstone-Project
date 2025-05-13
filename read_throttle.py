import time
import pulseio
import board

def readThrottle(T_p, pulses):
    
    while len(pulses) > 0:
        
        # loop through the pulses that have been collected
        p_in = pulses.popleft()
        
        # if the latest pulse width is within acceptable bounds
        if p_in < 2000 and p_in > 900:
            
            # Clear pulse list
            pulses.clear()
            
            # Return current normalized thottle value
            return max(0, min((p_in - 1000) / (1900 - 1000), 1))
        
    # If not in acceptable pulse range return previous throttle value
    return T_p
