import board
import digitalio
import storage

# EXAMPLE "boot.py" FILE
# NEEDS TO BE RENAMED TO "boot.py" ON THE PICO

# Set up a pin (GP16) to act as a switch
switch = digitalio.DigitalInOut(board.GP16)
switch.direction = digitalio.Direction.INPUT
switch.pull = digitalio.Pull.UP

# If the GP16 pin is connected to ground with a wire
# CircuitPython can write to the drive
# changes require you to unplug / replug in the Pico
storage.remount("/", readonly=switch.value)