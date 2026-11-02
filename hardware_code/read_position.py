# read_position.py

import time
from so101_utils import load_calibration, setup_motors

# Variables del SO101-ARM
PORT_ID = "COM5"  
ROBOT_NAME = "so101_follower"

# Calibración de setup y del motor bus
calibration = load_calibration(ROBOT_NAME)
bus = setup_motors(calibration, PORT_ID)

# Hay que deshabilitar el torque para poder mover los motores 
# libremente antes de una calibración.
bus.disable_torque()

# Print de posiciones.
while True:
    present_pos = bus.sync_read("Present_Position")
    print(present_pos)
    time.sleep(0.02)  # cada 50Hz
