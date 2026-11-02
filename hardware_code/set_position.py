import time
from so101_utils import load_calibration, move_to_pose, hold_position, setup_motors

# Variables del SO101-ARM
PORT_ID = "COM5"  
ROBOT_NAME = "so101_follower"  

desired_position = {
    'shoulder_pan': 0.0,   # se trabaja en grados, con 0.0 como posición inicial
    'shoulder_lift': 0.0,
    'elbow_flex': 0.0,
    'wrist_flex': 0.0,
    'wrist_roll': 0.0,
    'gripper': 10.0           # 0-100 para el gripper
}
move_time = 2.0  # tiempo que se le da para llegar al goal
hold_time = 2.0  # tiempo total para mantenerse en la posición

# Estas funciones estan en so101_utils.py
calibration = load_calibration(ROBOT_NAME)
bus = setup_motors(calibration, PORT_ID)

starting_pose = bus.sync_read("Present_Position")

try:
    move_to_pose(bus, desired_position, move_time)   # ir a la posición deseada
    hold_position(bus, hold_time)                    # mantener
    move_to_pose(bus, starting_pose, move_time)      # regresar al inicio
finally:
    bus.disable_torque()                             # siempre soltar motores
