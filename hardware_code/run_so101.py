import time
from so101_utils import load_calibration, setup_motors, move_to_pose, hold_position

# Variables del SO101-ARM
PORT_ID = "COM5"   # Puerto del follower, en la carpeta images esta el puerto
ROBOT_NAME = "so101_follower"        # debe coincidir con calibration_files/so101_follower.json
SPEED_FACTOR = 1.2               # Si se baja se mueve más rapido
LOOP = True                     # True = repetir la secuencia hasta detener

# brazo "acostado" en su pose inicial
rest_position = {
    'shoulder_pan': 0.0, 'shoulder_lift': -102.00, 'elbow_flex': 96.80,
    'wrist_flex': 62.88, 'wrist_roll': -91.67, 'gripper': 0.78,
}

# Misma secuencia que run_mujoco_simulation_startingpose.py: (pose, tiempo_movimiento, tiempo_espera)
sequence = [
    ({'shoulder_pan': 0.0,   'shoulder_lift': 0.0,   'elbow_flex': 0.0,
      'wrist_flex': 0.0,     'wrist_roll': 0.0,      'gripper': 0.0},   2.0, 2.0),
    ({'shoulder_pan': 0.0,  'shoulder_lift': -20.0, 'elbow_flex': 30.0,
      'wrist_flex': 20.0,    'wrist_roll': 0.0,      'gripper': 30.0},  2.0, 1.0),
    ({'shoulder_pan': 0.0, 'shoulder_lift': 20.0,  'elbow_flex': -20.0,
      'wrist_flex': -20.0,   'wrist_roll': 45.0,     'gripper': 10.0},  2.0, 1.0),
    (rest_position,                                                     2.0, 1.0),
]

# Movimiento de la posicion inicial al goal
def go_to(bus, pose, duration):
    move_to_pose(bus, pose, duration)
    pos = bus.sync_read("Present_Position")
    bus.sync_write("Goal_Position", pos, normalize=True)


calibration = load_calibration(ROBOT_NAME)
bus = setup_motors(calibration, PORT_ID)

try:
    print("Posición inicial:", bus.sync_read("Present_Position"))
    input("Despeja el área alrededor del brazo y presiona Enter para comenzar (Ctrl+C para cancelar)...")

    while True:
        for pose, move_time, hold_time in sequence:
            go_to(bus, pose, move_time * SPEED_FACTOR)
            hold_position(bus, hold_time)
        if not LOOP:
            break

    print("Secuencia terminada.")
except KeyboardInterrupt:
    print("\nInterrumpido por el usuario.")
finally:
    #cuando termina se cae el brazo pq se desactiva el torque.
    bus.disable_torque()
    print("Torque desactivado.")