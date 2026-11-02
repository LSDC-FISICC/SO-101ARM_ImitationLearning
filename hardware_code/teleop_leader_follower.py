import time
from so101_utils import load_calibration, setup_motors, move_to_pose


# Variables del SO101-ARM tanto Leader como Follower
FOLLOWER_PORT = "COM5"     # Puerto del follower
LEADER_PORT = "COM4"       # Puerto del leader
FOLLOWER_NAME = "so101_follower"           # calibration_files/so101_follower.json
LEADER_NAME = "so101_leader"               # calibration_files/so101_leader.json

CONTROL_HZ = 60          # frecuencia del lazo de control
MAX_STEP = 5.0           # máx. cambio por ciclo (grados; gripper en unidades 0-100).
                         # A 60 Hz equivale a 300 °/s. 
SYNC_TIME = 3.0          # segundos que tarda el follower en alcanzar la pose inicial del leader
RETURN_TO_REST = True    # al terminar, llevar el follower a reposo antes de soltar el torque

# posicion de reposo (en grados, gripper en 0-100)
rest_position = {
    'shoulder_pan': 0.0, 'shoulder_lift': -92.25, 'elbow_flex': 88.80,
    'wrist_flex': 61.88, 'wrist_roll': -8.00, 'gripper': 0.0,
}

JOINTS = list(rest_position.keys())


def limit_step(target, last_cmd, max_step):
    out = {} # limitar el cambio de posición a max_step por articulación
    for j in JOINTS:
        delta = target[j] - last_cmd[j]
        delta = max(-max_step, min(max_step, delta))
        out[j] = last_cmd[j] + delta
    return out

# Calibraciones y buses de comunicación
follower = setup_motors(load_calibration(FOLLOWER_NAME), FOLLOWER_PORT)
leader = setup_motors(load_calibration(LEADER_NAME), LEADER_PORT)

# El leader debe poder moverse a mano sin torque
leader.disable_torque()

try:
    leader_pose = leader.sync_read("Present_Position")
    print("Leader:  ", leader_pose)
    print("Follower:", follower.sync_read("Present_Position"))
    input("Sujeta/despeja el follower. Enter para sincronizarlo con el leader (Ctrl+C cancela)...")

    # Como leader y follower pueden estar en posiciones muy distintas
    # Se lleva primero al follower a la posición del leader, para que luego pueda seguirlo.
    print("Moviendo el follower a la posición del leader...")
    move_to_pose(follower, leader_pose, SYNC_TIME)
    last_cmd = dict(leader_pose)
    follower.sync_write("Goal_Position", last_cmd, normalize=True)

    print("Teleoperación activa. Ctrl+C para terminar.")

    # lazo de teleoperación
    # Casi igual a la teleoperacion oficial de LeroBot
    # Fue hecho apra pruebas
    period = 1.0 / CONTROL_HZ
    next_tick = time.perf_counter()
    while True:
        target = leader.sync_read("Present_Position")
        cmd = limit_step(target, last_cmd, MAX_STEP)
        follower.sync_write("Goal_Position", cmd, normalize=True)
        last_cmd = cmd

        next_tick += period
        sleep = next_tick - time.perf_counter()
        if sleep > 0:
            time.sleep(sleep)
        else:
            next_tick = time.perf_counter() 

except KeyboardInterrupt:
    print("\nTerminando teleoperación...")
finally:
    try:
        if RETURN_TO_REST: #Cuando termina la teleoperación se lleva el follower a reposo antes de soltar el torque
            print("Llevando el follower a reposo...")
            move_to_pose(follower, rest_position, 3.0)
            follower.sync_write("Goal_Position", rest_position, normalize=True)
            time.sleep(0.5)
    finally:
        follower.disable_torque()   # el brazo cae por gravedad si no está en reposo
        leader.disable_torque()
        print("Torque desactivado en ambos brazos.")