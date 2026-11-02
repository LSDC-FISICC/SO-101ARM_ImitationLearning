
import csv
import time
from pathlib import Path

from so101_utils import load_calibration, setup_motors

# Variables del SO101-ARM
ROBOT_NAME = "so101_follower"
PORT_ID = "COM5"

JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]

# Se va a generar una secuencia de movimientos: 0, +20, 0, -20, 0 grados con el fin 
# de determinar el error de seguimiento y la respuesta del sistema.
OFFSETS_DEG = [0.0, 20.0, 0.0, -20.0, 0.0]

SAMPLE_PERIOD = 0.05  # Aproximadamente 20 muestras/s
SETTLE_TIME = 2.0     # Duración de cada objetivo

OUTPUT_CSV = Path(__file__).with_name("joint_tracking_all.csv")


def read_positions(bus):
    positions = bus.sync_read("Present_Position")
    return {joint: float(positions[joint]) for joint in JOINTS}


def main():
    calibration = load_calibration(ROBOT_NAME)
    bus = None

    try:
        bus = setup_motors(calibration, PORT_ID)

        # Obtener las posiciones actuales antes de ordenar movimientos.
        initial_positions = read_positions(bus)

        print("\nPRUEBA DE SEGUIMIENTO DEL SO-101")
        print("=" * 55)
        print("Puerto:", PORT_ID)
        print("\nPosiciones iniciales:")
        for joint in JOINTS:
            print(f"  {joint:<15}: {initial_positions[joint]:8.2f}°")

        print("\nSe moverá una articulación a la vez.")
        print("Cada una recibirá objetivos relativos a su posición inicial.")
        print("Comprueba que el brazo tenga espacio libre y que los")
        print("movimientos de +/-3 grados sean seguros.")
        confirmation = input("\nEscribe SI para comenzar: ").strip()

        if confirmation != "SI":
            print("Prueba cancelada; no se enviaron objetivos de movimiento.")
            return

        with OUTPUT_CSV.open(
            "w", newline="", encoding="utf-8"
        ) as file:
            writer = csv.writer(file)
            writer.writerow([
                "time_s",
                "joint",
                "target_deg",
                "measured_deg",
                "error_deg",
                "phase",
            ])

            start_time = time.monotonic()

            for joint in JOINTS:
                start_angle = initial_positions[joint]

                print(f"\n--- {joint} ---")

                for step, offset in enumerate(OFFSETS_DEG, start=1):
                    target = start_angle + offset

                    print(
                        f"\nObjetivo {step}/{len(OFFSETS_DEG)}: "
                        f"{target:.2f}° "
                        f"(desplazamiento {offset:+.2f}°)"
                    )

                    # Solo se ordena movimiento a la articulación evaluada
                    # Movemos una articulación a la vez para poder medir el error de seguimiento
                    bus.write("Goal_Position", joint, target)

                    phase_start = time.monotonic()

                    while time.monotonic() - phase_start < SETTLE_TIME:
                        positions = read_positions(bus)
                        measured = positions[joint]
                        error = target - measured
                        elapsed = time.monotonic() - start_time

                        phase = (
                            "settling"
                            if time.monotonic() - phase_start < 0.5
                            else "hold"
                        )

                        writer.writerow([
                            f"{elapsed:.4f}",
                            joint,
                            f"{target:.4f}",
                            f"{measured:.4f}",
                            f"{error:.4f}",
                            phase,
                        ])

                        print(
                            f"\rComandada: {target:7.2f}° | "
                            f"Medida: {measured:7.2f}° | "
                            f"Error: {error:7.2f}°",
                            end="",
                            flush=True,
                        )

                        file.flush()
                        time.sleep(SAMPLE_PERIOD)

                    print()

        print("\nPrueba terminada.")
        print(f"CSV guardado en:\n{OUTPUT_CSV}")

    finally:
        if bus is not None:
            bus.disconnect()


if __name__ == "__main__":
    main()
