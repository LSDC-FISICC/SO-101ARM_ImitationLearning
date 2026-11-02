import numpy as np
import kinematics as K

# Se prueba numericamente la consistencia entre FK e IK
# Rangos de prueba
RANGES = {
    "shoulder_pan": (-60.0, 60.0),
    "shoulder_lift": (-80.0, 20.0),
    "elbow_flex": (-10.0, 90.0),
    "wrist_flex": (-60.0, 60.0),
    "wrist_roll": (-90.0, 90.0),
}

JOINTS = K.JOINTS


def make_pose(values):
    return {joint: float(value) for joint, value in zip(JOINTS, values)}


def pose_text(pose):
    return "(" + ", ".join(f"{pose[j]:6.1f}" for j in JOINTS) + ")"


def run_test(name, q_cal):
    # hacemos la vuelta FK -> IK -> FK y medimos el error
    p_target, pitch_target = K.fk_cal(q_cal)

    try:
        q_ik = K.ik_cal(
            p_target,
            pitch_target,
            q5_deg=q_cal["wrist_roll"],
            q_ref_cal=q_cal,
        )
        p_reconstructed, pitch_reconstructed = K.fk_cal(q_ik)

        error_m = float(np.linalg.norm(p_reconstructed - p_target))
        error_mm = error_m * 1000.0
        pitch_error_deg = abs(float(pitch_reconstructed - pitch_target))

        return {
            "name": name,
            "status": "OK",
            "q_test": q_cal,
            "q_ik": q_ik,
            "target": p_target,
            "reconstructed": p_reconstructed,
            "error_mm": error_mm,
            "pitch_error_deg": pitch_error_deg,
            "message": "",
        }

    except K.Unreachable as exc:
        return {
            "name": name,
            "status": "NO IK",
            "q_test": q_cal,
            "q_ik": None,
            "target": p_target,
            "reconstructed": None,
            "error_mm": None,
            "pitch_error_deg": None,
            "message": str(exc),
        }


def main():
    np.set_printoptions(precision=4, suppress=True)
    rng = np.random.default_rng(42)

    # Poses fijas para que los resultados iniciales sean reproducibles.
    test_poses = [
        ("cero", [0, 0, 0, 0, 0]),
        ("pose_A", [20, -30, 45, -15, 0]),
        ("pose_B", [-35, -50, 60, 10, 30]),
        ("pose_C", [45, -20, 25, -35, -45]),
        ("pose_D", [-15, 10, 35, -25, 60]),
    ]

    # Añadir poses aleatorias dentro de los rangos configurados.
    for i in range(25):
        values = [
            rng.uniform(*RANGES[joint])
            for joint in JOINTS
        ]
        test_poses.append((f"random_{i + 1:02d}", values))

    results = [
        run_test(name, make_pose(values))
        for name, values in test_poses
    ]

    print("\nVERIFICACIÓN FK–IK DEL SO-101 (SOFTWARE, SIN MOTORES)")
    print("=" * 100)
    print("Ángulos en grados de calibración; posiciones y errores calculados en metros/mm.")
    print()
    header = (
        f"{'Pose':<12} {'Estado':<8} {'X [m]':>9} {'Y [m]':>9} {'Z [m]':>9} "
        f"{'Error pos. [mm]':>17} {'Error pitch [°]':>17}"
    )
    print(header)
    print("-" * len(header))

    successful = []
    failed = []

    for result in results:
        x, y, z = result["target"]
        if result["status"] == "OK":
            successful.append(result)
            print(
                f"{result['name']:<12} {result['status']:<8} "
                f"{x:9.4f} {y:9.4f} {z:9.4f} "
                f"{result['error_mm']:17.6f} "
                f"{result['pitch_error_deg']:17.6f}"
            )
        else:
            failed.append(result)
            print(
                f"{result['name']:<12} {result['status']:<8} "
                f"{x:9.4f} {y:9.4f} {z:9.4f} "
                f"{'—':>17} {'—':>17}"
            )
            print(f"    Motivo: {result['message']}")

    print("\nRESUMEN")
    print("-" * 40)
    print(f"Poses evaluadas:          {len(results)}")
    print(f"Soluciones IK obtenidas:  {len(successful)}")
    print(f"Sin solución IK:          {len(failed)}")

    if successful:
        errors = np.array([r["error_mm"] for r in successful])
        pitch_errors = np.array([r["pitch_error_deg"] for r in successful])
        print(f"Error de posición medio:  {errors.mean():.6f} mm")
        print(f"Error de posición máximo: {errors.max():.6f} mm")
        print(f"Error de pitch máximo:    {pitch_errors.max():.6f} grados")

        worst = max(successful, key=lambda r: r["error_mm"])
        print(f"Pose con mayor error:     {worst['name']}")

        # Tolerancia numérica para tener de referencia
        tolerance_mm = 1e-3
        passed = int(np.sum(errors <= tolerance_mm))
        print(
            f"Pruebas con error <= {tolerance_mm:g} mm: "
            f"{passed}/{len(successful)}"
        )

    print("\nNOTAS")
    print("- Esta prueba comprueba la consistencia matemática FK -> IK -> FK.")


if __name__ == "__main__":
    main()
