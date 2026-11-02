
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt

# GRAFICAR DATOS DEL CSV DE SEGUIMIENTO DE ARTICULACIONES

BASE_DIR = Path(__file__).resolve().parent

CSV_FILE = BASE_DIR / "joint_tracking_all.csv"
OUTPUT_DIR = BASE_DIR / "joint_tracking_plots"

OUTPUT_DIR.mkdir(parents=True, exist_ok=True)


def main():
    # Verificar que existe el CSV
    if not CSV_FILE.exists():
        raise FileNotFoundError(
            f"No se encontró el archivo:\n{CSV_FILE}\n"
            "Verifica que hayas ejecutado primero "
            "test_joint_tracking.py."
        )

    # Cargar los datos
    df = pd.read_csv(CSV_FILE)

    required_columns = [
        "time_s",
        "joint",
        "target_deg",
        "measured_deg",
        "error_deg",
    ]

    missing = [col for col in required_columns if col not in df.columns]

    if missing:
        raise ValueError(
            f"Faltan columnas en el CSV: {missing}"
        )

    # Convertir columnas numéricas
    for col in [
        "time_s",
        "target_deg",
        "measured_deg",
        "error_deg",
    ]:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    df = df.dropna(subset=required_columns)

    if df.empty:
        raise ValueError("El CSV no contiene datos válidos.")

    # Corregir el error por si se modificó manualmente el CSV
    df["error_deg"] = df["target_deg"] - df["measured_deg"]

    joints = df["joint"].unique()

    print("\nESTADÍSTICAS DE SEGUIMIENTO")
    print("=" * 75)

    for joint in joints:
        data = df[df["joint"] == joint].sort_values("time_s")

        target = data["target_deg"]
        measured = data["measured_deg"]
        error = data["error_deg"]

        mae = error.abs().mean()
        rmse = (error.pow(2).mean()) ** 0.5
        max_error = error.abs().max()
        mean_error = error.mean()

        print(f"\nArticulación: {joint}")
        print(f"  Número de muestras:       {len(data)}")
        print(f"  Error medio (sesgo):      {mean_error:+.3f} grados")
        print(f"  Error absoluto medio:     {mae:.3f} grados")
        print(f"  RMSE:                     {rmse:.3f} grados")
        print(f"  Error absoluto máximo:    {max_error:.3f} grados")

        
        # FIGURA: POSICIÓN Y ERROR
       

        fig, (ax_pos, ax_err) = plt.subplots(
            2,
            1,
            figsize=(12, 8),
            sharex=True,
            gridspec_kw={"height_ratios": [2, 1]},
        )

        # Posición comandada vs. medida
        ax_pos.plot(
            data["time_s"],
            target,
            label="Posición comandada",
            linestyle="--",
        )

        ax_pos.plot(
            data["time_s"],
            measured,
            label="Posición medida",
        )

        ax_pos.set_title(
            f"Seguimiento de posición: {joint}"
        )
        ax_pos.set_ylabel("Ángulo (grados)")
        ax_pos.grid(True)
        ax_pos.legend()

        # Error de seguimiento
        ax_err.plot(
            data["time_s"],
            error,
            label="Error = comandada - medida",
        )

        ax_err.axhline(
            y=0,
            linestyle="--",
            linewidth=1,
        )

        ax_err.set_title("Error de seguimiento")
        ax_err.set_xlabel("Tiempo (s)")
        ax_err.set_ylabel("Error (grados)")
        ax_err.grid(True)
        ax_err.legend()

        fig.tight_layout()

        output_file = OUTPUT_DIR / f"{joint}.png"
        fig.savefig(output_file, dpi=200, bbox_inches="tight")

        print(f"  Gráfica guardada: {output_file.name}")

    print("\n" + "=" * 75)
    print(f"Gráficas guardadas en:\n{OUTPUT_DIR}")

    # Mostrar las figuras en pantalla
    plt.show()


if __name__ == "__main__":
    main()
