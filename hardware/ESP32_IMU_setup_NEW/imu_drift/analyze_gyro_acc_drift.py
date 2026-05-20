from pathlib import Path
from io import StringIO
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


G = 9.80665

REQUIRED = [
    "t_us",
    "ax_corr", "ay_corr", "az_corr",
    "gx_corr", "gy_corr", "gz_corr",
]


def load_csv(path: Path) -> pd.DataFrame:
    with open(path, "r", encoding="latin1", errors="ignore") as f:
        lines = f.readlines()

    header_line = None
    for i, line in enumerate(lines):
        if "t_us" in line and "ax_corr" in line and "gx_corr" in line:
            header_line = i
            break

    if header_line is None:
        raise ValueError(f"Kunne ikke finde CSV-header i {path.name}")

    df = pd.read_csv(StringIO("".join(lines[header_line:])))

    missing = [c for c in REQUIRED if c not in df.columns]
    if missing:
        raise ValueError(f"Mangler kolonner i {path.name}: {missing}")

    df = df[REQUIRED].copy()
    df["t_s"] = (df["t_us"] - df["t_us"].iloc[0]) / 1e6
    return df


def integrate_angle(t: np.ndarray, rate_dps: np.ndarray) -> np.ndarray:
    dt = np.diff(t, prepend=t[0])
    dt[0] = 0.0
    return np.cumsum(rate_dps * dt)


def drift_deg_per_min(t: np.ndarray, angle_deg: np.ndarray) -> float:
    return np.polyfit(t, angle_deg, 1)[0] * 60.0


def analyze_file(path: Path) -> dict:
    df = load_csv(path)
    t = df["t_s"].to_numpy()

    # Accelerometerdata konverteres fra m/s² til g
    ax = df["ax_corr"].to_numpy() / G
    ay = df["ay_corr"].to_numpy() / G
    az = df["az_corr"].to_numpy() / G

    gx = df["gx_corr"].to_numpy()
    gy = df["gy_corr"].to_numpy()
    gz = df["gz_corr"].to_numpy()

    acc_mag = np.sqrt(ax**2 + ay**2 + az**2)
    acc_error = acc_mag - 1.0

    angle_x = integrate_angle(t, gx)
    angle_y = integrate_angle(t, gy)
    angle_z = integrate_angle(t, gz)

    summary = {
        "file": path.name,
        "duration_s": t[-1],
        "samples": len(df),

        "gx_mean_dps": np.mean(gx),
        "gy_mean_dps": np.mean(gy),
        "gz_mean_dps": np.mean(gz),

        "gx_std_dps": np.std(gx, ddof=1),
        "gy_std_dps": np.std(gy, ddof=1),
        "gz_std_dps": np.std(gz, ddof=1),

        "gx_drift_deg_per_min": drift_deg_per_min(t, angle_x),
        "gy_drift_deg_per_min": drift_deg_per_min(t, angle_y),
        "gz_drift_deg_per_min": drift_deg_per_min(t, angle_z),

        "gx_final_angle_deg": angle_x[-1],
        "gy_final_angle_deg": angle_y[-1],
        "gz_final_angle_deg": angle_z[-1],

        "ax_mean_g": np.mean(ax),
        "ay_mean_g": np.mean(ay),
        "az_mean_g": np.mean(az),

        "ax_std_g": np.std(ax, ddof=1),
        "ay_std_g": np.std(ay, ddof=1),
        "az_std_g": np.std(az, ddof=1),

        "acc_mag_mean_g": np.mean(acc_mag),
        "acc_mag_std_g": np.std(acc_mag, ddof=1),
        "acc_mag_error_mean_g": np.mean(acc_error),
        "acc_mag_error_max_abs_g": np.max(np.abs(acc_error)),
    }

    return {
        "path": path,
        "t": t,
        "ax": ax,
        "ay": ay,
        "az": az,
        "gx": gx,
        "gy": gy,
        "gz": gz,
        "acc_mag": acc_mag,
        "angle_x": angle_x,
        "angle_y": angle_y,
        "angle_z": angle_z,
        "summary": summary,
    }


def plot_individual_results(results: list[dict], out_dir: Path, save_plots: bool) -> None:
    for r in results:
        name = r["path"].stem

        # Plot 1: Samlet accelerationsstørrelse
        plt.figure(figsize=(10, 6))
        plt.plot(r["t"], r["acc_mag"], label="Samlet acceleration")
        plt.axhline(1.0, linestyle="--", label="1 g")
        plt.xlabel("Tid [s]")
        plt.ylabel("Samlet acceleration [g]")
        plt.title(f"Samlet accelerationsstørrelse — {name}")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        if save_plots:
            plt.savefig(out_dir / f"{name}_acc_mag.png", dpi=200)

        # Plot 2: Integreret gyrodrift
        plt.figure(figsize=(10, 6))
        plt.plot(r["t"], r["angle_x"], label="x")
        plt.plot(r["t"], r["angle_y"], label="y")
        plt.plot(r["t"], r["angle_z"], label="z")
        plt.xlabel("Tid [s]")
        plt.ylabel("Integreret vinkel [deg]")
        plt.title(f"Integreret gyrodrift — {name}")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        if save_plots:
            plt.savefig(out_dir / f"{name}_gyro_drift.png", dpi=200)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("files", nargs="+", help="CSV-filer med både gyro og accelerometer")
    parser.add_argument("--save-plots", action="store_true")
    args = parser.parse_args()

    results = [analyze_file(Path(f)) for f in args.files]
    summary_df = pd.DataFrame([r["summary"] for r in results])

    # Gem outputs i samme mappe som scriptet
    out_dir = Path(__file__).parent

    print("\nGyro- og accelerometerdrift:")
    print(summary_df.to_string(index=False, float_format=lambda x: f"{x:.6f}"))

    summary_path = out_dir / "gyro_acc_drift_summary.csv"
    summary_df.to_csv(summary_path, index=False)

    plot_individual_results(results, out_dir, args.save_plots)

    print(f"\nGemt summary: {summary_path}")

    if args.save_plots:
        print("Gemte individuelle plots for hver måling.")

    plt.show()


if __name__ == "__main__":
    main()