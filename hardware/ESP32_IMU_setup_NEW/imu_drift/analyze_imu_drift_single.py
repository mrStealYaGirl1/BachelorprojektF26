from importlib.resources import path
from pathlib import Path
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def load_csv(path: Path) -> pd.DataFrame:
    required = ["t_us", "gx_raw", "gy_raw", "gz_raw", "gx_corr", "gy_corr", "gz_corr"]

    header_line = None
    with open(path, "r", encoding="latin1", errors="ignore") as f:
        lines = f.readlines()

    for i, line in enumerate(lines):
        if line.strip().startswith("t_us,gx_raw,gy_raw,gz_raw,gx_corr,gy_corr,gz_corr"):
            header_line = i
            break

    if header_line is None:
        raise ValueError("Kunne ikke finde CSV-headeren.")

    df = pd.read_csv(path, skiprows=header_line, encoding="latin1")

    df = df[required].copy()
    df["t_s"] = (df["t_us"] - df["t_us"].iloc[0]) / 1e6
    return df

def integrate_angle(time_s: np.ndarray, rate_dps: np.ndarray) -> np.ndarray:
    dt = np.diff(time_s, prepend=time_s[0])
    dt[0] = 0.0
    return np.cumsum(rate_dps * dt)


def drift_rate_deg_per_min(time_s: np.ndarray, angle_deg: np.ndarray) -> float:
    if len(time_s) < 2:
        return float("nan")
    coeffs = np.polyfit(time_s, angle_deg, 1)
    slope_deg_per_s = coeffs[0]
    return slope_deg_per_s * 60.0


def summarize_axis(name: str, rate: pd.Series, angle: np.ndarray, time_s: np.ndarray) -> dict:
    return {
        "axis": name,
        "mean_dps": rate.mean(),
        "std_dps": rate.std(ddof=1),
        "min_dps": rate.min(),
        "max_dps": rate.max(),
        "final_angle_deg": angle[-1],
        "drift_deg_per_min_fit": drift_rate_deg_per_min(time_s, angle),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_file", type=str, help="Path til CSV-fil")
    parser.add_argument("--save-plots", action="store_true", help="Gem plots som PNG")
    args = parser.parse_args()

    path = Path(args.csv_file)
    df = load_csv(path)

    t = df["t_s"].to_numpy()

    gx = df["gx_corr"]
    gy = df["gy_corr"]
    gz = df["gz_corr"]

    angle_x = integrate_angle(t, gx.to_numpy())
    angle_y = integrate_angle(t, gy.to_numpy())
    angle_z = integrate_angle(t, gz.to_numpy())

    summaries = [
        summarize_axis("gx", gx, angle_x, t),
        summarize_axis("gy", gy, angle_y, t),
        summarize_axis("gz", gz, angle_z, t),
    ]
    summary_df = pd.DataFrame(summaries)

    print(f"\nFil: {path.name}")
    print(f"Antal samples: {len(df)}")
    print(f"Varighed: {t[-1]:.2f} s")
    if len(t) > 1:
        fs = 1.0 / np.mean(np.diff(t))
        print(f"Estimeret sample rate: {fs:.2f} Hz")

    print("\nBias-korrigeret gyrostatistik:")
    print(summary_df.to_string(index=False, float_format=lambda x: f"{x:.6f}"))

    fig1 = plt.figure(figsize=(10, 6))
    plt.plot(t, gx, label="gx_corr")
    plt.plot(t, gy, label="gy_corr")
    plt.plot(t, gz, label="gz_corr")
    plt.xlabel("Tid [s]")
    plt.ylabel("Gyro [dps]")
    plt.title(f"Bias-korrigeret gyro — {path.name}")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    fig2 = plt.figure(figsize=(10, 6))
    plt.plot(t, angle_x, label="angle_x")
    plt.plot(t, angle_y, label="angle_y")
    plt.plot(t, angle_z, label="angle_z")
    plt.xlabel("Tid [s]")
    plt.ylabel("Integreret vinkel [deg]")
    plt.title(f"Integreret drift — {path.name}")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    if args.save_plots:
        out_dir = Path(__file__).parent

        out_gyro = out_dir / f"{path.stem}_gyro.png"
        out_drift = out_dir / f"{path.stem}_drift.png"
        out_summary = out_dir / f"{path.stem}_comparison_summary.csv"

        fig1.savefig(out_gyro, dpi=200)
        fig2.savefig(out_drift, dpi=200)


        summary_df.to_csv(out_summary, index=False)

        print(f"Gemte summary: {out_summary}")
        print(f"Gemte plots:")
        print(f"- {out_gyro}")
        print(f"- {out_drift}")
        
        plt.show()


if __name__ == "__main__":
    main()