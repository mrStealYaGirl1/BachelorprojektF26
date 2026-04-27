from pathlib import Path
from io import StringIO
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


REQUIRED = ["t_us", "gx_raw", "gy_raw", "gz_raw", "gx_corr", "gy_corr", "gz_corr"]


def load_csv(path: Path) -> pd.DataFrame:
    with open(path, "r", encoding="latin1", errors="ignore") as f:
        lines = f.readlines()

    header_line = None
    for i, line in enumerate(lines):
        if line.strip().startswith("t_us,gx_raw,gy_raw,gz_raw,gx_corr,gy_corr,gz_corr"):
            header_line = i
            break

    if header_line is None:
        raise ValueError(f"Kunne ikke finde CSV-header i {path.name}")

    df = pd.read_csv(StringIO("".join(lines[header_line:])))
    df = df[REQUIRED].copy()
    df["t_s"] = (df["t_us"] - df["t_us"].iloc[0]) / 1e6

    return df


def integrate_angle(t, rate):
    dt = np.diff(t, prepend=t[0])
    dt[0] = 0
    return np.cumsum(rate * dt)


def drift_deg_per_min(t, angle):
    return np.polyfit(t, angle, 1)[0] * 60.0


def analyze_segment(df, name):
    t = df["t_s"].to_numpy()
    t = t - t[0]

    gx = df["gx_corr"].to_numpy()
    gy = df["gy_corr"].to_numpy()
    gz = df["gz_corr"].to_numpy()

    ax = integrate_angle(t, gx)
    ay = integrate_angle(t, gy)
    az = integrate_angle(t, gz)

    return {
        "segment": name,
        "duration_s": t[-1],
        "gx_mean_dps": np.mean(gx),
        "gy_mean_dps": np.mean(gy),
        "gz_mean_dps": np.mean(gz),
        "gx_drift_deg_per_min": drift_deg_per_min(t, ax),
        "gy_drift_deg_per_min": drift_deg_per_min(t, ay),
        "gz_drift_deg_per_min": drift_deg_per_min(t, az),
        "gx_final_angle_deg": ax[-1],
        "gy_final_angle_deg": ay[-1],
        "gz_final_angle_deg": az[-1],
        "t": t,
        "ax": ax,
        "ay": ay,
        "az": az,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_file")
    parser.add_argument("--threshold", type=float, default=10.0, help="gyro_mag threshold [dps]")
    parser.add_argument("--margin", type=float, default=5.0, help="sekunder der fjernes omkring bevægelse")
    parser.add_argument("--save-plots", action="store_true")
    args = parser.parse_args()

    path = Path(args.csv_file)
    df = load_csv(path)

    gyro_mag = np.sqrt(df["gx_corr"]**2 + df["gy_corr"]**2 + df["gz_corr"]**2)
    moving = gyro_mag > args.threshold

    if not moving.any():
        raise ValueError("Fandt ingen tydelig bevægelse. Sænk evt. --threshold.")

    move_start_s = df.loc[moving, "t_s"].iloc[0]
    move_end_s = df.loc[moving, "t_s"].iloc[-1]

    before = df[df["t_s"] < move_start_s - args.margin].copy()
    after = df[df["t_s"] > move_end_s + args.margin].copy()

    if len(before) < 100 or len(after) < 100:
        raise ValueError("For korte stille segmenter. Prøv lavere --margin eller tjek målingen.")

    before_result = analyze_segment(before, "før rotation")
    after_result = analyze_segment(after, "efter rotation")

    summary = pd.DataFrame([
        {k: v for k, v in before_result.items() if k not in ["t", "ax", "ay", "az"]},
        {k: v for k, v in after_result.items() if k not in ["t", "ax", "ay", "az"]},
    ])

    print(f"\nBevægelse fundet ca. fra {move_start_s:.2f} s til {move_end_s:.2f} s")
    print("\nSegmentanalyse:")
    print(summary.to_string(index=False, float_format=lambda x: f"{x:.6f}"))

    out_dir = Path(__file__).parent
    out_summary = out_dir / f"{path.stem}_orientation_split_summary.csv"
    summary.to_csv(out_summary, index=False)

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=False)

    axes[0].plot(before_result["t"], before_result["ax"], label="før rotation")
    axes[0].plot(after_result["t"], after_result["ax"], label="efter rotation")
    axes[0].set_title("X-drift før/efter rotation")
    axes[0].set_ylabel("X-vinkel [deg]")
    axes[0].grid(True)
    axes[0].legend()

    axes[1].plot(before_result["t"], before_result["ay"], label="før rotation")
    axes[1].plot(after_result["t"], after_result["ay"], label="efter rotation")
    axes[1].set_title("Y-drift før/efter rotation")
    axes[1].set_ylabel("Y-vinkel [deg]")
    axes[1].grid(True)
    axes[1].legend()

    axes[2].plot(before_result["t"], before_result["az"], label="før rotation")
    axes[2].plot(after_result["t"], after_result["az"], label="efter rotation")
    axes[2].set_title("Z-drift før/efter rotation")
    axes[2].set_ylabel("Z-vinkel [deg]")
    axes[2].set_xlabel("Tid i segment [s]")
    axes[2].grid(True)
    axes[2].legend()

    plt.tight_layout()

    if args.save_plots:
        out_plot = out_dir / f"{path.stem}_orientation_split_drift.png"
        fig.savefig(out_plot, dpi=200)
        print(f"\nGemt:")
        print(f"- {out_summary}")
        print(f"- {out_plot}")

    plt.show()


if __name__ == "__main__":
    main()