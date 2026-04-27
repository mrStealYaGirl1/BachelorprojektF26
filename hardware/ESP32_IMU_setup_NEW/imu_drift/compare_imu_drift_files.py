from pathlib import Path
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from io import StringIO


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

    csv_text = "".join(lines[header_line:])
    df = pd.read_csv(StringIO(csv_text))

    missing = [c for c in REQUIRED if c not in df.columns]
    if missing:
        raise ValueError(f"Mangler kolonner i {path.name}: {missing}")

    df = df[REQUIRED].copy()
    df["t_s"] = (df["t_us"] - df["t_us"].iloc[0]) / 1e6
    return df


def integrate_angle(t: np.ndarray, rate: np.ndarray) -> np.ndarray:
    dt = np.diff(t, prepend=t[0])
    dt[0] = 0.0
    return np.cumsum(rate * dt)


def fit_drift_deg_per_min(t: np.ndarray, angle: np.ndarray) -> float:
    coeffs = np.polyfit(t, angle, 1)
    return coeffs[0] * 60.0


def analyze_file(path: Path) -> dict:
    df = load_csv(path)
    t = df["t_s"].to_numpy()

    gx = df["gx_corr"].to_numpy()
    gy = df["gy_corr"].to_numpy()
    gz = df["gz_corr"].to_numpy()

    ax = integrate_angle(t, gx)
    ay = integrate_angle(t, gy)
    az = integrate_angle(t, gz)

    return {
        "name": path.stem,
        "df": df,
        "t": t,
        "gx": gx,
        "gy": gy,
        "gz": gz,
        "ax": ax,
        "ay": ay,
        "az": az,
        "summary": {
            "file": path.name,
            "duration_s": t[-1],
            "gx_mean_dps": np.mean(gx),
            "gy_mean_dps": np.mean(gy),
            "gz_mean_dps": np.mean(gz),
            "gx_std_dps": np.std(gx, ddof=1),
            "gy_std_dps": np.std(gy, ddof=1),
            "gz_std_dps": np.std(gz, ddof=1),
            "gx_drift_deg_per_min": fit_drift_deg_per_min(t, ax),
            "gy_drift_deg_per_min": fit_drift_deg_per_min(t, ay),
            "gz_drift_deg_per_min": fit_drift_deg_per_min(t, az),
            "gx_final_angle_deg": ax[-1],
            "gy_final_angle_deg": ay[-1],
            "gz_final_angle_deg": az[-1],
        }
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("files", nargs="+", help="CSV-filer")
    parser.add_argument("--save-plots", action="store_true")
    args = parser.parse_args()

    results = [analyze_file(Path(f)) for f in args.files]
    summary_df = pd.DataFrame([r["summary"] for r in results])

    out_dir = Path(__file__).parent

    print("\nSammenligning:")
    print(summary_df.to_string(index=False, float_format=lambda x: f"{x:.6f}"))

    # Corr data sammenligning, gx/gy/gz i samme figur
    fig_corr, axes_corr = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

    for r in results:
        axes_corr[0].plot(r["t"], r["gx"], label=r["name"])
        axes_corr[1].plot(r["t"], r["gy"], label=r["name"])
        axes_corr[2].plot(r["t"], r["gz"], label=r["name"])

    axes_corr[0].set_ylabel("gx_corr [dps]")
    axes_corr[0].set_title("Sammenligning af gx_corr")
    axes_corr[0].grid(True)
    axes_corr[0].legend()

    axes_corr[1].set_ylabel("gy_corr [dps]")
    axes_corr[1].set_title("Sammenligning af gy_corr")
    axes_corr[1].grid(True)
    axes_corr[1].legend()

    axes_corr[2].set_ylabel("gz_corr [dps]")
    axes_corr[2].set_title("Sammenligning af gz_corr")
    axes_corr[2].set_xlabel("Tid [s]")
    axes_corr[2].grid(True)
    axes_corr[2].legend()

    plt.tight_layout()

    # Integreret drift sammenligning, x/y/z i samme figur
    fig_drift, axes_drift = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

    for r in results:
        axes_drift[0].plot(r["t"], r["ax"], label=r["name"])
        axes_drift[1].plot(r["t"], r["ay"], label=r["name"])
        axes_drift[2].plot(r["t"], r["az"], label=r["name"])

    axes_drift[0].set_ylabel("X-vinkel [deg]")
    axes_drift[0].set_title("Sammenligning af integreret x-drift")
    axes_drift[0].grid(True)
    axes_drift[0].legend()

    axes_drift[1].set_ylabel("Y-vinkel [deg]")
    axes_drift[1].set_title("Sammenligning af integreret y-drift")
    axes_drift[1].grid(True)
    axes_drift[1].legend()

    axes_drift[2].set_ylabel("Z-vinkel [deg]")
    axes_drift[2].set_title("Sammenligning af integreret z-drift")
    axes_drift[2].set_xlabel("Tid [s]")
    axes_drift[2].grid(True)
    axes_drift[2].legend()

    plt.tight_layout()

    if args.save_plots:
        fig_corr.savefig(out_dir / "compare_xyz_corr.png", dpi=200)
        fig_drift.savefig(out_dir / "compare_xyz_drift.png", dpi=200)
        summary_df.to_csv(out_dir / "imu_drift_comparison_summary.csv", index=False)

        print("\nGemte filer:")
        print(f"- {out_dir / 'compare_xyz_corr.png'}")
        print(f"- {out_dir / 'compare_xyz_drift.png'}")
        print(f"- {out_dir / 'imu_drift_comparison_summary.csv'}")

    plt.show()


if __name__ == "__main__":
    main()