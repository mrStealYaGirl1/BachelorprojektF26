from pathlib import Path
import argparse
import numpy as np
import pandas as pd
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


def fit_deg_per_min(t: np.ndarray, angle: np.ndarray) -> float:
    coeffs = np.polyfit(t, angle, 1)
    return coeffs[0] * 60.0


def analyze_window(df: pd.DataFrame, start_s: float, end_s: float) -> dict:
    w = df[(df["t_s"] >= start_s) & (df["t_s"] < end_s)].copy()
    if len(w) < 10:
        return {
            "start_s": start_s,
            "end_s": end_s,
            "samples": len(w),
            "gx_mean_dps": np.nan,
            "gy_mean_dps": np.nan,
            "gz_mean_dps": np.nan,
            "gx_drift_deg_per_min": np.nan,
            "gy_drift_deg_per_min": np.nan,
            "gz_drift_deg_per_min": np.nan,
        }

    t = w["t_s"].to_numpy()
    t = t - t[0]

    gx = w["gx_corr"].to_numpy()
    gy = w["gy_corr"].to_numpy()
    gz = w["gz_corr"].to_numpy()

    ax = integrate_angle(t, gx)
    ay = integrate_angle(t, gy)
    az = integrate_angle(t, gz)

    return {
        "start_s": start_s,
        "end_s": end_s,
        "samples": len(w),
        "gx_mean_dps": np.mean(gx),
        "gy_mean_dps": np.mean(gy),
        "gz_mean_dps": np.mean(gz),
        "gx_drift_deg_per_min": fit_deg_per_min(t, ax),
        "gy_drift_deg_per_min": fit_deg_per_min(t, ay),
        "gz_drift_deg_per_min": fit_deg_per_min(t, az),
    }


def main() -> None:
    print("Window analysis starter...")
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_file", help="CSV-fil")
    parser.add_argument("--window", type=float, default=60.0, help="Vindueslængde i sekunder")
    args = parser.parse_args()

    path = Path(args.csv_file)
    df = load_csv(path)

    total_s = df["t_s"].iloc[-1]
    window = args.window

    rows = []
    start = 0.0
    while start < total_s:
        end = min(start + window, total_s + 1e-9)
        rows.append(analyze_window(df, start, end))
        start += window

    result_df = pd.DataFrame(rows)
    print(result_df.to_string(index=False, float_format=lambda x: f"{x:.6f}"))

    out = path.with_name(path.stem + f"_window_{int(window)}s_analysis.csv")
    result_df.to_csv(out, index=False)
    print(f"\nGemt: {out}")


if __name__ == "__main__":
    main()