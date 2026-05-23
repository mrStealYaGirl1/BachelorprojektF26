from pathlib import Path
from io import StringIO
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


R_SHUNT_OHM = 1.0


def load_waveforms_csv(path: Path) -> pd.DataFrame:
    with open(path, "r", encoding="latin1", errors="ignore") as f:
        lines = f.readlines()

    header_line = None
    for i, line in enumerate(lines):
        if line.strip().startswith("Time"):
            header_line = i
            break

    if header_line is None:
        raise ValueError(f"Kunne ikke finde header i {path.name}")

    df = pd.read_csv(StringIO("".join(lines[header_line:])))
    df = df.iloc[:, :2].copy()
    df.columns = ["time_s", "voltage_v"]

    df["time_s"] = df["time_s"] - df["time_s"].iloc[0]
    return df


def classify_file(name: str) -> str:
    n = name.lower()

    if "opstart" in n:
        return "opstart"
    if "advertising" in n:
        return "BLE advertising"
    if "connection" in n:
        return "BLE forbindelse"
    if "ble_send" in n or "ble" in n:
        return "BLE transmission"
    if "idle" in n:
        return "idle"

    return "ukendt"


def analyze_file(path: Path, zero_mode: str) -> dict:
    df = load_waveforms_csv(path)

    v = df["voltage_v"].to_numpy()

    if zero_mode == "none":
        offset = 0.0
    elif zero_mode == "median":
        offset = np.median(v)
    elif zero_mode == "first":
        n = max(10, int(0.05 * len(v)))
        offset = np.mean(v[:n])
    else:
        raise ValueError("zero_mode skal være none, median eller first")

    current_ma = ((v - offset) / R_SHUNT_OHM) * 1000.0

    df["current_ma"] = current_ma

    abs_v_max = np.max(np.abs(v))
    saturated = abs_v_max > 2.5

    return {
        "file": path.name,
        "type": classify_file(path.name),
        "duration_s": df["time_s"].iloc[-1],
        "mean_ma": np.mean(current_ma),
        "median_ma": np.median(current_ma),
        "rms_ma": np.sqrt(np.mean(current_ma ** 2)),
        "min_ma": np.min(current_ma),
        "max_ma": np.max(current_ma),
        "p95_ma": np.percentile(current_ma, 95),
        "p99_ma": np.percentile(current_ma, 99),
        "peak_to_peak_ma": np.max(current_ma) - np.min(current_ma),
        "offset_v": offset,
        "saturated": saturated,
        "df": df,
    }


def save_single_plot(result: dict, out_dir: Path) -> None:
    df = result["df"]

    plt.figure(figsize=(10, 5))
    plt.plot(df["time_s"], df["current_ma"])
    plt.xlabel("Tid [s]")
    plt.ylabel("Strøm [mA]")
    plt.title(f"Strømforbrug — {result['file']}")
    plt.grid(True)
    plt.tight_layout()

    out = out_dir / f"{Path(result['file']).stem}_current.png"
    plt.savefig(out, dpi=200)
    plt.close()


def save_combined_plot(results: list[dict], out_dir: Path) -> None:
    plt.figure(figsize=(10, 6))

    for r in results:
        if r["saturated"]:
            continue

        df = r["df"]
        plt.plot(df["time_s"], df["current_ma"], label=Path(r["file"]).stem)

    plt.xlabel("Tid [s]")
    plt.ylabel("Strøm [mA]")
    plt.title("Sammenligning af strømforbrug")
    plt.grid(True)
    plt.legend(fontsize=8)
    plt.tight_layout()

    out = out_dir / "current_comparison.png"
    plt.savefig(out, dpi=200)
    plt.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("files", nargs="+", help="WaveForms CSV-filer")
    parser.add_argument("--save-plots", action="store_true")
    parser.add_argument(
        "--zero",
        choices=["none", "median", "first"],
        default="none",
        help="Offset-korrektion. Brug typisk none først."
    )

    args = parser.parse_args()

    paths = [Path(f) for f in args.files]
    out_dir = Path(__file__).parent

    results = []
    for path in paths:
        results.append(analyze_file(path, args.zero))

    summary = pd.DataFrame([
        {k: v for k, v in r.items() if k != "df"}
        for r in results
    ])

    print("\nStrømforbrug - summary:")
    print(summary.to_string(index=False, float_format=lambda x: f"{x:.3f}"))

    out_summary = out_dir / "current_summary.csv"
    summary.to_csv(out_summary, index=False)

    print(f"\nGemt summary: {out_summary}")

    saturated = summary[summary["saturated"] == True]
    if len(saturated) > 0:
        print("\nOBS: Følgende målinger ser ud til at være clippet/saturated og bør vurderes kritisk:")
        for f in saturated["file"]:
            print(f"- {f}")

    if args.save_plots:
        for r in results:
            save_single_plot(r, out_dir)

        save_combined_plot(results, out_dir)
        print("\nPlots gemt i:", out_dir)


if __name__ == "__main__":
    main()