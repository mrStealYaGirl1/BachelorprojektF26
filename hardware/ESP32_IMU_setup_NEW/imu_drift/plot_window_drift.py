from pathlib import Path
import argparse
import pandas as pd
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("files", nargs="+", help="Window-analysis CSV-filer")
    parser.add_argument("--save", action="store_true")
    args = parser.parse_args()

    plt.figure(figsize=(10, 6))

    for file in args.files:
        path = Path(file)
        df = pd.read_csv(path)

        t_mid = (df["start_s"] + df["end_s"]) / 2.0 / 60.0  # minutter

        plt.plot(t_mid, df["gx_drift_deg_per_min"], marker="o", label=f"{path.stem} gx")
        plt.plot(t_mid, df["gy_drift_deg_per_min"], marker="o", label=f"{path.stem} gy")
        plt.plot(t_mid, df["gz_drift_deg_per_min"], marker="o", label=f"{path.stem} gz")

    plt.xlabel("Tid [min]")
    plt.ylabel("Drift [°/min]")
    plt.title("Gyrodrift over tid")
    plt.grid(True)
    plt.legend(fontsize=8)
    plt.tight_layout()

    if args.save:
        plt.savefig("imu_drift_window_comparison.png", dpi=200)
        print("Gemt: imu_drift_window_comparison.png")

    plt.show()


if __name__ == "__main__":
    main()