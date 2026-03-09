import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# =========================
# INDSTILLINGER
# =========================
CSV_FILE = "golf_event.csv"   # skift hvis filen hedder noget andet

# BMI270 settings
ACC_RANGE_G = 2.0
GYRO_RANGE_DPS = 2000.0
RAW_MAX = 32768.0

# vælg hvilken gyro-akse der bruges til simpel vinkelintegration
ANGLE_AXIS = "gz"   # "gx", "gy" eller "gz"

# =========================
# KONVERTERING
# =========================
def raw_acc_to_g(raw):
    return raw * (ACC_RANGE_G / RAW_MAX)

def raw_gyro_to_dps(raw):
    return raw * (GYRO_RANGE_DPS / RAW_MAX)

# =========================
# DUPLICATE FILTER
# =========================
def remove_duplicate_samples(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy()

    # Fjern helt identiske rækker
    df = df.drop_duplicates()

    # Fjern samples hvor sensordata gentager sig lige efter hinanden
    same_as_prev = (
        (df["ax"] == df["ax"].shift()) &
        (df["ay"] == df["ay"].shift()) &
        (df["az"] == df["az"].shift()) &
        (df["gx"] == df["gx"].shift()) &
        (df["gy"] == df["gy"].shift()) &
        (df["gz"] == df["gz"].shift())
    )

    df = df.loc[~same_as_prev].copy()
    df.reset_index(drop=True, inplace=True)
    return df

# =========================
# BEREGNINGER
# =========================
def prepare_data(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy()

    # tid i sekunder relativt til start
    df["t"] = (df["ts_ms"] - df["ts_ms"].iloc[0]) / 1000.0
    df["dt"] = df["t"].diff().fillna(0.0)

    # acceleration i g
    df["ax_g"] = raw_acc_to_g(df["ax"])
    df["ay_g"] = raw_acc_to_g(df["ay"])
    df["az_g"] = raw_acc_to_g(df["az"])
    df["acc_mag_g"] = np.sqrt(df["ax_g"]**2 + df["ay_g"]**2 + df["az_g"]**2)

    # gyro i dps
    df["gx_dps"] = raw_gyro_to_dps(df["gx"])
    df["gy_dps"] = raw_gyro_to_dps(df["gy"])
    df["gz_dps"] = raw_gyro_to_dps(df["gz"])
    df["gyro_mag_dps"] = np.sqrt(df["gx_dps"]**2 + df["gy_dps"]**2 + df["gz_dps"]**2)

    # simpel vinkel fra valgt gyro-akse
    angle_deg = [0.0]
    rate_col = f"{ANGLE_AXIS}_dps"

    for i in range(1, len(df)):
        new_angle = angle_deg[-1] + df.iloc[i][rate_col] * df.iloc[i]["dt"]
        angle_deg.append(new_angle)

    df["angle_deg"] = angle_deg

    return df

# =========================
# MAIN
# =========================
def main():

    if len(sys.argv) < 2:
        print("Brug: python visualize_golf.py <csv_fil>")
        return

    CSV_FILE = sys.argv[1]

    print(f"Indlæser fil: {CSV_FILE}")

    df = pd.read_csv(CSV_FILE)

    print(f"Oprindelige samples: {len(df)}")
    df = remove_duplicate_samples(df)
    print(f"Efter duplicate-filter: {len(df)}")

    df = prepare_data(df)

    # find peak for acc og gyro
    acc_peak_idx = df["acc_mag_g"].idxmax()
    gyro_peak_idx = df["gyro_mag_dps"].idxmax()

    acc_peak_t = df.loc[acc_peak_idx, "t"]
    gyro_peak_t = df.loc[gyro_peak_idx, "t"]

    # =========================
    # FIGUR MED 3 PLOTS
    # =========================
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    # --- Plot 1: Vinkel ---
    axes[0].plot(df["t"], df["angle_deg"], label=f"Vinkel fra {ANGLE_AXIS}", linewidth=1.5)
    axes[0].axvline(acc_peak_t, linestyle="--", linewidth=1, label="Peak acc")
    axes[0].axvline(gyro_peak_t, linestyle=":", linewidth=1, label="Peak gyro")
    axes[0].set_ylabel("Vinkel [deg]")
    axes[0].set_title(f"Golf-event analyse ({CSV_FILE})")
    axes[0].grid(True)
    axes[0].legend()

    # --- Plot 2: Gyro ---
    axes[1].plot(df["t"], df["gx_dps"], label="gx [dps]", linewidth=1)
    axes[1].plot(df["t"], df["gy_dps"], label="gy [dps]", linewidth=1)
    axes[1].plot(df["t"], df["gz_dps"], label="gz [dps]", linewidth=1)
    axes[1].plot(df["t"], df["gyro_mag_dps"], label="|gyro| [dps]", linewidth=2)
    axes[1].axvline(gyro_peak_t, linestyle=":", linewidth=1, label="Peak gyro")
    axes[1].set_ylabel("Gyro [dps]")
    axes[1].grid(True)
    axes[1].legend()

    # --- Plot 3: Acceleration ---
    axes[2].plot(df["t"], df["ax_g"], label="ax [g]", linewidth=1)
    axes[2].plot(df["t"], df["ay_g"], label="ay [g]", linewidth=1)
    axes[2].plot(df["t"], df["az_g"], label="az [g]", linewidth=1)
    axes[2].plot(df["t"], df["acc_mag_g"], label="|acc| [g]", linewidth=2)
    axes[2].axvline(acc_peak_t, linestyle="--", linewidth=1, label="Peak acc")
    axes[2].set_xlabel("Tid [s]")
    axes[2].set_ylabel("Acceleration [g]")
    axes[2].grid(True)
    axes[2].legend()

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()