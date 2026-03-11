import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# =========================
# INDSTILLINGER
# =========================
ACC_RANGE_G = 2.0
GYRO_RANGE_DPS = 2000.0
RAW_MAX = 32768.0

ANGLE_AXIS = "gz"   # "gx", "gy" eller "gz"

# Event setup fra ESP32
PRE_SAMPLES = 600
POST_SAMPLES = 400
EXPECTED_SAMPLES = PRE_SAMPLES + POST_SAMPLES
SAMPLE_RATE_HZ = 200.0

# vælg om plottet kun skal vise området tæt på slaget
CROP_AFTER_IMPACT_SEC = 1.5   # vis fx 1.5 sek efter impact
ENABLE_CROP = False           # True = beskær grafen efter impact

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

    # fjern helt identiske rækker
    df = df.drop_duplicates()

    # fjern samples hvor alle sensordata er identiske med forrige sample
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

    # sorter efter seq for sikkerhed
    df = df.sort_values("seq").reset_index(drop=True)

    # tid relativt til impact
    # seq = PRE_SAMPLES -> t = 0
    df["t"] = (df["seq"] - PRE_SAMPLES) / SAMPLE_RATE_HZ
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

    # simpel vinkelintegration fra valgt gyro-akse
    rate_col = f"{ANGLE_AXIS}_dps"
    angle_deg = [0.0]

    for i in range(1, len(df)):
        new_angle = angle_deg[-1] + df.iloc[i][rate_col] * df.iloc[i]["dt"]
        angle_deg.append(new_angle)

    df["angle_deg"] = angle_deg

    return df

# =========================
# FIND TOP OF BACKSWING
# =========================
def find_backswing_top(df: pd.DataFrame, angle_axis: str = "gz"):
    rate_col = f"{angle_axis}_dps"
    gyro = df[rate_col].values
    t = df["t"].values

    # brug kun tiden før impact
    pre_impact_mask = t < 0
    pre_idx = np.where(pre_impact_mask)[0]

    if len(pre_idx) < 2:
        return None, None

    gyro_pre = gyro[pre_idx]

    # find fortegnsskifte i gyro før impact
    sign_change_rel = np.where(np.diff(np.sign(gyro_pre)) != 0)[0]

    if len(sign_change_rel) == 0:
        return None, None

    # vælg sidste fortegnsskifte før impact = mest sandsynlige top af backswing
    idx_rel = sign_change_rel[-1]
    idx_abs = pre_idx[idx_rel + 1]

    return idx_abs, df.loc[idx_abs, "t"]

# =========================
# MAIN
# =========================
def main():
    if len(sys.argv) < 2:
        print("Brug: python visualize_swing.py <csv_fil>")
        return

    csv_file = sys.argv[1]
    print(f"Indlæser fil: {csv_file}")

    df = pd.read_csv(csv_file)

    print(f"Oprindelige samples: {len(df)}")
    df = remove_duplicate_samples(df)
    print(f"Efter duplicate-filter: {len(df)}")

    df = prepare_data(df)

    # impact er defineret som t = 0
    impact_t = 0.0

    # peaks
    acc_peak_idx = df["acc_mag_g"].idxmax()
    gyro_peak_idx = df["gyro_mag_dps"].idxmax()

    acc_peak_t = df.loc[acc_peak_idx, "t"]
    gyro_peak_t = df.loc[gyro_peak_idx, "t"]

    # backswing top
    backswing_top_idx, backswing_top_t = find_backswing_top(df, ANGLE_AXIS)

    # fallback hvis der ikke findes sign change
    if backswing_top_t is None:
        print("Kunne ikke finde tydelig top af backswing.")
        backswing_top_t = -0.2  # fallback
    else:
        print(f"Top af backswing fundet ved t = {backswing_top_t:.3f} s")

    # beskæring hvis ønsket
    if ENABLE_CROP:
        df = df[df["t"] <= CROP_AFTER_IMPACT_SEC].copy()

    # masker til farveopdeling
    back_mask = df["t"] <= backswing_top_t
    forward_mask = df["t"] > backswing_top_t
    
    # =========================
    # FIGUR MED 3 PLOTS
    # =========================
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    # ---------------------------------
    # Plot 1: Vinkel
    # ---------------------------------
    axes[0].plot(df["t"][back_mask], df["angle_deg"][back_mask],
             color="red", linewidth=2, label="Backswing")

    axes[0].plot(df["t"][forward_mask], df["angle_deg"][forward_mask],
             color="blue", linewidth=2, label="Forward swing")


    axes[0].axvline(backswing_top_t, color="purple", linestyle="--", linewidth=1.2, label="Top backswing")
    axes[0].axvline(impact_t, color="black", linestyle="-", linewidth=1.5, label="Impact")
    axes[0].axvline(acc_peak_t, color="gray", linestyle="--", linewidth=1, label="Peak acc")
    axes[0].axvline(gyro_peak_t, color="gray", linestyle=":", linewidth=1, label="Peak gyro")

    axes[0].set_ylabel("Vinkel [deg]")
    axes[0].set_title(f"Golf-event analyse ({csv_file})")
    axes[0].grid(True)
    axes[0].legend(loc="upper right")

    # ---------------------------------
    # Plot 2: Gyro
    # ---------------------------------
    axes[1].plot(df["t"], df["gx_dps"], linewidth=1, alpha=0.8, label="gx [dps]")
    axes[1].plot(df["t"], df["gy_dps"], linewidth=1, alpha=0.8, label="gy [dps]")
    axes[1].plot(df["t"], df["gz_dps"], linewidth=1, alpha=0.8, label="gz [dps]")
   
    axes[1].plot(df["t"][back_mask], df["gyro_mag_dps"][back_mask],
                color="red", linewidth=2.2, label="|gyro| backswing")

    axes[1].plot(df["t"][forward_mask], df["gyro_mag_dps"][forward_mask],
                color="blue", linewidth=2.2, label="|gyro| forward swing")

    axes[1].axvline(backswing_top_t, color="purple", linestyle="--", linewidth=1.2, label="Top backswing")
    axes[1].axvline(impact_t, color="black", linestyle="-", linewidth=1.5, label="Impact")
    axes[1].axvline(gyro_peak_t, color="gray", linestyle=":", linewidth=1, label="Peak gyro")

    axes[1].set_ylabel("Gyro [dps]")
    axes[1].grid(True)
    axes[1].legend(loc="upper right")

    # ---------------------------------
    # Plot 3: Acceleration
    # ---------------------------------
    axes[2].plot(df["t"], df["ax_g"], linewidth=1, alpha=0.8, label="ax [g]")
    axes[2].plot(df["t"], df["ay_g"], linewidth=1, alpha=0.8, label="ay [g]")
    axes[2].plot(df["t"], df["az_g"], linewidth=1, alpha=0.8, label="az [g]")

    axes[2].plot(df["t"][back_mask], df["acc_mag_g"][back_mask],
             color="red", linewidth=2.2, label="|acc| backswing")

    axes[2].plot(df["t"][forward_mask], df["acc_mag_g"][forward_mask],
             color="blue", linewidth=2.2, label="|acc| forward swing")

    axes[2].axvline(backswing_top_t, color="purple", linestyle="--", linewidth=1.2, label="Top backswing")
    axes[2].axvline(impact_t, color="black", linestyle="-", linewidth=1.5, label="Impact")
    axes[2].axvline(acc_peak_t, color="gray", linestyle="--", linewidth=1, label="Peak acc")

    axes[2].set_xlabel("Tid [s]")
    axes[2].set_ylabel("Acceleration [g]")
    axes[2].grid(True)
    axes[2].legend(loc="upper right")

    # fast x-akse
    axes[2].set_xlim(-PRE_SAMPLES / SAMPLE_RATE_HZ, POST_SAMPLES / SAMPLE_RATE_HZ)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()