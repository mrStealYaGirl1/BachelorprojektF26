import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# =========================
# DETECTION THRESHOLDS
# =========================

GYRO_ADDRESS_THRESHOLD = 5
GYRO_BACKSWING_THRESHOLD = 20
GYRO_FORWARD_THRESHOLD = 40

ACC_IMPACT_THRESHOLD = 1.4

ADDRESS_MIN_TIME = 0.5

# =========================
# INDSTILLINGER
# =========================

ACC_RANGE_G = 2.0
GYRO_RANGE_DPS = 2000.0
RAW_MAX = 32768.0

ANGLE_AXIS = "gz"

PRE_SAMPLES = 600
POST_SAMPLES = 400
EXPECTED_SAMPLES = PRE_SAMPLES + POST_SAMPLES
SAMPLE_RATE_HZ = 200.0

CROP_AFTER_IMPACT_SEC = 1.5
ENABLE_CROP = False

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

    df = df.drop_duplicates()

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

    df = df.sort_values("seq").reset_index(drop=True)

    df["t"] = (df["seq"] - PRE_SAMPLES) / SAMPLE_RATE_HZ
    df["dt"] = df["t"].diff().fillna(0.0)

    df["ax_g"] = raw_acc_to_g(df["ax"])
    df["ay_g"] = raw_acc_to_g(df["ay"])
    df["az_g"] = raw_acc_to_g(df["az"])

    df["acc_mag_g"] = np.sqrt(
        df["ax_g"]**2 +
        df["ay_g"]**2 +
        df["az_g"]**2
    )

    df["gx_dps"] = raw_gyro_to_dps(df["gx"])
    df["gy_dps"] = raw_gyro_to_dps(df["gy"])
    df["gz_dps"] = raw_gyro_to_dps(df["gz"])

    df["gyro_mag_dps"] = np.sqrt(
        df["gx_dps"]**2 +
        df["gy_dps"]**2 +
        df["gz_dps"]**2
    )

    rate_col = f"{ANGLE_AXIS}_dps"
    angle_deg = [0.0]

    for i in range(1, len(df)):
        new_angle = angle_deg[-1] + df.iloc[i][rate_col] * df.iloc[i]["dt"]
        angle_deg.append(new_angle)

    df["angle_deg"] = angle_deg

    return df

# =========================
# STATE DETECTION
# =========================

def detect_swing_states(df: pd.DataFrame):

    state = "IDLE"
    states = []

    still_counter = 0
    dt = 1.0 / SAMPLE_RATE_HZ

    for _, row in df.iterrows():

        g = row["gyro_mag_dps"]
        a = row["acc_mag_g"]

        if state == "IDLE":

            if g < GYRO_ADDRESS_THRESHOLD:
                still_counter += 1
            else:
                still_counter = 0

            if still_counter * dt > ADDRESS_MIN_TIME:
                state = "ADDRESS"

        elif state == "ADDRESS":

            if g > GYRO_BACKSWING_THRESHOLD:
                state = "BACKSWING"

        elif state == "BACKSWING":

            if g > GYRO_FORWARD_THRESHOLD:
                state = "FORWARD"

            if g < GYRO_ADDRESS_THRESHOLD:
                state = "IDLE"

        elif state == "FORWARD":

            if a > ACC_IMPACT_THRESHOLD:
                state = "IMPACT"

        elif state == "IMPACT":

            state = "FOLLOW"

        states.append(state)

    df["state"] = states

    return df

# =========================
# FIND TOP OF BACKSWING
# =========================

def find_backswing_top(df: pd.DataFrame, angle_axis="gz"):

    rate_col = f"{angle_axis}_dps"

    gyro = df[rate_col].values
    t = df["t"].values

    pre_mask = t < 0
    pre_idx = np.where(pre_mask)[0]

    if len(pre_idx) < 2:
        return None, None

    gyro_pre = gyro[pre_idx]

    sign_change = np.where(np.diff(np.sign(gyro_pre)) != 0)[0]

    if len(sign_change) == 0:
        return None, None

    idx_rel = sign_change[-1]
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

    # ----------------------
    # STATE DETECTION
    # ----------------------

    df = detect_swing_states(df)

    # ----------------------
    # STATE PLOT
    # ----------------------

    state_map = {
        "IDLE":0,
        "ADDRESS":1,
        "BACKSWING":2,
        "FORWARD":3,
        "IMPACT":4,
        "FOLLOW":5
    }

    df["state_num"] = df["state"].map(state_map)

    plt.figure(figsize=(10,3))
    plt.plot(df["t"], df["state_num"])
    plt.title("Swing state")
    plt.xlabel("time")
    plt.ylabel("state")
    plt.show()

    # ----------------------
    # IMPACT
    # ----------------------

    impact_t = 0.0

    acc_peak_idx = df["acc_mag_g"].idxmax()
    gyro_peak_idx = df["gyro_mag_dps"].idxmax()

    acc_peak_t = df.loc[acc_peak_idx, "t"]
    gyro_peak_t = df.loc[gyro_peak_idx, "t"]

    # ----------------------
    # BACKSWING TOP
    # ----------------------

    backswing_top_idx, backswing_top_t = find_backswing_top(df, ANGLE_AXIS)

    if backswing_top_t is None:
        backswing_top_t = -0.2
    else:
        print(f"Top af backswing ved t = {backswing_top_t:.3f} s")

    # ----------------------
    # MASKER
    # ----------------------

    back_mask = df["t"] <= backswing_top_t
    forward_mask = df["t"] > backswing_top_t

    # ----------------------
    # FIGUR
    # ----------------------

    fig, axes = plt.subplots(3,1,figsize=(12,10),sharex=True)

    # VINKEL

    axes[0].plot(df["t"][back_mask],df["angle_deg"][back_mask],color="red",linewidth=2,label="Backswing")
    axes[0].plot(df["t"][forward_mask],df["angle_deg"][forward_mask],color="blue",linewidth=2,label="Forward")

    axes[0].axvline(backswing_top_t,color="purple",linestyle="--")
    axes[0].axvline(impact_t,color="black")
    axes[0].axvline(acc_peak_t,color="gray",linestyle="--")
    axes[0].axvline(gyro_peak_t,color="gray",linestyle=":")

    axes[0].set_ylabel("Angle [deg]")
    axes[0].grid(True)
    axes[0].legend()

    # GYRO

    axes[1].plot(df["t"],df["gx_dps"],alpha=0.8,label="gx")
    axes[1].plot(df["t"],df["gy_dps"],alpha=0.8,label="gy")
    axes[1].plot(df["t"],df["gz_dps"],alpha=0.8,label="gz")

    axes[1].plot(df["t"][back_mask],df["gyro_mag_dps"][back_mask],color="red",linewidth=2,label="|gyro| back")
    axes[1].plot(df["t"][forward_mask],df["gyro_mag_dps"][forward_mask],color="blue",linewidth=2,label="|gyro| forward")

    axes[1].set_ylabel("Gyro [dps]")
    axes[1].grid(True)
    axes[1].legend()

    # ACC

    axes[2].plot(df["t"],df["ax_g"],alpha=0.8,label="ax")
    axes[2].plot(df["t"],df["ay_g"],alpha=0.8,label="ay")
    axes[2].plot(df["t"],df["az_g"],alpha=0.8,label="az")

    axes[2].plot(df["t"][back_mask],df["acc_mag_g"][back_mask],color="red",linewidth=2,label="|acc| back")
    axes[2].plot(df["t"][forward_mask],df["acc_mag_g"][forward_mask],color="blue",linewidth=2,label="|acc| forward")

    axes[2].axvline(acc_peak_t,color="gray",linestyle="--")

    axes[2].set_xlabel("Tid [s]")
    axes[2].set_ylabel("Acceleration [g]")
    axes[2].grid(True)
    axes[2].legend()

    axes[2].set_xlim(-PRE_SAMPLES / SAMPLE_RATE_HZ, POST_SAMPLES / SAMPLE_RATE_HZ)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()