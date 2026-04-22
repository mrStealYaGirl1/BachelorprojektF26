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

ANGLE_AXIS = "gz"

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
# DATA PREP
# =========================

def prepare_data(df: pd.DataFrame, impact_us: int) -> pd.DataFrame:
    df = df.copy()

    df = df.sort_values("seq").reset_index(drop=True)

    impact_ms = impact_us / 1000.0
    df["t"] = (df["ts_ms"] - impact_ms) / 1000.0
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
# META TIL RELATIVE TIDER
# =========================

def compute_phase_times(meta_row: pd.Series):
    impact_us = meta_row["impact_us"]

    def rel_s(us_value):
        if pd.isna(us_value) or us_value == 0:
            return None
        return (us_value - impact_us) / 1_000_000.0

    return {
        "address_t": rel_s(meta_row["address_start_us"]),
        "backswing_t": rel_s(meta_row["backswing_start_us"]),
        "forward_t": rel_s(meta_row["forward_start_us"]),
        "impact_t": 0.0,
        "follow_t": rel_s(meta_row["follow_start_us"]),
        "end_t": rel_s(meta_row["end_us"]),
        "event_start_t": rel_s(meta_row["event_start_us"]),
        "event_end_t": rel_s(meta_row["event_end_us"]),
    }

# =========================
# FARVEOMRÅDER
# =========================

def add_phase_spans(ax, phase_times):
    address_t = phase_times["address_t"]
    backswing_t = phase_times["backswing_t"]
    forward_t = phase_times["forward_t"]
    impact_t = phase_times["impact_t"]
    follow_t = phase_times["follow_t"]
    end_t = phase_times["end_t"]

    if address_t is not None and backswing_t is not None and address_t < backswing_t:
        ax.axvspan(address_t, backswing_t, alpha=0.10, color="gray", label="Address")

    if backswing_t is not None and forward_t is not None and backswing_t < forward_t:
        ax.axvspan(backswing_t, forward_t, alpha=0.12, color="red", label="Backswing")

    if forward_t is not None and impact_t is not None and forward_t < impact_t:
        ax.axvspan(forward_t, impact_t, alpha=0.12, color="gold", label="Forward swing")

    if follow_t is not None and end_t is not None and follow_t < end_t:
        ax.axvspan(follow_t, end_t, alpha=0.10, color="blue", label="Follow-through")

# =========================
# LODRETTE LINJER
# =========================

def add_phase_lines(ax, phase_times):
    lines = [
        ("Address", phase_times["address_t"], "--"),
        ("Backswing", phase_times["backswing_t"], "--"),
        ("Forward", phase_times["forward_t"], "--"),
        ("Impact", phase_times["impact_t"], "-"),
        ("Follow", phase_times["follow_t"], "--"),
        ("End", phase_times["end_t"], "--"),
    ]

    for label, t, style in lines:
        if t is not None:
            ax.axvline(t, linestyle=style, linewidth=1.5, label=label)

# =========================
# MAIN
# =========================

def main():
    if len(sys.argv) < 3:
        print("Brug: python visualize_swing.py <imu_csv> <meta_csv> [event_id]")
        return

    imu_csv = sys.argv[1]
    meta_csv = sys.argv[2]
    selected_event_id = int(sys.argv[3]) if len(sys.argv) >= 4 else None

    print(f"Indlæser IMU-fil:  {imu_csv}")
    print(f"Indlæser META-fil: {meta_csv}")

    imu_df = pd.read_csv(imu_csv)
    meta_df = pd.read_csv(meta_csv)

    if imu_df.empty:
        print("IMU-filen er tom")
        return

    if meta_df.empty:
        print("META-filen er tom")
        return

    available_events = sorted(imu_df["event_id"].unique())

    if selected_event_id is None:
        selected_event_id = available_events[0]

    if selected_event_id not in available_events:
        print(f"Event {selected_event_id} findes ikke i IMU-filen")
        print("Tilgængelige event_id:", available_events)
        return

    imu_event = imu_df[imu_df["event_id"] == selected_event_id].copy()
    meta_event = meta_df[meta_df["event_id"] == selected_event_id].copy()

    if meta_event.empty:
        print(f"Ingen META fundet for event {selected_event_id}")
        return

    meta_row = meta_event.iloc[0]

    sample_rate_hz = float(meta_row["sample_rate_hz"])
    total_samples = int(meta_row["total_samples"])
    pre_samples = int(meta_row["pre_samples"])
    post_samples = int(meta_row["post_samples"])
    impact_index_in_event = int(meta_row["impact_index_in_event"])
    impact_us = int(meta_row["impact_us"])

    print(f"Valgt event_id: {selected_event_id}")
    print(f"Samples i IMU-fil: {len(imu_event)}")
    print(f"Forventede samples ifølge META: {total_samples}")
    print(f"Sample rate: {sample_rate_hz} Hz")
    print(f"Impact index i event: {impact_index_in_event}")
    print(f"Impact tid [us]: {impact_us}")

    imu_event = remove_duplicate_samples(imu_event)
    imu_event = prepare_data(imu_event, impact_us)

    phase_times = compute_phase_times(meta_row)

    print("\nFasetider relativt til impact:")
    for key, value in phase_times.items():
        if value is not None:
            print(f"{key}: {value:.3f} s")

    tempo = (phase_times['forward_t'] - phase_times['backswing_t']) / (phase_times['impact_t'] - phase_times['forward_t'])
    print(f"Tempo (backswing til forward / forward til impact): {tempo}")

    print("\nDEBUG:")
    print(f"imu t min/max: {imu_event['t'].min():.3f} / {imu_event['t'].max():.3f}")
    print(f"event_start_t: {phase_times['event_start_t']}")
    print(f"event_end_t:   {phase_times['event_end_t']}")
    print(f"impact_t:      {phase_times['impact_t']}")

    # Peaks
    acc_peak_idx = imu_event["acc_mag_g"].idxmax()
    gyro_peak_idx = imu_event["gyro_mag_dps"].idxmax()

    acc_peak_t = imu_event.loc[acc_peak_idx, "t"]
    gyro_peak_t = imu_event.loc[gyro_peak_idx, "t"]

    # Masker til vinkel-plot
    forward_t = phase_times["forward_t"]

    if forward_t is not None:
        back_mask = imu_event["t"] < forward_t
        forward_mask = imu_event["t"] >= forward_t
    else:
        back_mask = imu_event["t"] <= 0
        forward_mask = imu_event["t"] > 0

    # Crop
    plot_df = imu_event.copy()
    if ENABLE_CROP:
        plot_df = plot_df[plot_df["t"] <= CROP_AFTER_IMPACT_SEC].copy()

    # maskerne skal passe til plot_df
    if forward_t is not None:
        back_mask_plot = plot_df["t"] < forward_t
        forward_mask_plot = plot_df["t"] >= forward_t
    else:
        back_mask_plot = plot_df["t"] <= 0
        forward_mask_plot = plot_df["t"] > 0

    # Maks vinkel i det viste plot-interval
    angle_peak_idx = plot_df["angle_deg"].idxmax()
    angle_peak_t = plot_df.loc[angle_peak_idx, "t"]
    angle_peak_deg = plot_df.loc[angle_peak_idx, "angle_deg"]

    print(f"Maks vinkel: {angle_peak_deg:.2f} deg ved t = {angle_peak_t:.3f} s")


    # Figur
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    # ----------------------
    # VINKEL
    # ----------------------
    axes[0].plot(
        plot_df["t"][back_mask_plot],
        plot_df["angle_deg"][back_mask_plot],
        color="red",
        linewidth=2,
        label="Backswing"
    )
    axes[0].plot(
        plot_df["t"][forward_mask_plot],
        plot_df["angle_deg"][forward_mask_plot],
        color="blue",
        linewidth=2,
        label="Forward/Follow"
    )

    add_phase_spans(axes[0], phase_times)
    add_phase_lines(axes[0], phase_times)

    axes[0].axvline(acc_peak_t, linestyle="--", linewidth=1.2, label="Acc peak")
    axes[0].axvline(gyro_peak_t, linestyle=":", linewidth=1.2, label="Gyro peak")

    # Markér maks vinkel
    axes[0].plot(angle_peak_t, angle_peak_deg, "ko", label="Maks vinkel")
    axes[0].annotate(
        f"Maks: {angle_peak_deg:.1f}°",
        xy=(angle_peak_t, angle_peak_deg),
        xytext=(-110, -10),
        textcoords="offset points",
        fontsize=10,
        bbox=dict(boxstyle="round", fc="white", alpha=0.8),
        arrowprops=dict(arrowstyle="->")
    )

    axes[0].set_ylabel("Angle [deg]")
    axes[0].set_title(f"Golf-event analyse (event {selected_event_id}({imu_csv}))")
    axes[0].grid(True)
    axes[0].legend(loc="upper right", ncol=2)

    # ----------------------
    # GYRO
    # ----------------------
    axes[1].plot(plot_df["t"], plot_df["gx_dps"], alpha=0.8, label="gx")
    axes[1].plot(plot_df["t"], plot_df["gy_dps"], alpha=0.8, label="gy")
    axes[1].plot(plot_df["t"], plot_df["gz_dps"], alpha=0.8, label="gz")
    axes[1].plot(plot_df["t"], plot_df["gyro_mag_dps"], linewidth=2, label="|gyro|")

    add_phase_spans(axes[1], phase_times)
    add_phase_lines(axes[1], phase_times)

    axes[1].set_ylabel("Gyro [dps]")
    axes[1].grid(True)
    axes[1].legend(loc="upper right", ncol=2)

    # ----------------------
    # ACC
    # ----------------------
    axes[2].plot(plot_df["t"], plot_df["ax_g"], alpha=0.8, label="ax")
    axes[2].plot(plot_df["t"], plot_df["ay_g"], alpha=0.8, label="ay")
    axes[2].plot(plot_df["t"], plot_df["az_g"], alpha=0.8, label="az")
    axes[2].plot(plot_df["t"], plot_df["acc_mag_g"], linewidth=2, label="|acc|")

    add_phase_spans(axes[2], phase_times)
    add_phase_lines(axes[2], phase_times)

    axes[2].axvline(acc_peak_t, linestyle="--", linewidth=1.2, label="Acc peak")

    axes[2].set_xlabel("Tid relativt til impact [s]")
    axes[2].set_ylabel("Acceleration [g]")
    axes[2].grid(True)
    axes[2].legend(loc="upper right", ncol=2)

    meta_x_min = phase_times["event_start_t"]
    meta_x_max = phase_times["event_end_t"]

    data_x_min = float(plot_df["t"].min())
    data_x_max = float(plot_df["t"].max())

    if meta_x_min is None:
        meta_x_min = data_x_min
    if meta_x_max is None:
        meta_x_max = data_x_max

    x_min = min(meta_x_min, data_x_min)
    x_max = max(meta_x_max, data_x_max)

    if ENABLE_CROP:
        x_max = min(x_max, CROP_AFTER_IMPACT_SEC)

    if x_max <= x_min:
        print("Advarsel: ugyldige x-limits fra meta, bruger data-baserede grænser i stedet.")
        x_min = data_x_min
        x_max = data_x_max

    for ax in axes:
        ax.set_xlim(x_min, x_max)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()