import sys
from pathlib import Path

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

ACC_RANGE_G = 2.0
GYRO_RANGE_DPS = 2000.0
RAW_MAX = 32768.0

ANGLE_AXIS = "gz"

CROP_AFTER_IMPACT_SEC = 1.5
ENABLE_CROP = False


def raw_acc_to_g(raw):
    return raw * (ACC_RANGE_G / RAW_MAX)


def raw_gyro_to_dps(raw):
    return raw * (GYRO_RANGE_DPS / RAW_MAX)


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


def prepare_data(df, sample_rate_hz, impact_index_in_event):
    df = df.copy()
    df = df.sort_values("seq").reset_index(drop=True)

    df["t"] = (df["seq"] - impact_index_in_event) / sample_rate_hz
    df["dt"] = df["t"].diff().fillna(1.0 / sample_rate_hz)

    df["ax_g"] = raw_acc_to_g(df["ax"])
    df["ay_g"] = raw_acc_to_g(df["ay"])
    df["az_g"] = raw_acc_to_g(df["az"])
    df["acc_mag_g"] = np.sqrt(df["ax_g"]**2 + df["ay_g"]**2 + df["az_g"]**2)

    df["gx_dps"] = raw_gyro_to_dps(df["gx"])
    df["gy_dps"] = raw_gyro_to_dps(df["gy"])
    df["gz_dps"] = raw_gyro_to_dps(df["gz"])
    df["gyro_mag_dps"] = np.sqrt(df["gx_dps"]**2 + df["gy_dps"]**2 + df["gz_dps"]**2)

    rate_col = f"{ANGLE_AXIS}_dps"
    df["angle_deg"] = (df[rate_col] * df["dt"]).cumsum()

    return df


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


def add_phase_spans(ax, phase_times):
    if phase_times["address_t"] is not None and phase_times["backswing_t"] is not None:
        ax.axvspan(phase_times["address_t"], phase_times["backswing_t"], alpha=0.10, color="gray", label="Address")

    if phase_times["backswing_t"] is not None and phase_times["forward_t"] is not None:
        ax.axvspan(phase_times["backswing_t"], phase_times["forward_t"], alpha=0.12, color="red", label="Backswing")

    if phase_times["forward_t"] is not None:
        ax.axvspan(phase_times["forward_t"], 0.0, alpha=0.12, color="gold", label="Forward swing")

    if phase_times["follow_t"] is not None and phase_times["end_t"] is not None:
        ax.axvspan(phase_times["follow_t"], phase_times["end_t"], alpha=0.10, color="blue", label="Follow-through")


def add_phase_lines(ax, phase_times):
    lines = [
        ("Address", phase_times["address_t"], "--"),
        ("Backswing", phase_times["backswing_t"], "--"),
        ("Forward", phase_times["forward_t"], "--"),
        ("Impact", 0.0, "-"),
        ("Follow", phase_times["follow_t"], "--"),
        ("End", phase_times["end_t"], "--"),
    ]

    for label, t, style in lines:
        if t is not None:
            ax.axvline(t, linestyle=style, linewidth=1.5, label=label)


def main():
    if len(sys.argv) < 3:
        print("Brug: python visualize_swing_imu_and_meta_data.py <imu_csv> <meta_csv> [event_id]")
        return

    imu_csv = sys.argv[1]
    meta_csv = sys.argv[2]

    imu_path = Path(imu_csv)
    base_name = imu_path.stem.replace("_imu", "")

    selected_event_id = int(sys.argv[3]) if len(sys.argv) >= 4 else None

    print(f"Indlæser IMU-fil:  {imu_csv}")
    print(f"Indlæser META-fil: {meta_csv}")

    imu_df = pd.read_csv(imu_csv)
    meta_df = pd.read_csv(meta_csv)

    if imu_df.empty or meta_df.empty:
        print("IMU- eller META-filen er tom")
        return

    available_events = sorted(imu_df["event_id"].unique())

    if selected_event_id is None:
        selected_event_id = available_events[0]

    imu_event = imu_df[imu_df["event_id"] == selected_event_id].copy()
    meta_event = meta_df[meta_df["event_id"] == selected_event_id].copy()

    if meta_event.empty:
        print(f"Ingen META fundet for event {selected_event_id}")
        return

    meta_row = meta_event.iloc[0]

    sample_rate_hz = float(meta_row["sample_rate_hz"])
    impact_index_in_event = int(meta_row["impact_index_in_event"])

    print(f"Valgt event_id: {selected_event_id}")
    print(f"Samples i IMU-fil: {len(imu_event)}")
    print(f"Sample rate: {sample_rate_hz} Hz")
    print(f"Impact index i event: {impact_index_in_event}")

    imu_event = remove_duplicate_samples(imu_event)
    imu_event = prepare_data(imu_event, sample_rate_hz, impact_index_in_event)

    phase_times = compute_phase_times(meta_row)

    print("\nDEBUG:")
    print(f"imu t min/max: {imu_event['t'].min():.3f} / {imu_event['t'].max():.3f}")
    print(f"event_start_t: {phase_times['event_start_t']}")
    print(f"event_end_t:   {phase_times['event_end_t']}")

    acc_peak_idx = imu_event["acc_mag_g"].idxmax()
    gyro_peak_idx = imu_event["gyro_mag_dps"].idxmax()

    acc_peak_t = imu_event.loc[acc_peak_idx, "t"]
    gyro_peak_t = imu_event.loc[gyro_peak_idx, "t"]

    forward_t = phase_times["forward_t"]

    plot_df = imu_event.copy()

    if ENABLE_CROP:
        plot_df = plot_df[plot_df["t"] <= CROP_AFTER_IMPACT_SEC].copy()

    if forward_t is not None:
        back_mask_plot = plot_df["t"] < forward_t
        forward_mask_plot = plot_df["t"] >= forward_t
    else:
        back_mask_plot = plot_df["t"] <= 0
        forward_mask_plot = plot_df["t"] > 0

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    axes[0].plot(plot_df["t"][back_mask_plot], plot_df["angle_deg"][back_mask_plot], color="red", linewidth=2, label="Backswing")
    axes[0].plot(plot_df["t"][forward_mask_plot], plot_df["angle_deg"][forward_mask_plot], color="blue", linewidth=2, label="Forward/Follow")
    add_phase_spans(axes[0], phase_times)
    add_phase_lines(axes[0], phase_times)
    axes[0].axvline(acc_peak_t, linestyle="--", linewidth=1.2, label="Acc peak")
    axes[0].axvline(gyro_peak_t, linestyle=":", linewidth=1.2, label="Gyro peak")
    axes[0].set_ylabel("Angle [deg]")
    axes[0].set_title(base_name)
    axes[0].grid(True)
    axes[0].legend(loc="upper right", ncol=2)

    axes[1].plot(plot_df["t"], plot_df["gx_dps"], alpha=0.8, label="gx")
    axes[1].plot(plot_df["t"], plot_df["gy_dps"], alpha=0.8, label="gy")
    axes[1].plot(plot_df["t"], plot_df["gz_dps"], alpha=0.8, label="gz")
    axes[1].plot(plot_df["t"], plot_df["gyro_mag_dps"], linewidth=2, label="|gyro|")
    add_phase_spans(axes[1], phase_times)
    add_phase_lines(axes[1], phase_times)
    axes[1].set_ylabel("Gyro [dps]")
    axes[1].grid(True)
    axes[1].legend(loc="upper right", ncol=2)

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

    x_min = min(float(plot_df["t"].min()), phase_times["event_start_t"] or float(plot_df["t"].min()))
    x_max = max(float(plot_df["t"].max()), phase_times["event_end_t"] or float(plot_df["t"].max()))

    for ax in axes:
        ax.set_xlim(x_min, x_max)

    plt.tight_layout()

    output_name = f"{base_name}.png"
    plt.savefig(output_name, dpi=300, bbox_inches="tight")
    print(f"Plot gemt som: {output_name}")

    plt.show()


if __name__ == "__main__":
    main()