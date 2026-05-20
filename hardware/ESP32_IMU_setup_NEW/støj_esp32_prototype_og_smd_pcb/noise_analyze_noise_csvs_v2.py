from __future__ import annotations

from pathlib import Path
import re
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# ============================================================
# INDSTILLINGER
# ============================================================

ANALYZE_ALL_CSV_FILES = True

SELECTED_FILES = {
    # Iteration A
    "Iteration_A_ESP32_idle": "støj_esp3v3_1ms-div.csv",
    "Iteration_A_ESP32_BLE_send": "støj_esp32_blesend-1ms-div6.csv",
    "Iteration_A_ESP32_BLE_supplerende": "støj_esp32_ble_1ms-div10.csv",

    # Iteration A - ekstern DC-DC converter
    "Iteration_A_DC_DC_output": "støj_buck_1ms-div.csv",
    "Iteration_A_DC_DC_zoom": "støj_buck_400us-div2.csv",
    "Iteration_A_DC_DC_worst_case": "støj_buckboost_1ms-div.csv",

    # Iteration C
    "Iteration_C_3V3_idle": "smd_pcb_3v3_scope_16.csv",
    "Iteration_C_3V3_BLE_send": "smd_pcb_ble_send_scope_11.csv",
}


# ============================================================
# CSV-LÆSNING
# ============================================================

def load_scope_csv(filepath: Path) -> tuple[np.ndarray, np.ndarray]:
    """
    Læser CSV-filer fra Analog Discovery / scope export.

    Understøtter bl.a.:
    - Time (s),Channel...
    - second,Volt
    """

    header_line = None

    with filepath.open("r", encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()

    for i, line in enumerate(lines):
        stripped = line.strip()

        if stripped.startswith("Time (s),"):
            header_line = i
            break

        if stripped.startswith("second,Volt"):
            header_line = i
            break

    if header_line is None:
        raise ValueError(f"Kunne ikke finde data-header i {filepath.name}")

    data = pd.read_csv(filepath, skiprows=header_line)

    if data.shape[1] < 2:
        raise ValueError(f"For få kolonner i {filepath.name}")

    data = data.iloc[:, :2].copy()
    data.columns = ["time", "voltage"]

    time = data["time"].to_numpy(dtype=float)
    voltage = data["voltage"].to_numpy(dtype=float)

    if len(time) < 4:
        raise ValueError(f"For få samples i {filepath.name}")

    return time, voltage


# ============================================================
# SIGNALANALYSE
# ============================================================

def compute_fft(
    time: np.ndarray,
    voltage: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:

    signal = voltage - np.mean(voltage)

    dt = np.mean(np.diff(time))
    fs = 1.0 / dt

    fft_vals = np.fft.rfft(signal)
    freqs = np.fft.rfftfreq(len(signal), d=dt)

    magnitude = np.abs(fft_vals)
    magnitude_db = 20 * np.log10(magnitude + 1e-15)

    return signal, freqs, magnitude_db, fs


def find_top_peaks(
    freqs: np.ndarray,
    magnitude_db: np.ndarray,
    min_freq_hz: float = 1_000,
    min_spacing_hz: float = 5_000,
    max_peaks: int = 10,
) -> list[tuple[float, float]]:

    valid = freqs > min_freq_hz

    freqs_valid = freqs[valid]
    mag_valid = magnitude_db[valid]

    peak_indices = []

    for i in range(1, len(mag_valid) - 1):
        if mag_valid[i] > mag_valid[i - 1] and mag_valid[i] > mag_valid[i + 1]:
            peak_indices.append(i)

    peak_indices = sorted(
        peak_indices,
        key=lambda i: mag_valid[i],
        reverse=True
    )

    selected = []
    used_freqs = []

    for i in peak_indices:
        f = float(freqs_valid[i])
        a = float(mag_valid[i])

        if all(abs(f - uf) > min_spacing_hz for uf in used_freqs):
            selected.append((f, a))
            used_freqs.append(f)

        if len(selected) >= max_peaks:
            break

    return selected


def infer_dominant_noise_peak(
    peaks: list[tuple[float, float]]
) -> tuple[float | None, float | None]:
    """
    Finder en dominerende støjfrekvens.

    Vigtigt:
    Dette kaldes ikke "switching-frekvens", fordi iteration C bruger LDO,
    og derfor ikke forventes at have en fast switch-mode frekvens.
    """

    candidates = [
        (f, a)
        for f, a in peaks
        if 20_000 <= f <= 1_000_000
    ]

    if not candidates:
        return None, None

    return max(candidates, key=lambda x: x[1])


# ============================================================
# PLOTS
# ============================================================

def sanitize_filename(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9_\-\.]+", "_", name)


def save_plots(
    label: str,
    filepath: Path,
    outdir: Path,
    time: np.ndarray,
    signal: np.ndarray,
    freqs: np.ndarray,
    magnitude_db: np.ndarray,
    top_peaks: list[tuple[float, float]],
) -> tuple[Path, Path]:

    stem = sanitize_filename(label)

    time_plot = outdir / f"{stem}_time.png"
    fft_plot = outdir / f"{stem}_fft.png"

    # Tidsdomæne
    plt.figure(figsize=(10, 4))
    plt.plot(time * 1000, signal * 1000, linewidth=1)
    plt.xlabel("Tid [ms]")
    plt.ylabel("Spændingsvariation [mV]")
    plt.title(f"Tidsdomæne: {filepath.name}")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(time_plot, dpi=200)
    plt.close()

    # FFT
    plt.figure(figsize=(10, 4))
    plt.plot(freqs / 1000, magnitude_db, linewidth=1)
    plt.xlabel("Frekvens [kHz]")
    plt.ylabel("Amplitude [dB]")
    plt.title(f"FFT: {filepath.name}")
    plt.grid(True)

    for f, a in top_peaks[:5]:
        plt.plot(f / 1000, a, "o")
        plt.text(
            f / 1000,
            a + 2,
            f"{f / 1000:.1f} kHz",
            fontsize=8
        )

    plt.tight_layout()
    plt.savefig(fft_plot, dpi=200)
    plt.close()

    return time_plot, fft_plot


# ============================================================
# ANALYSE AF ÉN FIL
# ============================================================

def analyze_file(
    label: str,
    filepath: Path,
    outdir: Path
) -> dict:

    time, voltage = load_scope_csv(filepath)

    signal, freqs, magnitude_db, fs = compute_fft(time, voltage)

    peaks = find_top_peaks(freqs, magnitude_db)
    dominant_f, dominant_a = infer_dominant_noise_peak(peaks)

    time_plot, fft_plot = save_plots(
        label,
        filepath,
        outdir,
        time,
        signal,
        freqs,
        magnitude_db,
        peaks
    )

    result = {
        "label": label,
        "file": filepath.name,
        "samples": len(time),
        "sampling_frequency_hz": fs,
        "duration_ms": float((time[-1] - time[0]) * 1000),
        "mean_voltage_v": float(np.mean(voltage)),
        "std_noise_mv": float(np.std(signal) * 1000),
        "peak_to_peak_mv": float((np.max(signal) - np.min(signal)) * 1000),
        "min_noise_mv": float(np.min(signal) * 1000),
        "max_noise_mv": float(np.max(signal) * 1000),
        "dominant_noise_peak_hz": dominant_f,
        "dominant_noise_peak_khz": None if dominant_f is None else dominant_f / 1000,
        "dominant_noise_peak_db": dominant_a,
        "top_peak_1_khz": None if len(peaks) < 1 else peaks[0][0] / 1000,
        "top_peak_1_db": None if len(peaks) < 1 else peaks[0][1],
        "top_peak_2_khz": None if len(peaks) < 2 else peaks[1][0] / 1000,
        "top_peak_2_db": None if len(peaks) < 2 else peaks[1][1],
        "top_peak_3_khz": None if len(peaks) < 3 else peaks[2][0] / 1000,
        "top_peak_3_db": None if len(peaks) < 3 else peaks[2][1],
        "time_plot": time_plot.name,
        "fft_plot": fft_plot.name,
    }

    print("\n================================================")
    print(label)
    print("================================================")
    print(f"Fil: {filepath.name}")
    print(f"Samples: {len(time)}")
    print(f"Samplingfrekvens: {fs:.1f} Hz")
    print(f"Varighed: {(time[-1] - time[0]) * 1000:.3f} ms")
    print(f"Gennemsnitsspænding: {np.mean(voltage):.6f} V")
    print(f"Std. støj: {np.std(signal) * 1000:.3f} mV")
    print(f"Peak-to-peak: {(np.max(signal) - np.min(signal)) * 1000:.3f} mV")

    if peaks:
        print("\nTop FFT-peaks:")
        for i, (f, a) in enumerate(peaks[:5], start=1):
            print(f"{i}: {f / 1000:.1f} kHz, {a:.2f} dB")

    if dominant_f is not None:
        print(f"\nDominerende støjpeak: {dominant_f / 1000:.1f} kHz")

    return result


# ============================================================
# MAIN
# ============================================================

def main() -> None:

    base_dir = Path(__file__).parent
    measurement_dir = base_dir / "støjmålinger"
    outdir = base_dir / "fft_plots"

    outdir.mkdir(exist_ok=True)

    results = []
    failed = []

    if ANALYZE_ALL_CSV_FILES:
        csv_files = sorted(measurement_dir.glob("*.csv"))
        files_to_analyze = {
            filepath.stem: filepath.name
            for filepath in csv_files
        }
    else:
        files_to_analyze = SELECTED_FILES

    print(f"Analyserer {len(files_to_analyze)} målinger...")

    for label, filename in files_to_analyze.items():

        filepath = measurement_dir / filename

        try:
            results.append(
                analyze_file(
                    label,
                    filepath,
                    outdir
                )
            )

        except Exception as e:
            failed.append((filename, str(e)))
            print(f"\nFEJL i {filename}: {e}")

    if results:
        results_df = pd.DataFrame(results)

        summary_path = outdir / "noise_analysis_summary.csv"

        results_df.to_csv(summary_path, index=False)

        print(f"\nGemte oversigt i:\n{summary_path}")

    if failed:
        failed_path = outdir / "noise_analysis_errors.txt"

        with failed_path.open("w", encoding="utf-8") as f:
            for name, err in failed:
                f.write(f"{name}: {err}\n")

        print(f"\nGemte fejlrapport i:\n{failed_path}")

    print(f"\nAlle plots gemt i:\n{outdir}")


if __name__ == "__main__":
    main()