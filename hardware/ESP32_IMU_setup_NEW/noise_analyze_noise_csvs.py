from __future__ import annotations

from pathlib import Path
import re
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def load_waveforms_csv(filepath: Path) -> tuple[np.ndarray, np.ndarray]:
    """Læs en Digilent WaveForms oscilloskop-CSV."""
    header_line = None

    with filepath.open("r", encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()

    for i, line in enumerate(lines):
        if line.strip().startswith("Time (s),"):
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


def compute_fft(time: np.ndarray, voltage: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """Beregn FFT og returnér signal uden DC, frekvenser, magnitude i dB og fs."""
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
    """Find simple lokale peaks i FFT'en."""
    valid = freqs > min_freq_hz
    freqs_valid = freqs[valid]
    mag_valid = magnitude_db[valid]

    local_peak_indices: list[int] = []
    for i in range(1, len(mag_valid) - 1):
        if mag_valid[i] > mag_valid[i - 1] and mag_valid[i] > mag_valid[i + 1]:
            local_peak_indices.append(i)

    local_peak_indices.sort(key=lambda i: mag_valid[i], reverse=True)

    selected: list[tuple[float, float]] = []
    used_freqs: list[float] = []

    for idx in local_peak_indices:
        f = float(freqs_valid[idx])
        a = float(mag_valid[idx])

        if all(abs(f - uf) > min_spacing_hz for uf in used_freqs):
            selected.append((f, a))
            used_freqs.append(f)

        if len(selected) >= max_peaks:
            break

    return selected


def infer_switching_peak(peaks: list[tuple[float, float]]) -> tuple[float | None, float | None]:
    """
    Vælg et sandsynligt switching-peak.
    Enkel heuristik: foretræk peaks mellem 20 kHz og 500 kHz.
    """
    candidates = [(f, a) for f, a in peaks if 20_000 <= f <= 500_000]
    if not candidates:
        return None, None

    # højeste amplitude blandt kandidater
    best = max(candidates, key=lambda x: x[1])
    return best


def sanitize_filename(name: str) -> str:
    """Gør filnavn sikkert til outputfiler."""
    return re.sub(r"[^A-Za-z0-9_\-\.]+", "_", name)


def save_plots(
    filepath: Path,
    outdir: Path,
    time: np.ndarray,
    signal: np.ndarray,
    freqs: np.ndarray,
    magnitude_db: np.ndarray,
    top_peaks: list[tuple[float, float]],
) -> tuple[Path, Path]:
    """Gem tidsplot og FFT-plot."""
    stem = sanitize_filename(filepath.stem)

    time_plot = outdir / f"{stem}_time.png"
    fft_plot = outdir / f"{stem}_fft.png"

    # Tidsplot
    plt.figure(figsize=(10, 4))
    plt.plot(time * 1000, signal * 1000, linewidth=1)
    plt.xlabel("Tid [ms]")
    plt.ylabel("Spændingsvariation [mV]")
    plt.title(f"Tidsdomæne: {filepath.name}")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(time_plot, dpi=200)
    plt.close()

    # FFT-plot
    plt.figure(figsize=(10, 4))
    plt.plot(freqs / 1000, magnitude_db, linewidth=1)
    plt.xlabel("Frekvens [kHz]")
    plt.ylabel("Amplitude [dB]")
    plt.title(f"FFT: {filepath.name}")
    plt.grid(True)

    # marker op til 5 peaks
    for f, a in top_peaks[:5]:
        plt.plot(f / 1000, a, "o")
        plt.text(f / 1000, a + 2, f"{f/1000:.1f} kHz", fontsize=8)

    plt.tight_layout()
    plt.savefig(fft_plot, dpi=200)
    plt.close()

    return time_plot, fft_plot


def analyze_file(filepath: Path, outdir: Path) -> dict:
    """Analyser én fil og returnér resultater."""
    time, voltage = load_waveforms_csv(filepath)
    signal, freqs, magnitude_db, fs = compute_fft(time, voltage)
    peaks = find_top_peaks(freqs, magnitude_db)
    switch_f, switch_a = infer_switching_peak(peaks)
    time_plot, fft_plot = save_plots(filepath, outdir, time, signal, freqs, magnitude_db, peaks)

    result = {
        "file": filepath.name,
        "samples": len(time),
        "fs_hz": fs,
        "duration_s": float(time[-1] - time[0]),
        "mean_voltage_v": float(np.mean(voltage)),
        "std_voltage_v": float(np.std(signal)),
        "switching_peak_hz": switch_f,
        "switching_peak_db": switch_a,
        "top_peak_1_hz": peaks[0][0] if len(peaks) > 0 else None,
        "top_peak_1_db": peaks[0][1] if len(peaks) > 0 else None,
        "top_peak_2_hz": peaks[1][0] if len(peaks) > 1 else None,
        "top_peak_2_db": peaks[1][1] if len(peaks) > 1 else None,
        "top_peak_3_hz": peaks[2][0] if len(peaks) > 2 else None,
        "top_peak_3_db": peaks[2][1] if len(peaks) > 2 else None,
        "time_plot": str(time_plot.name),
        "fft_plot": str(fft_plot.name),
    }

    print(f"\nFil: {filepath.name}")
    print(f"  Samples: {len(time)}")
    print(f"  Samplingfrekvens: {fs:.1f} Hz")
    print(f"  Varighed: {time[-1] - time[0]:.6f} s")
    print(f"  Std. af signal (uden DC): {np.std(signal)*1000:.3f} mV")

    if peaks:
        print("  Top peaks:")
        for i, (f, a) in enumerate(peaks[:5], start=1):
            print(f"    {i}: {f:.1f} Hz, {a:.2f} dB")
    else:
        print("  Ingen peaks fundet.")

    if switch_f is not None:
        print(f"  Estimeret switching-peak: {switch_f:.1f} Hz ({switch_a:.2f} dB)")
    else:
        print("  Ingen sandsynlig switching-frekvens fundet.")

    return result


def main() -> None:
    base_dir = Path(__file__).parent
    outdir = base_dir / "fft_plots"
    outdir.mkdir(exist_ok=True)

    # Finder alle relevante støj-filer
    csv_files = sorted(base_dir.glob("støj_*.csv"))

    if not csv_files:
        print("Ingen filer fundet, der matcher 'støj_*.csv' i denne mappe.")
        return

    print(f"Fandt {len(csv_files)} CSV-filer.")

    results: list[dict] = []
    failed: list[tuple[str, str]] = []

    for filepath in csv_files:
        try:
            results.append(analyze_file(filepath, outdir))
        except Exception as e:
            failed.append((filepath.name, str(e)))
            print(f"\nFEJL i {filepath.name}: {e}")

    if results:
        results_df = pd.DataFrame(results)
        summary_path = outdir / "noise_analysis_summary.csv"
        results_df.to_csv(summary_path, index=False)
        print(f"\nGemte samlet oversigt i: {summary_path}")

    if failed:
        failed_path = outdir / "noise_analysis_errors.txt"
        with failed_path.open("w", encoding="utf-8") as f:
            for name, err in failed:
                f.write(f"{name}: {err}\n")
        print(f"Gemte fejlrapport i: {failed_path}")

    print(f"\nPlots er gemt i: {outdir}")


if __name__ == "__main__":
    main()