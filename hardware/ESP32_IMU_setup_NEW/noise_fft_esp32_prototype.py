import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

filename = Path(__file__).parent / "støj_esp3v3_1ms-div.csv"

# find header-linjen
header_line = None
with open(filename, "r", encoding="utf-8", errors="ignore") as f:
    lines = f.readlines()

for i, line in enumerate(lines):
    if line.strip().startswith("Time (s),"):
        header_line = i
        break

if header_line is None:
    raise ValueError("Kunne ikke finde header-linjen 'Time (s),...' i CSV-filen.")

# læs data fra headeren og ned
data = pd.read_csv(filename, skiprows=header_line)

# omdøb kolonner for nemheds skyld
data.columns = ["time", "voltage"]

time = data["time"].to_numpy()
voltage = data["voltage"].to_numpy()

# fjern DC-komponent
signal = voltage - np.mean(voltage)

# samplingfrekvens
dt = time[1] - time[0]
fs = 1 / dt

# FFT
fft_vals = np.fft.rfft(signal)
freqs = np.fft.rfftfreq(len(signal), d=dt)
magnitude = np.abs(fft_vals)
magnitude_db = 20 * np.log10(magnitude + 1e-15)

# find peaks over 1 kHz
min_freq = 1000
valid = freqs > min_freq
freqs_valid = freqs[valid]
mag_valid = magnitude_db[valid]

peak_indices = []
for i in range(1, len(mag_valid) - 1):
    if mag_valid[i] > mag_valid[i - 1] and mag_valid[i] > mag_valid[i + 1]:
        peak_indices.append(i)

peak_indices = sorted(peak_indices, key=lambda i: mag_valid[i], reverse=True)

print(f"Antal samples: {len(signal)}")
print(f"Samplingfrekvens: {fs:.1f} Hz")
print("Top peaks:")

shown = 0
used_freqs = []
for i in peak_indices:
    f = freqs_valid[i]
    a = mag_valid[i]
    if all(abs(f - uf) > 5000 for uf in used_freqs):
        print(f"{shown+1}: {f:.1f} Hz, {a:.2f} dB")
        used_freqs.append(f)
        shown += 1
    if shown >= 10:
        break

# tidsplot
plt.figure(figsize=(10, 4))
plt.plot(time * 1000, signal * 1000)
plt.xlabel("Tid [ms]")
plt.ylabel("Spændingsvariation [mV]")
plt.title("3.3 V støj i tidsdomænet")
plt.grid()
plt.tight_layout()

# FFT-plot
plt.figure(figsize=(10, 4))
plt.plot(freqs / 1000, magnitude_db)
plt.xlabel("Frekvens [kHz]")
plt.ylabel("Amplitude [dB]")
plt.title("FFT af 3.3 V støj")
plt.grid()
plt.tight_layout()

plt.show()