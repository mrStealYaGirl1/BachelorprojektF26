import re
import pandas as pd
import matplotlib.pyplot as plt

LOG_FILE = "impact_log.txt"

print("Starter analyse...")
print("Læser fil:", LOG_FILE)

events = []
current = None

latency_re = re.compile(
    r"IMPACT_LATENCY:.*?"
    r"impact_idx=(\d+).*?"
    r"detect_idx=(\d+).*?"
    r"sample_delay=(\d+).*?"
    r"sample_delay_ms=([0-9.]+).*?"
    r"software_latency_ms=([0-9.]+).*?"
    r"total_latency_ms=([0-9.]+)"
)

notify_re = re.compile(
    r"IMPACT_NOTIFY: impact_to_notify_ms=([0-9.]+)"
)

capture_re = re.compile(
    r"CAPTURE_START: impact_to_capture_start_ms=([0-9.]+)"
)

queue_re = re.compile(
    r"Queueing time:\s+([0-9.]+) sec"
)

with open(LOG_FILE, "r", encoding="utf-8", errors="ignore") as f:
    lines = f.readlines()

print("Antal linjer i fil:", len(lines))

for line in lines:
    line = line.strip()

    m = latency_re.search(line)
    if m:
        if current is not None:
            events.append(current)

        current = {
            "impact_idx": int(m.group(1)),
            "detect_idx": int(m.group(2)),
            "sample_delay": int(m.group(3)),
            "sample_delay_ms": float(m.group(4)),
            "software_latency_ms": float(m.group(5)),
            "total_latency_ms": float(m.group(6)),
        }
        continue

    m = notify_re.search(line)
    if m and current is not None:
        current["impact_to_notify_ms"] = float(m.group(1))
        continue

    m = capture_re.search(line)
    if m and current is not None:
        current["impact_to_capture_start_ms"] = float(m.group(1))
        continue

    m = queue_re.search(line)
    if m and current is not None:
        current["queueing_time_sec"] = float(m.group(1))
        continue

if current is not None:
    events.append(current)

print("Antal fundne impact-events:", len(events))

if len(events) == 0:
    print("\nIngen events fundet.")
    print("Tjek at impact_log.txt faktisk indeholder linjer med IMPACT_LATENCY.")
    print("\nFørste 5 linjer i filen:")
    for line in lines[:5]:
        print(repr(line))
    raise SystemExit

df = pd.DataFrame(events)
df.insert(0, "swing_nr", range(1, len(df) + 1))

print("\n=== Impact timing table ===")
print(df.to_string(index=False))

print("\n=== Statistics ===")
stats_cols = [
    "sample_delay_ms",
    "software_latency_ms",
    "total_latency_ms",
    "impact_to_notify_ms",
    "impact_to_capture_start_ms",
    "queueing_time_sec",
]

stats_cols = [c for c in stats_cols if c in df.columns]

stats_df = df[stats_cols].describe().loc[["count", "mean", "min", "max", "std"]]

print(stats_df)

stats_df.to_csv("impact_timing_statistics.csv")
print("Statistik gemt som: impact_timing_statistics.csv")

df.to_csv("impact_timing_results.csv", index=False)
print("\nCSV gemt som: impact_timing_results.csv")

plt.figure(figsize=(10, 5))

if "impact_to_notify_ms" in df.columns:
    plt.plot(df["swing_nr"], df["impact_to_notify_ms"], marker="o", label="Impact → notify")

if "impact_to_capture_start_ms" in df.columns:
    plt.plot(df["swing_nr"], df["impact_to_capture_start_ms"], marker="o", label="Impact → capture start")

plt.axhline(50, linestyle="--", label="Krav: 50 ms")

plt.xlabel("Slag nr.")
plt.ylabel("Tid [ms]")
plt.title("Impact timing pr. slag")
plt.legend()
plt.grid(True)
plt.tight_layout()

plt.savefig("impact_timing_plot.png", dpi=300)
print("Graf gemt som: impact_timing_plot.png")

plt.show()