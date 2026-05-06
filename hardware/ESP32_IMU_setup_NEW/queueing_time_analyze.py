import re
import pandas as pd
import matplotlib.pyplot as plt

LOG_FILE = "queueing_time.txt"

print("Starter analyse...")
print("Læser fil:", LOG_FILE)

events = []
current = None

patterns = {
    "impact_notify": re.compile(r"IMPACT_NOTIFY: impact_to_notify_ms=([0-9.]+)"),
    "capture_start": re.compile(r"CAPTURE_START: impact_to_capture_start_ms=([0-9.]+)"),
    "enqueue": re.compile(r"QUEUE_FLOW: enqueue_duration_ms=([0-9.]+)"),
    "queueing": re.compile(r"Queueing time:\s+([0-9.]+) sec"),
    "packets": re.compile(r"Packets queued:\s+(\d+)"),
    "drops": re.compile(r"Queue drops:\s+(\d+)"),
    "rate": re.compile(r"Queue rate:\s+([0-9.]+) packets/sec"),
    "throughput": re.compile(r"Estimated queue throughput:\s+([0-9.]+) kB/s"),
    "total_data": re.compile(r"Total queued data:\s+([0-9.]+) kB"),
}

with open(LOG_FILE, "r", encoding="utf-8", errors="ignore") as f:
    lines = f.readlines()

for line in lines:
    line = line.strip()

    if m := patterns["impact_notify"].search(line):
        if current is not None:
            events.append(current)
        current = {
            "impact_to_notify_ms": float(m.group(1))
        }

    elif m := patterns["capture_start"].search(line):
        if current is not None:
            current["impact_to_capture_start_ms"] = float(m.group(1))

    elif m := patterns["enqueue"].search(line):
        if current is not None:
            current["enqueue_duration_ms"] = float(m.group(1))

    elif m := patterns["queueing"].search(line):
        if current is not None:
            current["queueing_time_sec"] = float(m.group(1))
            current["queueing_time_ms"] = float(m.group(1)) * 1000.0

    elif m := patterns["packets"].search(line):
        if current is not None:
            current["packets_queued"] = int(m.group(1))

    elif m := patterns["drops"].search(line):
        if current is not None:
            current["queue_drops"] = int(m.group(1))

    elif m := patterns["rate"].search(line):
        if current is not None:
            current["queue_rate_pkt_sec"] = float(m.group(1))

    elif m := patterns["throughput"].search(line):
        if current is not None:
            current["throughput_kB_sec"] = float(m.group(1))

    elif m := patterns["total_data"].search(line):
        if current is not None:
            current["total_data_kB"] = float(m.group(1))

if current is not None:
    events.append(current)

if not events:
    print("Ingen events fundet. Tjek at filen indeholder IMPACT_NOTIFY og Queueing time.")
    raise SystemExit

df = pd.DataFrame(events)
df.insert(0, "swing_nr", range(1, len(df) + 1))

# Beregn BLE-send/tømningstid
if "queueing_time_ms" in df.columns and "enqueue_duration_ms" in df.columns:
    df["ble_send_after_enqueue_ms"] = df["queueing_time_ms"] - df["enqueue_duration_ms"]

print("\n=== Tabel ===")
print(df.to_string(index=False))

stats_cols = [
    "impact_to_notify_ms",
    "impact_to_capture_start_ms",
    "enqueue_duration_ms",
    "queueing_time_ms",
    "ble_send_after_enqueue_ms",
    "packets_queued",
    "queue_drops",
    "queue_rate_pkt_sec",
    "throughput_kB_sec",
]

stats_cols = [c for c in stats_cols if c in df.columns]

stats = df[stats_cols].describe().loc[["count", "mean", "min", "max", "std"]]

print("\n=== Statistik ===")
print(stats)

# Variation i procent: (max-min)/mean*100
variation = {}
for col in stats_cols:
    mean = df[col].mean()
    if mean != 0:
        variation[col] = ((df[col].max() - df[col].min()) / mean) * 100

variation_df = pd.DataFrame.from_dict(
    variation,
    orient="index",
    columns=["variation_percent"]
)

print("\n=== Variation ===")
print(variation_df)

# Gem resultater
df.to_csv("queueing_time_results.csv", index=False)
stats.to_csv("queueing_time_statistics.csv")
variation_df.to_csv("queueing_time_variation.csv")

print("\nGemt:")
print("queueing_time_results.csv")
print("queueing_time_statistics.csv")
print("queueing_time_variation.csv")

# Graf 1: Enqueue og total queueing time
plt.figure(figsize=(10, 5))

if "enqueue_duration_ms" in df.columns:
    plt.plot(df["swing_nr"], df["enqueue_duration_ms"], marker="o", label="Enqueue duration")

if "queueing_time_ms" in df.columns:
    plt.plot(df["swing_nr"], df["queueing_time_ms"], marker="o", label="Total queueing/BLE time")

plt.xlabel("Slag nr.")
plt.ylabel("Tid [ms]")
plt.title("Behandlingstid pr. slag")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("queueing_time_plot.png", dpi=300)
plt.show()

# Graf 2: Impact til capture start
plt.figure(figsize=(10, 5))

if "impact_to_capture_start_ms" in df.columns:
    plt.plot(df["swing_nr"], df["impact_to_capture_start_ms"], marker="o", label="Impact → capture start")

if "impact_to_notify_ms" in df.columns:
    plt.plot(df["swing_nr"], df["impact_to_notify_ms"], marker="o", label="Impact → notify")

plt.axhline(50, linestyle="--", label="Krav: 50 ms")
plt.xlabel("Slag nr.")
plt.ylabel("Tid [ms]")
plt.title("Impact-responstid pr. slag")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("impact_response_plot.png", dpi=300)
plt.show()

print("\nGrafer gemt:")
print("queueing_time_plot.png")
print("impact_response_plot.png")