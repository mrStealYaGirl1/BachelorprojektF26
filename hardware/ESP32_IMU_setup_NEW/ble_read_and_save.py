import asyncio
import struct
import csv
from datetime import datetime
from bleak import BleakClient, BleakScanner

DEVICE_NAME = "GOLF_IMU"
CHAR_UUID = "99887766-5544-3322-1100-ffeeddccbbaa"

# ===== BLE packet format =====
BLE_IMU_SAMPLES_PER_PKT = 5
PKT_HEADER_FMT = "<HH"     # event_id, sample_count
SAMPLE_FMT = "<hhhhhhIH"   # ax ay az gx gy gz ts_ms seq

PKT_HEADER_SIZE = struct.calcsize(PKT_HEADER_FMT)
SAMPLE_SIZE = struct.calcsize(SAMPLE_FMT)
PKT_SIZE = PKT_HEADER_SIZE + BLE_IMU_SAMPLES_PER_PKT * SAMPLE_SIZE

# ===== Expected event size =====
EXPECTED_SAMPLES_PER_EVENT = 1000

# ===== Globals =====
csv_file = None
csv_writer = None
csv_filename = None

event_sample_counts = {}
finished_events = set()


def open_csv_file():
    global csv_file, csv_writer, csv_filename

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"golf_event_{timestamp}.csv"

    csv_file = open(csv_filename, "w", newline="", encoding="utf-8")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        "event_id", "seq", "ts_ms",
        "ax", "ay", "az",
        "gx", "gy", "gz"
    ])

    print(f"Saving data to: {csv_filename}")


def close_csv_file():
    global csv_file
    if csv_file:
        csv_file.close()
        print(f"CSV file closed: {csv_filename}")


def decode_packet(data: bytes):
    if len(data) != PKT_SIZE:
        print(f"Unexpected packet size: {len(data)} (expected {PKT_SIZE})")
        return

    event_id, sample_count = struct.unpack_from(PKT_HEADER_FMT, data, 0)

    offset = PKT_HEADER_SIZE

    for _ in range(sample_count):
        ax, ay, az, gx, gy, gz, ts_ms, seq = struct.unpack_from(SAMPLE_FMT, data, offset)
        offset += SAMPLE_SIZE

        process_sample(event_id, seq, ts_ms, ax, ay, az, gx, gy, gz)


def process_sample(event_id, seq, ts_ms, ax, ay, az, gx, gy, gz):
    global csv_writer, event_sample_counts, finished_events

    # status-print hver 100. sample
    if seq % 100 == 0:
        print(f"EV:{event_id} SEQ:{seq} T:{ts_ms}")

    # gem sample i CSV
    csv_writer.writerow([event_id, seq, ts_ms, ax, ay, az, gx, gy, gz])

    # opret event hvis det ikke findes endnu
    if event_id not in event_sample_counts:
        event_sample_counts[event_id] = 0
        print(f"Started receiving event {event_id}")

    # tæl samples
    event_sample_counts[event_id] += 1

    # skriv når event er færdigt
    if (
        event_sample_counts[event_id] >= EXPECTED_SAMPLES_PER_EVENT
        and event_id not in finished_events
    ):
        finished_events.add(event_id)
        print(
            f"Event {event_id} finished. "
            f"Received {event_sample_counts[event_id]} samples."
        )


def notification_handler(sender, data):
    try:
        decode_packet(data)
    except Exception as e:
        print(f"Error in notification handler: {e}")


async def main():
    open_csv_file()

    print("Scanning for device...")
    devices = await BleakScanner.discover()

    target = None
    for d in devices:
        print(d.name, d.address)
        if d.name and DEVICE_NAME in d.name:
            target = d

    if target is None:
        print("Device not found")
        close_csv_file()
        return

    print("Found device:", target.address)

    try:
        async with BleakClient(target.address) as client:
            print("Connected!")
            await client.start_notify(CHAR_UUID, notification_handler)
            print("Notifications started. Waiting for data...")

            while True:
                await asyncio.sleep(1)

    except KeyboardInterrupt:
        print("Stopping by user...")

    finally:
        close_csv_file()

        if event_sample_counts:
            print("\nSummary:")
            for event_id in sorted(event_sample_counts.keys()):
                count = event_sample_counts[event_id]
                status = "OK" if count >= EXPECTED_SAMPLES_PER_EVENT else "INCOMPLETE"
                print(f"Event {event_id}: {count} samples ({status})")


if __name__ == "__main__":
    asyncio.run(main())