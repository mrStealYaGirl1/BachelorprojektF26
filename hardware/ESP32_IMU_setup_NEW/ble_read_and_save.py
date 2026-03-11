import asyncio
import struct
import csv
from datetime import datetime
from bleak import BleakClient, BleakScanner

DEVICE_NAME = "GOLF_IMU"
CHAR_UUID = "99887766-5544-3322-1100-ffeeddccbbaa"

# ===== BLE packet format =====
BLE_PKT_TYPE_META = 1
BLE_PKT_TYPE_IMU = 2

BLE_IMU_SAMPLES_PER_PKT = 5

META_FMT = "<BHHBH"        # pkt_type, event_id, total_samples, putt_start_valid, impact_seq
IMU_HEADER_FMT = "<BHH"    # pkt_type, event_id, sample_count
SAMPLE_FMT = "<hhhhhhIH"   # ax ay az gx gy gz ts_ms seq

META_SIZE = struct.calcsize(META_FMT)
IMU_HEADER_SIZE = struct.calcsize(IMU_HEADER_FMT)
SAMPLE_SIZE = struct.calcsize(SAMPLE_FMT)
IMU_PKT_SIZE = IMU_HEADER_SIZE + BLE_IMU_SAMPLES_PER_PKT * SAMPLE_SIZE

# ===== Expected event size =====
EXPECTED_SAMPLES_PER_EVENT = 1000

# ===== Globals =====
csv_file = None
csv_writer = None
csv_filename = None

event_sample_counts = {}
finished_events = set()
last_seq_per_event = {}
event_meta = {}

def open_csv_file():
    global csv_file, csv_writer, csv_filename

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"golf_event_{timestamp}.csv"

    csv_file = open(csv_filename, "w", newline="", encoding="utf-8")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
    "event_id", "seq", "ts_ms",
    "ax", "ay", "az",
    "gx", "gy", "gz",
    "putt_start_valid", "impact_seq"
])

    print(f"Saving data to: {csv_filename}")


def close_csv_file():
    global csv_file
    if csv_file:
        csv_file.close()
        print(f"CSV file closed: {csv_filename}")



def decode_metadata_packet(data: bytes):
    global event_meta

    if len(data) != META_SIZE:
        print(f"Unexpected metadata packet size: {len(data)} (expected {META_SIZE})")
        return

    pkt_type, event_id, total_samples, putt_start_valid, impact_seq = struct.unpack(META_FMT, data)

    event_meta[event_id] = {
        "total_samples": total_samples,
        "putt_start_valid": putt_start_valid,
        "impact_seq": impact_seq,
    }

    print(
        f"Metadata event {event_id}: "
        f"total_samples={total_samples}, "
        f"putt_start_valid={putt_start_valid}, "
        f"impact_seq={impact_seq}"
    )

def decode_packet(data: bytes):
    if not data:
        return

    pkt_type = data[0]

    if pkt_type == BLE_PKT_TYPE_META:
        decode_metadata_packet(data)
        return

    if pkt_type != BLE_PKT_TYPE_IMU:
        print(f"Unknown packet type: {pkt_type}")
        return

    if len(data) != IMU_PKT_SIZE:
        print(f"Unexpected IMU packet size: {len(data)} (expected {IMU_PKT_SIZE})")
        return

    pkt_type, event_id, sample_count = struct.unpack_from(IMU_HEADER_FMT, data, 0)

    offset = IMU_HEADER_SIZE

    for _ in range(sample_count):
        ax, ay, az, gx, gy, gz, ts_ms, seq = struct.unpack_from(SAMPLE_FMT, data, offset)
        offset += SAMPLE_SIZE

        process_sample(event_id, seq, ts_ms, ax, ay, az, gx, gy, gz)
        

def process_sample(event_id, seq, ts_ms, ax, ay, az, gx, gy, gz):
    global csv_writer, event_sample_counts, finished_events, last_seq_per_event

    if event_id not in event_sample_counts:
        event_sample_counts[event_id] = 0
        last_seq_per_event[event_id] = None
        print(f"Started receiving event {event_id}")

    last_seq = last_seq_per_event[event_id]
    if last_seq is not None and seq != last_seq + 1:
        print(f"GAP in event {event_id}: expected {last_seq + 1}, got {seq}")

    last_seq_per_event[event_id] = seq

    if seq % 100 == 0:
        print(f"EV:{event_id} SEQ:{seq} T:{ts_ms}")

    #csv_writer.writerow([event_id, seq, ts_ms, ax, ay, az, gx, gy, gz])
    meta = event_meta.get(event_id, {})
    putt_start_valid = meta.get("putt_start_valid", "")
    impact_seq = meta.get("impact_seq", "")

    csv_writer.writerow([
        event_id, seq, ts_ms,
        ax, ay, az,
        gx, gy, gz,
        putt_start_valid, impact_seq
    ])
    

    event_sample_counts[event_id] += 1

    if (
        event_sample_counts[event_id] >= EXPECTED_SAMPLES_PER_EVENT
        and event_id not in finished_events
    ):
        finished_events.add(event_id)
        print(f"Event {event_id} finished. Received {event_sample_counts[event_id]} samples.")


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

                meta = event_meta.get(event_id, {})
                expected = meta.get("total_samples", EXPECTED_SAMPLES_PER_EVENT)
                putt_start_valid = meta.get("putt_start_valid", "N/A")
                impact_seq = meta.get("impact_seq", "N/A")

                status = "OK" if count >= expected else "INCOMPLETE"

                print(
                    f"Event {event_id}: {count} samples ({status}), "
                    f"putt_start_valid={putt_start_valid}, impact_seq={impact_seq}"
                )


if __name__ == "__main__":
    asyncio.run(main())