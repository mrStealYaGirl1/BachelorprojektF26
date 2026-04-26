import asyncio
import struct
import csv
import os
from datetime import datetime
from bleak import BleakClient, BleakScanner

DEVICE_NAME = "GOLF_IMU"
CHAR_UUID = "99887766-5544-3322-1100-ffeeddccbbaa"

# =========================================================
# Packet type IDs
# =========================================================
BLE_PKT_TYPE_META = 1
BLE_PKT_TYPE_IMU = 2

# =========================================================
# BLE IMU packet format
# C struct:
# uint16_t event_id;
# uint16_t packet_type;
# uint16_t sample_count;
# ble_imu_sample_t samples[5];
#
# ble_imu_sample_t:
# int16_t ax, ay, az, gx, gy, gz;
# uint32_t ts_ms;
# uint16_t seq;
# =========================================================
BLE_IMU_SAMPLES_PER_PKT = 5

IMU_HEADER_FMT = "<HHH"          # event_id, packet_type, sample_count
IMU_SAMPLE_FMT = "<hhhhhhIH"     # ax ay az gx gy gz ts_ms seq

IMU_HEADER_SIZE = struct.calcsize(IMU_HEADER_FMT)     # 6
IMU_SAMPLE_SIZE = struct.calcsize(IMU_SAMPLE_FMT)     # 18
IMU_PKT_SIZE = IMU_HEADER_SIZE + BLE_IMU_SAMPLES_PER_PKT * IMU_SAMPLE_SIZE  # 96

# =========================================================
# BLE META packet format
# C struct:
# uint16_t event_id;
# uint16_t packet_type;
# uint32_t swing_id;
# uint16_t sample_rate_hz;
# uint16_t total_samples;
# uint16_t pre_samples;
# uint16_t post_samples;
# uint16_t impact_index_in_event;
# uint64_t address_start_us;
# uint64_t backswing_start_us;
# uint64_t forward_start_us;
# uint64_t impact_us;
# uint64_t follow_start_us;
# uint64_t end_us;
# uint64_t event_start_us;
# uint64_t event_end_us;
# =========================================================
META_FMT = "<HHIHHHHHQQQQQQQQ"
META_SIZE = struct.calcsize(META_FMT)   # 82 bytes

# =========================================================
# Expected event size
# =========================================================
EXPECTED_SAMPLES_PER_EVENT = 1000

# =========================================================
# Globals
# =========================================================
imu_csv_file = None
imu_csv_writer = None
imu_csv_filename = None

meta_csv_file = None
meta_csv_writer = None
meta_csv_filename = None

event_sample_counts = {}
finished_events = set()
last_seq_per_event = {}
meta_per_event = {}


def open_csv_files():
    global imu_csv_file, imu_csv_writer, imu_csv_filename
    global meta_csv_file, meta_csv_writer, meta_csv_filename

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    # 👇 mappe hvor filer skal gemmes
    base_dir = os.path.join("golf_events", "med_meta")

    # 👇 opret mappe hvis den ikke findes
    os.makedirs(base_dir, exist_ok=True)

    imu_csv_filename = os.path.join(base_dir, f"golf_event_{timestamp}_imu.csv")
    meta_csv_filename = os.path.join(base_dir, f"golf_event_{timestamp}_meta.csv")

    imu_csv_file = open(imu_csv_filename, "w", newline="", encoding="utf-8")
    imu_csv_writer = csv.writer(imu_csv_file)
    imu_csv_writer.writerow([
        "event_id", "seq", "ts_ms",
        "ax", "ay", "az",
        "gx", "gy", "gz"
    ])

    meta_csv_file = open(meta_csv_filename, "w", newline="", encoding="utf-8")
    meta_csv_writer = csv.writer(meta_csv_file)
    meta_csv_writer.writerow([
        "event_id",
        "swing_id",
        "sample_rate_hz",
        "total_samples",
        "pre_samples",
        "post_samples",
        "impact_index_in_event",
        "address_start_us",
        "backswing_start_us",
        "forward_start_us",
        "impact_us",
        "follow_start_us",
        "end_us",
        "event_start_us",
        "event_end_us"
    ])

    print(f"Saving IMU data to:  {imu_csv_filename}")
    print(f"Saving META data to: {meta_csv_filename}")


def close_csv_files():
    global imu_csv_file, meta_csv_file

    if imu_csv_file:
        imu_csv_file.close()
        print(f"IMU CSV file closed: {imu_csv_filename}")

    if meta_csv_file:
        meta_csv_file.close()
        print(f"META CSV file closed: {meta_csv_filename}")


def decode_notification(data: bytes):
    if len(data) < 4:
        print(f"Packet too short: {len(data)} bytes")
        return

    event_id, packet_type = struct.unpack_from("<HH", data, 0)

    if packet_type == BLE_PKT_TYPE_META:
        decode_meta_packet(data)

    elif packet_type == BLE_PKT_TYPE_IMU:
        decode_imu_packet(data)

    else:
        print(f"Unknown packet_type={packet_type} len={len(data)} event_id={event_id}")


def decode_meta_packet(data: bytes):
    if len(data) != META_SIZE:
        print(f"Unexpected META packet size: {len(data)} (expected {META_SIZE})")
        return

    (
        event_id,
        packet_type,
        swing_id,
        sample_rate_hz,
        total_samples,
        pre_samples,
        post_samples,
        impact_index_in_event,
        address_start_us,
        backswing_start_us,
        forward_start_us,
        impact_us,
        follow_start_us,
        end_us,
        event_start_us,
        event_end_us,
    ) = struct.unpack(META_FMT, data)

    meta = {
        "event_id": event_id,
        "packet_type": packet_type,
        "swing_id": swing_id,
        "sample_rate_hz": sample_rate_hz,
        "total_samples": total_samples,
        "pre_samples": pre_samples,
        "post_samples": post_samples,
        "impact_index_in_event": impact_index_in_event,
        "address_start_us": address_start_us,
        "backswing_start_us": backswing_start_us,
        "forward_start_us": forward_start_us,
        "impact_us": impact_us,
        "follow_start_us": follow_start_us,
        "end_us": end_us,
        "event_start_us": event_start_us,
        "event_end_us": event_end_us,
    }

    meta_per_event[event_id] = meta

    meta_csv_writer.writerow([
        event_id,
        swing_id,
        sample_rate_hz,
        total_samples,
        pre_samples,
        post_samples,
        impact_index_in_event,
        address_start_us,
        backswing_start_us,
        forward_start_us,
        impact_us,
        follow_start_us,
        end_us,
        event_start_us,
        event_end_us,
    ])

    print(
        f"[META] event={event_id} swing_id={swing_id} "
        f"samples={total_samples} fs={sample_rate_hz}Hz "
        f"impact_idx={impact_index_in_event}"
    )

    if backswing_start_us and impact_us:
        print(f"       backswing->impact: {(impact_us - backswing_start_us)/1000:.1f} ms")
    if forward_start_us and impact_us:
        print(f"       forward->impact:   {(impact_us - forward_start_us)/1000:.1f} ms")
    if event_start_us and event_end_us:
        print(f"       event duration:    {(event_end_us - event_start_us)/1_000_000:.3f} s")


def decode_imu_packet(data: bytes):
    if len(data) != IMU_PKT_SIZE:
        print(f"Unexpected IMU packet size: {len(data)} (expected {IMU_PKT_SIZE})")
        return

    event_id, packet_type, sample_count = struct.unpack_from(IMU_HEADER_FMT, data, 0)

    if sample_count > BLE_IMU_SAMPLES_PER_PKT:
        print(f"Invalid IMU sample_count={sample_count} for event {event_id}")
        return

    offset = IMU_HEADER_SIZE

    for _ in range(sample_count):
        ax, ay, az, gx, gy, gz, ts_ms, seq = struct.unpack_from(IMU_SAMPLE_FMT, data, offset)
        offset += IMU_SAMPLE_SIZE

        process_imu_sample(event_id, seq, ts_ms, ax, ay, az, gx, gy, gz)


def process_imu_sample(event_id, seq, ts_ms, ax, ay, az, gx, gy, gz):
    global imu_csv_writer, event_sample_counts, finished_events, last_seq_per_event

    if event_id not in event_sample_counts:
        event_sample_counts[event_id] = 0
        last_seq_per_event[event_id] = None
        print(f"Started receiving IMU for event {event_id}")

    last_seq = last_seq_per_event[event_id]
    if last_seq is not None and seq != last_seq + 1:
        print(f"GAP in event {event_id}: expected {last_seq + 1}, got {seq}")

    last_seq_per_event[event_id] = seq

    if seq % 100 == 0:
        print(f"[IMU] EV:{event_id} SEQ:{seq} T:{ts_ms}")

    imu_csv_writer.writerow([event_id, seq, ts_ms, ax, ay, az, gx, gy, gz])

    event_sample_counts[event_id] += 1

    expected_samples = EXPECTED_SAMPLES_PER_EVENT
    if event_id in meta_per_event:
        expected_samples = meta_per_event[event_id]["total_samples"]

    if event_sample_counts[event_id] >= expected_samples and event_id not in finished_events:
        finished_events.add(event_id)
        print(f"Event {event_id} finished. Received {event_sample_counts[event_id]} samples.")


def notification_handler(sender, data):
    try:
        decode_notification(data)
    except Exception as e:
        print(f"Error in notification handler: {e}")


async def main():
    open_csv_files()

    print("Scanning for device...")
    devices = await BleakScanner.discover()

    target = None
    for d in devices:
        print(d.name, d.address)
        if d.name and DEVICE_NAME in d.name:
            target = d

    if target is None:
        print("Device not found")
        close_csv_files()
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
        close_csv_files()

        if event_sample_counts:
            print("\nSummary:")
            for event_id in sorted(event_sample_counts.keys()):
                count = event_sample_counts[event_id]
                expected = meta_per_event.get(event_id, {}).get("total_samples", EXPECTED_SAMPLES_PER_EVENT)
                status = "OK" if count >= expected else "INCOMPLETE"
                print(f"Event {event_id}: {count}/{expected} samples ({status})")

        if meta_per_event:
            print("\nMETA received for events:")
            for event_id in sorted(meta_per_event.keys()):
                meta = meta_per_event[event_id]
                print(
                    f"Event {event_id}: swing_id={meta['swing_id']}, "
                    f"impact_idx={meta['impact_index_in_event']}, "
                    f"sample_rate={meta['sample_rate_hz']} Hz"
                )


if __name__ == "__main__":
    asyncio.run(main())