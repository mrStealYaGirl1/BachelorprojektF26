# ble_read_and_save.py

import asyncio
import struct
import csv
from datetime import datetime
from bleak import BleakClient, BleakScanner

DEVICE_NAME = "IMU_GOLF_V3"
CHAR_UUID = "0000fff1-0000-1000-8000-00805f9b34fb"

BLE_PKT_TYPE_META = 1
BLE_PKT_TYPE_IMU = 2

IMU_HEADER_FMT = "<HHH"
IMU_SAMPLE_FMT = "<hhhhhhIH"

IMU_HEADER_SIZE = struct.calcsize(IMU_HEADER_FMT)
IMU_SAMPLE_SIZE = struct.calcsize(IMU_SAMPLE_FMT)

# ESP32/nRF fælles meta-format
META_FMT = "<HHIHHHHHQQQQQQQQ"
META_SIZE = struct.calcsize(META_FMT)  # 82 bytes

EXPECTED_SAMPLES_PER_EVENT = 1000

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

    imu_csv_filename = f"golf_event_{timestamp}_imu.csv"
    meta_csv_filename = f"golf_event_{timestamp}_meta.csv"

    imu_csv_file = open(imu_csv_filename, "w", newline="", encoding="utf-8")
    imu_csv_writer = csv.writer(imu_csv_file)
    imu_csv_writer.writerow([
        "event_id", "seq", "ts_us",
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
        print("RAW:", data.hex(" "))


def decode_meta_packet(data: bytes):
    if len(data) != META_SIZE:
        print(f"Unexpected META packet size: {len(data)} bytes (expected {META_SIZE})")
        print("RAW:", data.hex(" "))
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
    meta_csv_file.flush()

    duration_us = (event_end_us - event_start_us) / 1000.0

    print(
        f"[META] event={event_id} swing_id={swing_id} "
        f"samples={total_samples} fs={sample_rate_hz}Hz "
        f"impact_idx={impact_index_in_event} "
        f"duration={duration_us:.1f} us"
    )


def decode_imu_packet(data: bytes):
    if len(data) < IMU_HEADER_SIZE:
        print(f"Packet too small: {len(data)}")
        return

    event_id, packet_type, sample_count = struct.unpack_from(IMU_HEADER_FMT, data, 0)

    expected_size = IMU_HEADER_SIZE + sample_count * IMU_SAMPLE_SIZE

    if len(data) != expected_size:
        print(f"Size mismatch: got {len(data)}, expected {expected_size}, samples={sample_count}")
        print("RAW:", data.hex(" "))
        return

    offset = IMU_HEADER_SIZE

    for _ in range(sample_count):
        ax, ay, az, gx, gy, gz, ts_us, seq = struct.unpack_from(IMU_SAMPLE_FMT, data, offset)
        offset += IMU_SAMPLE_SIZE
        process_imu_sample(event_id, seq, ts_us, ax, ay, az, gx, gy, gz)


def process_imu_sample(event_id, seq, ts_us, ax, ay, az, gx, gy, gz):
    if event_id not in event_sample_counts:
        event_sample_counts[event_id] = 0
        last_seq_per_event[event_id] = None
        print(f"Started receiving IMU for event {event_id}")

    last_seq = last_seq_per_event[event_id]

    if last_seq is not None and seq != last_seq + 1:
        print(f"GAP in event {event_id}: expected {last_seq + 1}, got {seq}")

    last_seq_per_event[event_id] = seq

    if seq % 100 == 0:
        print(f"[IMU] EV:{event_id} SEQ:{seq} T:{ts_us}")

    imu_csv_writer.writerow([
        event_id, seq, ts_us,
        ax, ay, az,
        gx, gy, gz
    ])

    event_sample_counts[event_id] += 1

    expected_samples = EXPECTED_SAMPLES_PER_EVENT

    if event_id in meta_per_event:
        expected_samples = meta_per_event[event_id]["total_samples"]

    if event_sample_counts[event_id] >= expected_samples and event_id not in finished_events:
        finished_events.add(event_id)
        print(f"Event {event_id} finished. Received {event_sample_counts[event_id]} samples.")
        imu_csv_file.flush()


def notification_handler(sender, data):
    try:
        decode_notification(data)
    except Exception as e:
        print(f"Error in notification handler: {e}")
        print("RAW:", data.hex(" "))


async def main():
    open_csv_files()

    print("Scanning for device...")
    devices = await BleakScanner.discover(timeout=5.0)

    target = None

    for d in devices:
        print("NAME:", d.name, "ADDR:", d.address)

        if d.name and DEVICE_NAME in d.name:
            target = d
            break

    if target is None:
        print("❌ Device not found")
        close_csv_files()
        return

    print("Connecting to:", target.address)

    try:
        async with BleakClient(target.address) as client:
            print("Connected!")

            print("Services:")
            for service in client.services:
                print(service)
                for char in service.characteristics:
                    print("  ", char.uuid)

            await client.start_notify(CHAR_UUID, notification_handler)
            print("Notifications started. Waiting for data...")

            while True:
                await asyncio.sleep(1)

    except KeyboardInterrupt:
        print("Stopping by user...")

    except Exception as e:
        print(f"ERROR: {e}")

    finally:
        close_csv_files()

        if event_sample_counts:
            print("\nSummary:")
            for event_id in sorted(event_sample_counts.keys()):
                count = event_sample_counts[event_id]
                expected = meta_per_event.get(event_id, {}).get(
                    "total_samples",
                    EXPECTED_SAMPLES_PER_EVENT
                )
                status = "OK" if count >= expected else "INCOMPLETE"
                print(f"Event {event_id}: {count}/{expected} samples ({status})")

        if meta_per_event:
            print("\nMETA received for events:")
            for event_id in sorted(meta_per_event.keys()):
                meta = meta_per_event[event_id]
                duration_us = (meta["event_end_us"] - meta["event_start_us"]) / 1000.0

                print(
                    f"Event {event_id}: "
                    f"swing_id={meta['swing_id']}, "
                    f"impact_idx={meta['impact_index_in_event']}, "
                    f"sample_rate={meta['sample_rate_hz']} Hz, "
                    f"duration={duration_us:.1f} us"
                )


if __name__ == "__main__":
    asyncio.run(main())