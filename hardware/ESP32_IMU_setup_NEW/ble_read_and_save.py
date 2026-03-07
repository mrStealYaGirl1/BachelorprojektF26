import asyncio
import struct
import math
import csv
from bleak import BleakClient, BleakScanner

DEVICE_NAME = "GOLF_IMU"
CHAR_UUID = "99887766-5544-3322-1100-ffeeddccbbaa"

BLE_IMU_SAMPLES_PER_PKT = 5
PKT_HEADER_FMT = "<HH"
SAMPLE_FMT = "<hhhhhhIH"

PKT_HEADER_SIZE = struct.calcsize(PKT_HEADER_FMT)
SAMPLE_SIZE = struct.calcsize(SAMPLE_FMT)
PKT_SIZE = PKT_HEADER_SIZE + BLE_IMU_SAMPLES_PER_PKT * SAMPLE_SIZE

csv_file = None
csv_writer = None


def open_csv_file():
    global csv_file, csv_writer
    csv_file = open("golf_event.csv", "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        "event_id", "seq", "ts_ms",
        "ax", "ay", "az",
        "gx", "gy", "gz"
    ])


def close_csv_file():
    global csv_file
    if csv_file:
        csv_file.close()


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
    global csv_writer

    print(
        f"EV:{event_id:3} SEQ:{seq:4} "
        f"ACC_RAW: ({ax:6},{ay:6},{az:6}) "
        f"GYR_RAW: ({gx:6},{gy:6},{gz:6}) "
        f"| T:{ts_ms}"
    )

    csv_writer.writerow([event_id, seq, ts_ms, ax, ay, az, gx, gy, gz])


def notification_handler(sender, data):
    decode_packet(data)


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

    async with BleakClient(target.address) as client:
        print("Connected!")
        await client.start_notify(CHAR_UUID, notification_handler)

        try:
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            print("Stopping...")
        finally:
            close_csv_file()


if __name__ == "__main__":
    asyncio.run(main())