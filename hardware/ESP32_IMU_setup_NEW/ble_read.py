import asyncio
import struct
from bleak import BleakClient, BleakScanner

DEVICE_NAME = "GOLF_IMU"

CHAR_UUID = "99887766-5544-3322-1100-ffeeddccbbaa"

SAMPLE_SIZE = 20


def decode_packet(data):

    count = len(data) // SAMPLE_SIZE

    for i in range(count):

        start = i * SAMPLE_SIZE
        end = start + SAMPLE_SIZE

        sample = data[start:end]

        ax, ay, az, gx, gy, gz, ts_ms, seq, event_id = struct.unpack("<hhhhhhIHH", sample)

        print(
            f"SEQ:{seq:4} "
            f"AX:{ax:6} AY:{ay:6} AZ:{az:6} | "
            f"GX:{gx:6} GY:{gy:6} GZ:{gz:6} | "
            f"T:{ts_ms} "
            f"EV:{event_id}"
        )


def notification_handler(sender, data):

    print("Packet length:", len(data))

    decode_packet(data)


async def main():

    print("Scanning for device...")

    devices = await BleakScanner.discover()

    for d in devices:
        print(d.name, d.address)

    target = None

    for d in devices:
        if d.name and DEVICE_NAME in d.name:
            target = d
            break

    if target is None:
        print("Device not found")
        return

    print("Found device:", target.address)

    async with BleakClient(target.address) as client:

        print("Connected!")

        await client.start_notify(CHAR_UUID, notification_handler)

        while True:
            await asyncio.sleep(1)


asyncio.run(main())