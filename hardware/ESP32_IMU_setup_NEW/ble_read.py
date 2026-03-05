import asyncio
import struct
import math
from bleak import BleakClient, BleakScanner

DEVICE_NAME = "GOLF_IMU"
CHAR_UUID = "99887766-5544-3322-1100-ffeeddccbbaa"

SAMPLE_SIZE = 20

# swing state
samples = []
max_accel = 0
max_gyro = 0
swing_start_time = None
impact_time = None


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

        process_sample(ax, ay, az, gx, gy, gz, ts_ms)


SWING_START_THRESHOLD = 1200
IMPACT_THRESHOLD = 32000
MIN_SWING_TIME = 120

def process_sample(ax, ay, az, gx, gy, gz, timestamp):

    global max_accel, max_gyro
    global swing_start_time, impact_time

    accel = math.sqrt(ax*ax + ay*ay + az*az)
    gyro  = math.sqrt(gx*gx + gy*gy + gz*gz)

    samples.append((timestamp, accel, gyro))

    if accel > max_accel:
        max_accel = accel

    if gyro > max_gyro:
        max_gyro = gyro

    # detect swing start
    if swing_start_time is None and gyro > SWING_START_THRESHOLD:
        swing_start_time = timestamp

    # detect impact
    if (
        swing_start_time is not None
        and impact_time is None
        and (timestamp - swing_start_time) > MIN_SWING_TIME
        and accel > IMPACT_THRESHOLD
    ):
        impact_time = timestamp
        calculate_metrics()
        
        print(f"ACC:{accel:.0f}  GYRO:{gyro:.0f}")


def calculate_metrics():

    global max_accel, max_gyro
    global swing_start_time, impact_time

    swing_duration = impact_time - swing_start_time

    print("\nSWING DETECTED")
    print(f"Max acceleration: {max_accel/2048:.2f} g")
    print(f"Max gyro speed:   {max_gyro:.2f}")
    print(f"Swing duration:   {swing_duration} ms")

    reset_swing()


def reset_swing():

    global samples
    global max_accel, max_gyro
    global swing_start_time, impact_time

    samples = []
    max_accel = 0
    max_gyro = 0
    swing_start_time = None
    impact_time = None


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