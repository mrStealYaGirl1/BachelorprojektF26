# import asyncio
# import struct
# import math
# from bleak import BleakClient, BleakScanner

# DEVICE_NAME = "GOLF_IMU"
# CHAR_UUID = "99887766-5544-3322-1100-ffeeddccbbaa"

# SAMPLE_SIZE = 20

# # swing state
# samples = []
# max_accel = 0
# max_gyro = 0
# swing_start_time = None
# impact_time = None


# def decode_packet(data):

#     count = len(data) // SAMPLE_SIZE

#     for i in range(count):

#         start = i * SAMPLE_SIZE
#         end = start + SAMPLE_SIZE

#         sample = data[start:end]

#         ax, ay, az, gx, gy, gz, ts_ms, seq, event_id = struct.unpack("<hhhhhhIHH", sample)

#         print(
#             f"SEQ:{seq:4} "
#             f"AX:{ax:6} AY:{ay:6} AZ:{az:6} | "
#             f"GX:{gx:6} GY:{gy:6} GZ:{gz:6} | "
#             f"T:{ts_ms} "
#             f"EV:{event_id}"
#         )

#         process_sample(ax, ay, az, gx, gy, gz, ts_ms)


# SWING_START_THRESHOLD = 1200
# IMPACT_THRESHOLD = 32000
# MIN_SWING_TIME = 120

# def process_sample(ax, ay, az, gx, gy, gz, timestamp):

#     global max_accel, max_gyro
#     global swing_start_time, impact_time

#     accel = math.sqrt(ax*ax + ay*ay + az*az)
#     gyro  = math.sqrt(gx*gx + gy*gy + gz*gz)

#     samples.append((timestamp, accel, gyro))

#     if accel > max_accel:
#         max_accel = accel

#     if gyro > max_gyro:
#         max_gyro = gyro

#     # detect swing start
#     if swing_start_time is None and gyro > SWING_START_THRESHOLD:
#         swing_start_time = timestamp

#     # detect impact
#     if (
#         swing_start_time is not None
#         and impact_time is None
#         and (timestamp - swing_start_time) > MIN_SWING_TIME
#         and accel > IMPACT_THRESHOLD
#     ):
#         impact_time = timestamp
#         calculate_metrics()
        
#         print(f"ACC:{accel:.0f}  GYRO:{gyro:.0f}")


# def calculate_metrics():

#     global max_accel, max_gyro
#     global swing_start_time, impact_time

#     swing_duration = impact_time - swing_start_time

#     print("\nSWING DETECTED")
#     print(f"Max acceleration: {max_accel/2048:.2f} g")
#     print(f"Max gyro speed:   {max_gyro:.2f}")
#     print(f"Swing duration:   {swing_duration} ms")

#     reset_swing()


# def reset_swing():

#     global samples
#     global max_accel, max_gyro
#     global swing_start_time, impact_time

#     samples = []
#     max_accel = 0
#     max_gyro = 0
#     swing_start_time = None
#     impact_time = None


# def notification_handler(sender, data):

#     print("Packet length:", len(data))

#     decode_packet(data)


# async def main():

#     print("Scanning for device...")

#     devices = await BleakScanner.discover()

#     for d in devices:
#         print(d.name, d.address)

#     target = None

#     for d in devices:
#         if d.name and DEVICE_NAME in d.name:
#             target = d
#             break

#     if target is None:
#         print("Device not found")
#         return

#     print("Found device:", target.address)

#     async with BleakClient(target.address) as client:

#         print("Connected!")

#         await client.start_notify(CHAR_UUID, notification_handler)

#         while True:
#             await asyncio.sleep(1)


# asyncio.run(main())
import asyncio
import struct
import math
from bleak import BleakClient, BleakScanner

DEVICE_NAME = "GOLF_IMU"
CHAR_UUID = "99887766-5544-3322-1100-ffeeddccbbaa"

# ESP32-side
BLE_IMU_SAMPLES_PER_PKT = 5

# packet:
# uint16 event_id
# uint16 sample_count
# samples[5]
#
# sample:
# h h h h h h I H
# = 18 bytes

PKT_HEADER_FMT = "<HH"
SAMPLE_FMT = "<hhhhhhIH"

PKT_HEADER_SIZE = struct.calcsize(PKT_HEADER_FMT)   # 4
SAMPLE_SIZE = struct.calcsize(SAMPLE_FMT)           # 18
PKT_SIZE = PKT_HEADER_SIZE + BLE_IMU_SAMPLES_PER_PKT * SAMPLE_SIZE

ACC_LSB_PER_G = 16384.0
GYRO_DPS_PER_LSB = 2000.0 / 32768.0

current_event_id = None
current_event_samples = []

max_accel_g = 0.0
max_gyro_dps = 0.0
first_ts_ms = None
last_ts_ms = None


def decode_packet(data: bytes):
    if len(data) != PKT_SIZE:
        print(f"Unexpected packet size: {len(data)} (expected {PKT_SIZE})")
        return

    event_id, sample_count = struct.unpack_from(PKT_HEADER_FMT, data, 0)

    if sample_count > BLE_IMU_SAMPLES_PER_PKT:
        print(f"Invalid sample_count: {sample_count}")
        return

    offset = PKT_HEADER_SIZE

    for i in range(sample_count):
        ax, ay, az, gx, gy, gz, ts_ms, seq = struct.unpack_from(SAMPLE_FMT, data, offset)
        offset += SAMPLE_SIZE

        process_sample(ax, ay, az, gx, gy, gz, ts_ms, seq, event_id)


def process_sample(ax, ay, az, gx, gy, gz, ts_ms, seq, event_id):
    global current_event_id, current_event_samples
    global max_accel_g, max_gyro_dps, first_ts_ms, last_ts_ms

    if current_event_id is None:
        start_new_event(event_id)
    elif event_id != current_event_id:
        finalize_event()
        start_new_event(event_id)

    ax_g = ax / ACC_LSB_PER_G
    ay_g = ay / ACC_LSB_PER_G
    az_g = az / ACC_LSB_PER_G

    gx_dps = gx * GYRO_DPS_PER_LSB
    gy_dps = gy * GYRO_DPS_PER_LSB
    gz_dps = gz * GYRO_DPS_PER_LSB

    accel_g = math.sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g)
    gyro_dps = math.sqrt(gx_dps * gx_dps + gy_dps * gy_dps + gz_dps * gz_dps)

    current_event_samples.append({
        "seq": seq,
        "ts_ms": ts_ms,
        "ax_raw": ax,
        "ay_raw": ay,
        "az_raw": az,
        "gx_raw": gx,
        "gy_raw": gy,
        "gz_raw": gz,
        "accel_g": accel_g,
        "gyro_dps": gyro_dps,
    })

    if accel_g > max_accel_g:
        max_accel_g = accel_g

    if gyro_dps > max_gyro_dps:
        max_gyro_dps = gyro_dps

    if first_ts_ms is None:
        first_ts_ms = ts_ms
    last_ts_ms = ts_ms

    print(
        f"EV:{event_id:3} SEQ:{seq:4} "
        f"ACC_RAW: ({ax:6},{ay:6},{az:6}) "
        f"GYR_RAW: ({gx:6},{gy:6},{gz:6}) "
        f"| ACC: {accel_g:6.2f} g "
        f"| GYRO: {gyro_dps:7.1f} dps "
        f"| T:{ts_ms}"
    )


def start_new_event(event_id):
    global current_event_id, current_event_samples
    global max_accel_g, max_gyro_dps, first_ts_ms, last_ts_ms

    current_event_id = event_id
    current_event_samples = []
    max_accel_g = 0.0
    max_gyro_dps = 0.0
    first_ts_ms = None
    last_ts_ms = None

    print(f"\n--- START EVENT {event_id} ---")


def finalize_event():
    global current_event_id, current_event_samples
    global max_accel_g, max_gyro_dps, first_ts_ms, last_ts_ms

    if current_event_id is None or len(current_event_samples) == 0:
        return

    duration_ms = 0
    if first_ts_ms is not None and last_ts_ms is not None:
        duration_ms = last_ts_ms - first_ts_ms

    seqs = [s["seq"] for s in current_event_samples]
    missing_packets = 0

    for i in range(1, len(seqs)):
        expected = seqs[i - 1] + 1
        if seqs[i] != expected:
            missing_packets += (seqs[i] - expected)

    print(f"\n=== EVENT {current_event_id} SUMMARY ===")
    print(f"Samples received:  {len(current_event_samples)}")
    print(f"Duration:          {duration_ms} ms")
    print(f"Max acceleration:  {max_accel_g:.2f} g")
    print(f"Max gyro speed:    {max_gyro_dps:.2f} dps")
    print(f"Missing samples:   {missing_packets}")
    print("=================================\n")


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

        try:
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            finalize_event()


if __name__ == "__main__":
    asyncio.run(main())