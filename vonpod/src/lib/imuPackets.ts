const BLE_PKT_TYPE_META = 1
const BLE_PKT_TYPE_IMU = 2

const BLE_IMU_SAMPLES_PER_PKT = 5
const IMU_HEADER_SIZE = 6
const SAMPLE_SIZE = 18
const IMU_PACKET_SIZE = IMU_HEADER_SIZE + BLE_IMU_SAMPLES_PER_PKT * SAMPLE_SIZE
const META_PACKET_SIZE = 82

const ACC_LSB_PER_G = 16384
const GYRO_DPS_PER_LSB = 2000 / 32768

const BASE64_ALPHABET = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/'
const BASE64_LOOKUP = new Map<string, number>(
  [...BASE64_ALPHABET].map((char, index) => [char, index])
)

export type ImuSample = {
  seq: number
  tsMs: number
  ax: number
  ay: number
  az: number
  gx: number
  gy: number
  gz: number
  accelG: number
  gyroDps: number
}

export type ImuPacket = {
  eventId: number
  packetType: number
  sampleCount: number
  samples: ImuSample[]
}

export type MetaPacket = {
  eventId: number
  packetType: number
  swingId: number
  sampleRateHz: number
  totalSamples: number
  preSamples: number
  postSamples: number
  impactIndexInEvent: number
  addressStartUs: number
  backswingStartUs: number
  forwardStartUs: number
  impactUs: number
  followStartUs: number
  endUs: number
  eventStartUs: number
  eventEndUs: number
}

export type DecodedBleNotification =
  | {
      kind: 'imu'
      packet: ImuPacket
    }
  | {
      kind: 'meta'
      packet: MetaPacket
    }

function base64ToBytes(base64: string): Uint8Array {
  const cleaned = base64.replace(/\s/g, '')

  if (cleaned.length === 0) {
    return new Uint8Array(0)
  }

  const output: number[] = []
  let buffer = 0
  let bits = 0

  for (let i = 0; i < cleaned.length; i += 1) {
    const char = cleaned[i]

    if (char === '=') {
      break
    }

    const value = BASE64_LOOKUP.get(char)
    if (value === undefined) {
      throw new Error(`Ugyldigt base64-tegn: ${char}`)
    }

    buffer = (buffer << 6) | value
    bits += 6

    while (bits >= 8) {
      bits -= 8
      output.push((buffer >> bits) & 0xff)
    }
  }

  return new Uint8Array(output)
}

function readUint64AsNumber(view: DataView, offset: number): number {
  const low = view.getUint32(offset, true)
  const high = view.getUint32(offset + 4, true)
  return high * 2 ** 32 + low
}

function decodeMetaPacket(view: DataView): MetaPacket {
  if (view.byteLength !== META_PACKET_SIZE) {
    throw new Error(
      `Unexpected META packet size: ${view.byteLength} (expected ${META_PACKET_SIZE})`
    )
  }

  const eventId = view.getUint16(0, true)
  const packetType = view.getUint16(2, true)

  return {
    eventId,
    packetType,
    swingId: view.getUint32(4, true),
    sampleRateHz: view.getUint16(8, true),
    totalSamples: view.getUint16(10, true),
    preSamples: view.getUint16(12, true),
    postSamples: view.getUint16(14, true),
    impactIndexInEvent: view.getUint16(16, true),
    addressStartUs: readUint64AsNumber(view, 18),
    backswingStartUs: readUint64AsNumber(view, 26),
    forwardStartUs: readUint64AsNumber(view, 34),
    impactUs: readUint64AsNumber(view, 42),
    followStartUs: readUint64AsNumber(view, 50),
    endUs: readUint64AsNumber(view, 58),
    eventStartUs: readUint64AsNumber(view, 66),
    eventEndUs: readUint64AsNumber(view, 74),
  }
}

function decodeImuPacket(view: DataView): ImuPacket {
  if (view.byteLength !== IMU_PACKET_SIZE) {
    throw new Error(`Unexpected IMU packet size: ${view.byteLength} (expected ${IMU_PACKET_SIZE})`)
  }

  const eventId = view.getUint16(0, true)
  const packetType = view.getUint16(2, true)
  const sampleCount = view.getUint16(4, true)

  if (sampleCount > BLE_IMU_SAMPLES_PER_PKT) {
    throw new Error(`Ugyldigt sample_count: ${sampleCount}`)
  }

  const samples: ImuSample[] = []
  let offset = IMU_HEADER_SIZE

  for (let i = 0; i < sampleCount; i += 1) {
    const ax = view.getInt16(offset + 0, true)
    const ay = view.getInt16(offset + 2, true)
    const az = view.getInt16(offset + 4, true)
    const gx = view.getInt16(offset + 6, true)
    const gy = view.getInt16(offset + 8, true)
    const gz = view.getInt16(offset + 10, true)
    const tsMs = view.getUint32(offset + 12, true)
    const seq = view.getUint16(offset + 16, true)

    const axG = ax / ACC_LSB_PER_G
    const ayG = ay / ACC_LSB_PER_G
    const azG = az / ACC_LSB_PER_G

    const gxDps = gx * GYRO_DPS_PER_LSB
    const gyDps = gy * GYRO_DPS_PER_LSB
    const gzDps = gz * GYRO_DPS_PER_LSB

    const accelG = Math.sqrt(axG * axG + ayG * ayG + azG * azG)
    const gyroDps = Math.sqrt(gxDps * gxDps + gyDps * gyDps + gzDps * gzDps)

    samples.push({
      seq,
      tsMs,
      ax,
      ay,
      az,
      gx,
      gy,
      gz,
      accelG,
      gyroDps,
    })

    offset += SAMPLE_SIZE
  }

  return {
    eventId,
    packetType,
    sampleCount,
    samples,
  }
}

export function decodeBleNotification(base64Value: string): DecodedBleNotification {
  const bytes = base64ToBytes(base64Value)

  if (bytes.length < 4) {
    throw new Error(`Pakke er for kort: ${bytes.length} bytes`)
  }

  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength)

  const packetType = view.getUint16(2, true)

  if (packetType === BLE_PKT_TYPE_META) {
    return {
      kind: 'meta',
      packet: decodeMetaPacket(view),
    }
  }

  if (packetType === BLE_PKT_TYPE_IMU) {
    return {
      kind: 'imu',
      packet: decodeImuPacket(view),
    }
  }

  throw new Error(`Ukendt packet_type=${packetType}, længde=${bytes.length}`)
}
