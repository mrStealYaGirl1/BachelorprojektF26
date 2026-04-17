import type { ImuSample, MetaPacket } from './imuPackets'
import {
  getHighestLocalEventId,
  loadStoredEvents,
  saveStoredEvents,
  type StoredImuEvent,
} from './imuStorage'

const SAMPLE_RATE_HZ = 200
const SAMPLE_PERIOD_MS = 5

function deterministicNoise(index: number, amplitude: number): number {
  const seed = (index * 1103515245 + 12345) & 0x7fffffff
  const normalized = seed / 0x7fffffff
  return (normalized * 2 - 1) * amplitude
}

function smoothstep(t: number): number {
  const clamped = Math.max(0, Math.min(1, t))
  return clamped * clamped * (3 - 2 * clamped)
}

function buildMockSamples(count: number): ImuSample[] {
  const addressSamples = SAMPLE_RATE_HZ // ~1 second near zero
  const backswingSamples = 120
  const downswingSamples = 60 // 2:1 ratio against backswing
  const impactNoiseSamples = 24
  const impactStart = addressSamples + backswingSamples + downswingSamples

  const samples: ImuSample[] = []

  for (let i = 0; i < count; i += 1) {
    const tsMs = i * SAMPLE_PERIOD_MS
    let gzValue = 0

    if (i < addressSamples) {
      gzValue = deterministicNoise(i, 35)
    } else if (i < addressSamples + backswingSamples) {
      const t = (i - addressSamples) / Math.max(1, backswingSamples - 1)
      const rise = smoothstep(Math.pow(t, 1.2))
      gzValue = rise * 5200 + deterministicNoise(i, 65)
    } else if (i < addressSamples + backswingSamples + downswingSamples) {
      const t = (i - addressSamples - backswingSamples) / Math.max(1, downswingSamples - 1)
      const fall = 1 - smoothstep(t)
      gzValue = fall * 5200 - t * 1800 + deterministicNoise(i, 90)
    } else if (i < impactStart + impactNoiseSamples) {
      const k = i - impactStart
      const spike = k % 3 === 0 ? 1900 : k % 2 === 0 ? -1500 : 1300
      gzValue = spike + deterministicNoise(i, 1200)
    } else {
      const t = i - (impactStart + impactNoiseSamples)
      const decay = Math.exp(-t / 120)
      const drift = (i % 28 < 14 ? 1 : -1) * 260
      gzValue = decay * drift + deterministicNoise(i, 70)
    }

    const gz = Math.round(gzValue)
    const gx = Math.round(gz * 0.18 + deterministicNoise(i + 20, 140))
    const gy = Math.round(-gz * 0.1 + deterministicNoise(i + 10, 120))

    const ax = Math.round(gx * 0.22 + deterministicNoise(i + 2, 120))
    const ay = Math.round(gy * 0.2 + deterministicNoise(i + 4, 100))
    const az = 16384 + Math.round(deterministicNoise(i + 7, 210))

    const accelG = Math.sqrt(ax * ax + ay * ay + az * az) / 16384
    const gyroDps = Math.sqrt(gx * gx + gy * gy + gz * gz) * (2000 / 32768)

    samples.push({
      seq: i,
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
  }

  return samples
}

function buildMockMeta(deviceEventId: number, sampleCount: number): MetaPacket {
  const eventStartUs = 1_000_000
  const usPerSample = Math.round(1_000_000 / SAMPLE_RATE_HZ)

  const backswingStartIndex = SAMPLE_RATE_HZ
  const forwardStartIndex = backswingStartIndex + 120
  const impactIndex = forwardStartIndex + 60
  const followStartIndex = impactIndex + 24
  const endIndex = Math.max(0, sampleCount - 1)

  const backswingStartUs = eventStartUs + backswingStartIndex * usPerSample
  const forwardStartUs = eventStartUs + forwardStartIndex * usPerSample
  const impactUs = eventStartUs + impactIndex * usPerSample
  const followStartUs = eventStartUs + followStartIndex * usPerSample
  const endUs = eventStartUs + endIndex * usPerSample

  return {
    eventId: deviceEventId,
    packetType: 1,
    swingId: deviceEventId,
    sampleRateHz: SAMPLE_RATE_HZ,
    totalSamples: sampleCount,
    preSamples: impactIndex,
    postSamples: Math.max(0, sampleCount - impactIndex),
    impactIndexInEvent: impactIndex,
    addressStartUs: eventStartUs,
    backswingStartUs,
    forwardStartUs,
    impactUs,
    followStartUs,
    endUs,
    eventStartUs,
    eventEndUs: endUs,
  }
}

export async function addMockPuttEvent(): Promise<StoredImuEvent> {
  const existing = await loadStoredEvents()

  const nextLocalEventId = getHighestLocalEventId(existing) + 1
  const deviceEventId = nextLocalEventId
  const sampleCount = 1000
  const samples = buildMockSamples(sampleCount)
  const meta = buildMockMeta(deviceEventId, sampleCount)

  const mockEvent: StoredImuEvent = {
    id: `mock-${Date.now()}-${nextLocalEventId}`,
    localEventId: nextLocalEventId,
    eventId: deviceEventId,
    savedAt: new Date().toISOString(),
    sampleCount,
    samples,
    meta,
  }

  await saveStoredEvents([...existing, mockEvent])
  return mockEvent
}
