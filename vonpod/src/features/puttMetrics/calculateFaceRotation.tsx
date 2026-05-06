import type { MetaPacket } from '../../lib/imuPackets'
import type { StoredImuEvent } from '../../lib/imuStorage'

const MICROSECONDS_TO_MILLISECONDS = 1000
const GYRO_DPS_PER_LSB = 2000 / 32768
const MAX_REASONABLE_SAMPLE_GAP_MS = 30

export type FaceRotationResult = {
  faceOpenAtTopDeg: number
  faceAngleAtImpactDeg: number
  faceClosureFromTopToImpactDeg: number
  maxFaceOpenDeg: number
  backswingMs: number
  downswingMs: number
}

function isPositiveFinite(value: number): boolean {
  return Number.isFinite(value) && value > 0
}

function toAbsoluteTsMs(meta: MetaPacket, phaseUs: number, eventStartSampleTsMs: number): number {
  return (phaseUs - meta.eventStartUs) / MICROSECONDS_TO_MILLISECONDS + eventStartSampleTsMs
}

function getSortedSamplesBySeq(event: StoredImuEvent): StoredImuEvent['samples'] {
  return [...event.samples].sort((a, b) => a.seq - b.seq)
}

function estimateGyroYBiasDps(
  samples: StoredImuEvent['samples'],
  backswingStartTsMs: number,
  fallbackWindowMs: number
): number {
  const baselineSamples = samples.filter(
    (sample) => sample.tsMs >= backswingStartTsMs - fallbackWindowMs && sample.tsMs < backswingStartTsMs
  )

  if (baselineSamples.length === 0) {
    return 0
  }

  const meanGyLsb =
    baselineSamples.reduce((sum, sample) => sum + sample.gy, 0) / baselineSamples.length

  return meanGyLsb * GYRO_DPS_PER_LSB
}

function integrateGyroYRotationDeg(
  samples: StoredImuEvent['samples'],
  startTsMs: number,
  endTsMs: number,
  samplePeriodMs: number,
  gyroYBiasDps: number
): number {
  if (endTsMs <= startTsMs || samples.length < 2) {
    return 0
  }

  let integratedAngleDeg = 0

  for (let i = 0; i < samples.length - 1; i += 1) {
    const current = samples[i]
    const next = samples[i + 1]

    const segmentStartMs = Math.max(current.tsMs, startTsMs)
    const segmentEndMs = Math.min(next.tsMs, endTsMs)
    if (segmentEndMs <= segmentStartMs) {
      continue
    }

    const rawDtMs = next.tsMs - current.tsMs
    const dtMs =
      rawDtMs > 0 && rawDtMs <= MAX_REASONABLE_SAMPLE_GAP_MS ? rawDtMs : samplePeriodMs
    if (!isPositiveFinite(dtMs)) {
      continue
    }

    const overlapRatio = (segmentEndMs - segmentStartMs) / dtMs
    if (overlapRatio <= 0) {
      continue
    }

    const currentGyDps = current.gy * GYRO_DPS_PER_LSB - gyroYBiasDps
    const nextGyDps = next.gy * GYRO_DPS_PER_LSB - gyroYBiasDps
    const meanGyDps = (currentGyDps + nextGyDps) / 2

    integratedAngleDeg += meanGyDps * ((segmentEndMs - segmentStartMs) / 1000)
  }

  return integratedAngleDeg
}

export function calculateFaceRotationFromEvent(event: StoredImuEvent): FaceRotationResult | null {
  const meta = event.meta
  if (!meta || event.samples.length < 2) {
    return null
  }

  const { backswingStartUs, forwardStartUs, impactUs, sampleRateHz } = meta
  if (!isPositiveFinite(backswingStartUs) || !isPositiveFinite(forwardStartUs) || !isPositiveFinite(impactUs)) {
    return null
  }

  if (!(backswingStartUs < forwardStartUs && forwardStartUs < impactUs)) {
    return null
  }

  if (!isPositiveFinite(sampleRateHz)) {
    return null
  }

  const samples = getSortedSamplesBySeq(event)
  const eventStartSampleTsMs = samples[0].tsMs

  const backswingStartTsMs = toAbsoluteTsMs(meta, backswingStartUs, eventStartSampleTsMs)
  const forwardStartTsMs = toAbsoluteTsMs(meta, forwardStartUs, eventStartSampleTsMs)
  const impactTsMs = toAbsoluteTsMs(meta, impactUs, eventStartSampleTsMs)

  if (!(backswingStartTsMs < forwardStartTsMs && forwardStartTsMs < impactTsMs)) {
    return null
  }

  const samplePeriodMs = 1000 / sampleRateHz
  const gyroYBiasDps = estimateGyroYBiasDps(samples, backswingStartTsMs, 150)

  const backswingRotationDeg = integrateGyroYRotationDeg(
    samples,
    backswingStartTsMs,
    forwardStartTsMs,
    samplePeriodMs,
    gyroYBiasDps
  )

  const downswingRotationDeg = integrateGyroYRotationDeg(
    samples,
    forwardStartTsMs,
    impactTsMs,
    samplePeriodMs,
    gyroYBiasDps
  )

  const faceOpenAtTopDeg = backswingRotationDeg
  const faceAngleAtImpactDeg = backswingRotationDeg + downswingRotationDeg
  const faceClosureFromTopToImpactDeg = faceOpenAtTopDeg - faceAngleAtImpactDeg
  const maxFaceOpenDeg = Math.max(faceOpenAtTopDeg, faceAngleAtImpactDeg)

  return {
    faceOpenAtTopDeg,
    faceAngleAtImpactDeg,
    faceClosureFromTopToImpactDeg,
    maxFaceOpenDeg,
    backswingMs: (forwardStartUs - backswingStartUs) / MICROSECONDS_TO_MILLISECONDS,
    downswingMs: (impactUs - forwardStartUs) / MICROSECONDS_TO_MILLISECONDS,
  }
}

export function getLatestValidFaceRotation(events: StoredImuEvent[]): FaceRotationResult | null {
  const sortedByNewest = [...events].sort(
    (a, b) => new Date(b.savedAt).getTime() - new Date(a.savedAt).getTime()
  )

  for (const event of sortedByNewest) {
    const faceRotation = calculateFaceRotationFromEvent(event)
    if (faceRotation) {
      return faceRotation
    }
  }

  return null
}