import type { MetaPacket } from '../../lib/imuPackets'
import type { StoredImuEvent } from '../../lib/imuStorage'

const MICROSECONDS_TO_MILLISECONDS = 1000
const GYRO_DPS_PER_LSB = 2000 / 32768
const MAX_REASONABLE_SAMPLE_GAP_MS = 30

export type LieAngleImpactResult = {
  impactAngleDeg: number
  impactAngleUnitLabel: string
  backswingAngleDeg: number
  forwardSwingAngleDeg: number
  peakAngleDeg: number
}

function isPositiveFinite(value: number): boolean {
  return Number.isFinite(value) && value > 0
}

function toAbsoluteTsMs(meta: MetaPacket, phaseUs: number, eventStartSampleTsMs: number): number {
  return (phaseUs - meta.eventStartUs) / MICROSECONDS_TO_MILLISECONDS + eventStartSampleTsMs
}

function getSortedSamplesBySeq(event: StoredImuEvent): StoredImuEvent['samples'] {
  return [...event.samples].sort((leftSample, rightSample) => leftSample.seq - rightSample.seq)
}

function estimateGyroZBiasDps(
  samples: StoredImuEvent['samples'],
  addressStartTsMs: number,
  fallbackWindowMs: number
): number {
  const baselineSamples = samples.filter(
    (sample) => sample.tsMs >= addressStartTsMs - fallbackWindowMs && sample.tsMs < addressStartTsMs
  )

  if (baselineSamples.length === 0) {
    return 0
  }

  const meanGzLsb = baselineSamples.reduce((sum, sample) => sum + sample.gz, 0) / baselineSamples.length
  return meanGzLsb * GYRO_DPS_PER_LSB
}

function integrateGyroZRotationSeries(
  samples: StoredImuEvent['samples'],
  startTsMs: number,
  endTsMs: number,
  samplePeriodMs: number,
  gyroZBiasDps: number
): Array<{ timeMs: number; angleDeg: number }> {
  if (endTsMs <= startTsMs || samples.length < 2) {
    return []
  }

  let accumulatedAngleDeg = 0
  const angleSeries: Array<{ timeMs: number; angleDeg: number }> = [{ timeMs: startTsMs, angleDeg: 0 }]

  for (let index = 0; index < samples.length - 1; index += 1) {
    const currentSample = samples[index]
    const nextSample = samples[index + 1]

    const segmentStartMs = Math.max(currentSample.tsMs, startTsMs)
    const segmentEndMs = Math.min(nextSample.tsMs, endTsMs)
    if (segmentEndMs <= segmentStartMs) {
      continue
    }

    const rawDtMs = nextSample.tsMs - currentSample.tsMs
    const dtMs = rawDtMs > 0 && rawDtMs <= MAX_REASONABLE_SAMPLE_GAP_MS ? rawDtMs : samplePeriodMs
    if (!isPositiveFinite(dtMs)) {
      continue
    }

    const overlapRatio = (segmentEndMs - segmentStartMs) / dtMs
    if (overlapRatio <= 0) {
      continue
    }

    const currentGzDps = currentSample.gz * GYRO_DPS_PER_LSB - gyroZBiasDps
    const nextGzDps = nextSample.gz * GYRO_DPS_PER_LSB - gyroZBiasDps
    const meanGzDps = (currentGzDps + nextGzDps) / 2

    accumulatedAngleDeg += meanGzDps * ((segmentEndMs - segmentStartMs) / 1000)
    angleSeries.push({ timeMs: segmentEndMs, angleDeg: accumulatedAngleDeg })

    if (segmentEndMs >= endTsMs) {
      break
    }
  }

  return angleSeries
}

function getAngleAtTimeMs(
  angleSeries: Array<{ timeMs: number; angleDeg: number }>,
  targetTsMs: number
): number | null {
  if (angleSeries.length === 0) {
    return null
  }

  if (targetTsMs <= angleSeries[0].timeMs) {
    return angleSeries[0].angleDeg
  }

  for (let index = 0; index < angleSeries.length - 1; index += 1) {
    const leftPoint = angleSeries[index]
    const rightPoint = angleSeries[index + 1]

    if (targetTsMs < leftPoint.timeMs || targetTsMs > rightPoint.timeMs) {
      continue
    }

    const spanMs = Math.max(1, rightPoint.timeMs - leftPoint.timeMs)
    const blend = (targetTsMs - leftPoint.timeMs) / spanMs
    return leftPoint.angleDeg * (1 - blend) + rightPoint.angleDeg * blend
  }

  return angleSeries[angleSeries.length - 1].angleDeg
}

function getLieAngleImpactUnitLabel(impactAngleDeg: number): string {
  return impactAngleDeg > 0 ? 'deg · up' : 'deg · down'
}

export function calculateLieAngleImpactFromEvent(event: StoredImuEvent): LieAngleImpactResult | null {
  const meta = event.meta
  if (!meta || event.samples.length < 2) {
    return null
  }

  const { addressStartUs, impactUs, sampleRateHz } = meta
  if (!isPositiveFinite(addressStartUs) || !isPositiveFinite(impactUs)) {
    return null
  }

  if (!isPositiveFinite(sampleRateHz)) {
    return null
  }

  const samples = getSortedSamplesBySeq(event)
  const eventStartSampleTsMs = samples[0].tsMs

  const addressStartTsMs = toAbsoluteTsMs(meta, addressStartUs, eventStartSampleTsMs)
  const impactTsMs = toAbsoluteTsMs(meta, impactUs, eventStartSampleTsMs)

  if (!(addressStartTsMs < impactTsMs)) {
    return null
  }

  const samplePeriodMs = 1000 / sampleRateHz
  const gyroZBiasDps = estimateGyroZBiasDps(samples, addressStartTsMs, 150)
  const angleSeries = integrateGyroZRotationSeries(samples, addressStartTsMs, impactTsMs, samplePeriodMs, gyroZBiasDps)

  if (angleSeries.length === 0) {
    return null
  }

  const peakAngleDeg = angleSeries.reduce(
    (bestPoint, currentPoint) => (currentPoint.angleDeg > bestPoint.angleDeg ? currentPoint : bestPoint),
    angleSeries[0]
  ).angleDeg

  const impactAngleDeg = getAngleAtTimeMs(angleSeries, impactTsMs)
  if (impactAngleDeg === null || !Number.isFinite(impactAngleDeg)) {
    return null
  }

  const backswingAngleDeg = peakAngleDeg
  const forwardSwingAngleDeg = backswingAngleDeg - impactAngleDeg
  const impactOffsetDeg = forwardSwingAngleDeg - backswingAngleDeg

  return {
    impactAngleDeg: impactOffsetDeg,
    impactAngleUnitLabel: getLieAngleImpactUnitLabel(impactOffsetDeg),
    backswingAngleDeg,
    forwardSwingAngleDeg,
    peakAngleDeg,
  }
}
