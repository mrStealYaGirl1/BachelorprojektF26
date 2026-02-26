// utils/metrics.ts

import { computeMagnitude, computeDerivative, rms, std, integrate } from './swingAnalysis'
import { detectSwingEvents } from './impactDetection'

export interface Metric {
  name: string
  value: number
  unit: string
}

export function computeAllMetrics(
  time: number[],
  accel: { x: number; y: number; z: number }[],
  gyro: { x: number; y: number; z: number }[]
): Metric[] {

  const dt = time[1] - time[0]

  const { startIndex, transitionIndex, impactIndex } =
    detectSwingEvents(time, gyro)

  const omegaMag = computeMagnitude(gyro)

  const metrics: Metric[] = []

  // 1️⃣ Tempo Ratio
  const backswingTime = time[transitionIndex] - time[startIndex]
  const downswingTime = time[impactIndex] - time[transitionIndex]
  metrics.push({
    name: 'Tempo Ratio',
    value: backswingTime / downswingTime,
    unit: 'ratio'
  })

  // 2️⃣ Total Duration
  metrics.push({
    name: 'Total Swing Duration',
    value: time[impactIndex] - time[startIndex],
    unit: 's'
  })

  // 3️⃣ Peak Angular Velocity
  metrics.push({
    name: 'Peak Angular Velocity',
    value: Math.max(...omegaMag),
    unit: 'rad/s'
  })

  // 4️⃣ Velocity Slope at Impact
  const omegaDerivative = computeDerivative(omegaMag, dt)
  metrics.push({
    name: 'Velocity Slope at Impact',
    value: omegaDerivative[impactIndex],
    unit: 'rad/s²'
  })

  // 5️⃣ Principal Axis Dominance
  const rmsX = rms(gyro.map(g => g.x))
  const rmsY = rms(gyro.map(g => g.y))
  const rmsZ = rms(gyro.map(g => g.z))
  const sorted = [rmsX, rmsY, rmsZ].sort((a, b) => b - a)
  metrics.push({
    name: 'Principal Axis Dominance',
    value: sorted[0] / sorted[1],
    unit: 'ratio'
  })

  // 6️⃣ Impact Stability
  const window = 5
  const impactWindow = omegaMag.slice(
    impactIndex - window,
    impactIndex + window
  )
  metrics.push({
    name: 'Impact Stability',
    value: std(impactWindow),
    unit: 'rad/s'
  })

  // 7️⃣ Transition Jerk RMS
  const accelMag = computeMagnitude(accel)
  const jerk = computeDerivative(accelMag, dt)
  const jerkWindow = jerk.slice(
    transitionIndex - window,
    transitionIndex + window
  )
  metrics.push({
    name: 'Transition Jerk RMS',
    value: rms(jerkWindow),
    unit: 'm/s³'
  })

  return metrics
}