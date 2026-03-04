// utils/csvExport.ts

import { computeMagnitude, computeDerivative } from './swingAnalysis'
import { detectSwingEvents } from './impactDetection'
import { computeAllMetrics } from './metrics'

export function generateCSV(
  time: number[],
  accel: { x: number; y: number; z: number }[],
  gyro: { x: number; y: number; z: number }[]
): string {

  const dt = time[1] - time[0]

  const events = detectSwingEvents(time, gyro)
  const metrics = computeAllMetrics(time, accel, gyro)

  const omegaMag = computeMagnitude(gyro)
  const accelMag = computeMagnitude(accel)
  const energy = omegaMag.map(w => w * w)
  const jerk = computeDerivative(accelMag, dt)

  let csv = ""

  // --- METADATA ---
  csv += "Section,Metadata\n"
  csv += `SamplingRate,${1 / dt}\n`
  csv += `TotalSamples,${time.length}\n\n`

  // --- EVENTS ---
  csv += "Section,Events\n"
  csv += `StartIndex,${events.startIndex}\n`
  csv += `TransitionIndex,${events.transitionIndex}\n`
  csv += `ImpactIndex,${events.impactIndex}\n\n`

  // --- SUMMARY ---
  csv += "Section,SummaryMetrics\n"
  metrics.forEach(m => {
    csv += `${m.name},${m.value},${m.unit}\n`
  })
  csv += "\n"

  // --- RAW DATA ---
  csv += "Section,RawData\n"
  csv += "Time,GyroX,GyroY,GyroZ,AccX,AccY,AccZ,OmegaMag,AccMag,Energy,Jerk\n"

  for (let i = 0; i < time.length; i++) {
    csv += [
      time[i],
      gyro[i].x,
      gyro[i].y,
      gyro[i].z,
      accel[i].x,
      accel[i].y,
      accel[i].z,
      omegaMag[i],
      accelMag[i],
      energy[i],
      jerk[i]
    ].join(",") + "\n"
  }

  return csv
}