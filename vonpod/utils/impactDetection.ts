// utils/impactDetection.ts

export interface SwingEvents {
  startIndex: number
  transitionIndex: number
  impactIndex: number
}

export function detectSwingEvents(
  time: number[],
  gyro: { x: number; y: number; z: number }[]
): SwingEvents {

  const omegaMag = gyro.map(g =>
    Math.sqrt(g.x * g.x + g.y * g.y + g.z * g.z)
  )

  // --- Start detection ---
  const movementThreshold = 1.0
  let startIndex = omegaMag.findIndex(v => v > movementThreshold)
  if (startIndex < 0) startIndex = 0

  // --- Dominant axis ---
  const axis = getDominantAxis(gyro)

  // --- Transition detection (sign change) ---
  const signal = gyro.map(g => g[axis])
  let transitionIndex = findSignChange(signal)
  if (transitionIndex < 0) transitionIndex = startIndex

  // --- Impact detection (peak angular velocity) ---
  const impactIndex = omegaMag.indexOf(Math.max(...omegaMag))

  return {
    startIndex,
    transitionIndex,
    impactIndex
  }
}

function getDominantAxis(
  gyro: { x: number; y: number; z: number }[]
): 'x' | 'y' | 'z' {

  const rms = (arr: number[]) =>
    Math.sqrt(arr.reduce((a, b) => a + b * b, 0) / arr.length)

  const rmsX = rms(gyro.map(g => g.x))
  const rmsY = rms(gyro.map(g => g.y))
  const rmsZ = rms(gyro.map(g => g.z))

  const max = Math.max(rmsX, rmsY, rmsZ)

  if (max === rmsX) return 'x'
  if (max === rmsY) return 'y'
  return 'z'
}

function findSignChange(signal: number[]): number {
  for (let i = 1; i < signal.length; i++) {
    if (signal[i - 1] < 0 && signal[i] > 0) {
      return i
    }
  }
  return -1
}