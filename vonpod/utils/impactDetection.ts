// utils/impactDetection.ts

export interface SwingIndices {
  startIndex: number
  transitionIndex: number
  impactIndex: number
}

/**
 * Simple event detection based on gyro magnitude.
 * These can be refined later.
 */
export function detectSwingEvents(
  time: number[],
  gyro: { x: number; y: number; z: number }[]
): SwingIndices {

  const omegaMag = gyro.map(g =>
    Math.sqrt(g.x * g.x + g.y * g.y + g.z * g.z)
  )

  // 1️⃣ Start = first point where movement exceeds threshold
  const threshold = 1.0
  const startIndex = omegaMag.findIndex(v => v > threshold)

  // 2️⃣ Transition = zero-crossing in dominant axis
  const dominantAxis = getDominantAxis(gyro)
  const transitionIndex = findSignChange(gyro.map(g => g[dominantAxis]))

  // 3️⃣ Impact = peak angular velocity
  const impactIndex = omegaMag.indexOf(Math.max(...omegaMag))

  return {
    startIndex: startIndex > 0 ? startIndex : 0,
    transitionIndex: transitionIndex > 0 ? transitionIndex : 0,
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
  return 0
}