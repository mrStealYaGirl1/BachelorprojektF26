// utils/swingAnalysis.ts

export function computeMagnitude(
  data: { x: number; y: number; z: number }[]
): number[] {
  return data.map(v =>
    Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
  )
}

export function computeDerivative(
  signal: number[],
  dt: number
): number[] {

  const derivative: number[] = []

  for (let i = 1; i < signal.length - 1; i++) {
    derivative.push((signal[i + 1] - signal[i - 1]) / (2 * dt))
  }

  derivative.unshift(derivative[0])
  derivative.push(derivative[derivative.length - 1])

  return derivative
}

export function rms(signal: number[]): number {
  const sum = signal.reduce((a, b) => a + b * b, 0)
  return Math.sqrt(sum / signal.length)
}

export function std(signal: number[]): number {
  const mean = signal.reduce((a, b) => a + b, 0) / signal.length
  const variance =
    signal.reduce((a, b) => a + (b - mean) ** 2, 0) / signal.length
  return Math.sqrt(variance)
}