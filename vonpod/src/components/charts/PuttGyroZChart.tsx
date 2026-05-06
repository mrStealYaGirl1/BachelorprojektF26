import React, { useMemo, useState } from 'react'
import { StyleSheet, Text, View, useWindowDimensions } from 'react-native'
import Svg, { Path } from 'react-native-svg'
import type { ImuSample, MetaPacket } from '../../lib/imuPackets'

const CHART_HEIGHT = 180
const GYRO_Z_DPS_PER_LSB = 2000 / 32768
const AXIS_TICK_MS = 250

type ChartLabel = {
  key: string
  left: number
  label: string
  color: string
}

type PuttGyroZChartPoint = {
  x: number
  y: number
}

type TimeSeriesPoint = {
  timeMs: number
  value: number
}

function metaTimeToSampleTsMs(meta: MetaPacket, eventStartSampleTsMs: number, metaTimeUs: number): number {
  return (metaTimeUs - meta.eventStartUs) / 1000 + eventStartSampleTsMs
}

function buildUniqueChartLabels(labels: ChartLabel[]): ChartLabel[] {
  const sortedLabels = [...labels].sort((leftLabel, rightLabel) => leftLabel.left - rightLabel.left)
  const mergedLabels: ChartLabel[] = []

  for (const label of sortedLabels) {
    const lastLabel = mergedLabels[mergedLabels.length - 1]

    if (lastLabel && Math.abs(lastLabel.left - label.left) < 8) {
      lastLabel.label = `${lastLabel.label} / ${label.label}`
      lastLabel.color = label.color === '#c62828' ? label.color : lastLabel.color
      continue
    }

    mergedLabels.push(label)
  }

  return mergedLabels
}

function smoothChartValues(values: number[]): number[] {
  if (values.length <= 2) {
    return values
  }

  const radius = 2
  return values.map((value, index) => {
    const startIndex = Math.max(0, index - radius)
    const endIndex = Math.min(values.length - 1, index + radius)
    let total = 0

    for (let sampleIndex = startIndex; sampleIndex <= endIndex; sampleIndex += 1) {
      total += values[sampleIndex]
    }

    return total / (endIndex - startIndex + 1)
  })
}

function integrateGyroSeries(samples: Array<{ timeMs: number; value: number }>): TimeSeriesPoint[] {
  if (samples.length === 0) {
    return []
  }

  const sortedSamples = [...samples].sort((leftSample, rightSample) => leftSample.timeMs - rightSample.timeMs)
  const integratedSeries: TimeSeriesPoint[] = [{ timeMs: sortedSamples[0].timeMs, value: 0 }]

  let accumulatedAngleDeg = 0

  for (let index = 1; index < sortedSamples.length; index += 1) {
    const previousSample = sortedSamples[index - 1]
    const currentSample = sortedSamples[index]
    const deltaTimeSec = Math.max(0, currentSample.timeMs - previousSample.timeMs) / 1000
    const deltaAngleDeg = ((previousSample.value + currentSample.value) / 2) * deltaTimeSec
    accumulatedAngleDeg += deltaAngleDeg
    integratedSeries.push({ timeMs: currentSample.timeMs, value: accumulatedAngleDeg })
  }

  return integratedSeries
}

function fitChartValues(
  samples: Array<{ timeMs: number; value: number }>,
  maxPoints: number,
  totalDurationMs: number
): Array<{ ratio: number; value: number }> {
  if (samples.length === 0) {
    return []
  }

  const sortedSamples = [...samples].sort((leftSample, rightSample) => leftSample.timeMs - rightSample.timeMs)
  const sourceValues = sortedSamples.map((sample) => sample.value)
  const firstPass = smoothChartValues(sourceValues)
  const secondPass = smoothChartValues(firstPass)

  const smoothedSeries = sortedSamples.map((sample, index) => ({
    timeMs: sample.timeMs,
    value: secondPass[index],
  }))

  const pointCount = Math.max(2, Math.min(maxPoints, smoothedSeries.length))
  if (pointCount <= 1) {
    return [{ ratio: 0, value: smoothedSeries[0].value }]
  }

  const fitted: Array<{ ratio: number; value: number }> = []
  let sourceIndex = 0
  const safeTotalDurationMs = Math.max(1, totalDurationMs)

  for (let index = 0; index < pointCount; index += 1) {
    const ratio = index / (pointCount - 1)
    const targetTimeMs = ratio * safeTotalDurationMs

    while (
      sourceIndex < smoothedSeries.length - 2 &&
      smoothedSeries[sourceIndex + 1].timeMs <= targetTimeMs
    ) {
      sourceIndex += 1
    }

    const leftSample = smoothedSeries[sourceIndex]
    const rightSample = smoothedSeries[Math.min(sourceIndex + 1, smoothedSeries.length - 1)]
    const spanMs = Math.max(1, rightSample.timeMs - leftSample.timeMs)
    const blend = Math.max(0, Math.min(1, (targetTimeMs - leftSample.timeMs) / spanMs))
    const interpolatedValue = leftSample.value * (1 - blend) + rightSample.value * blend

    fitted.push({ ratio, value: interpolatedValue })
  }

  return fitted
}

function buildSmoothPath(points: PuttGyroZChartPoint[]): string {
  if (points.length === 0) {
    return ''
  }

  if (points.length === 1) {
    return `M ${points[0].x} ${points[0].y}`
  }

  const pathParts: string[] = [`M ${points[0].x} ${points[0].y}`]

  for (let index = 0; index < points.length - 1; index += 1) {
    const previousPoint = points[index - 1] ?? points[index]
    const currentPoint = points[index]
    const nextPoint = points[index + 1]
    const afterNextPoint = points[index + 2] ?? nextPoint

    const controlPoint1X = currentPoint.x + (nextPoint.x - previousPoint.x) / 6
    const controlPoint1Y = currentPoint.y + (nextPoint.y - previousPoint.y) / 6
    const controlPoint2X = nextPoint.x - (afterNextPoint.x - currentPoint.x) / 6
    const controlPoint2Y = nextPoint.y - (afterNextPoint.y - currentPoint.y) / 6

    pathParts.push(
      `C ${controlPoint1X} ${controlPoint1Y}, ${controlPoint2X} ${controlPoint2Y}, ${nextPoint.x} ${nextPoint.y}`
    )
  }

  return pathParts.join(' ')
}

export default function PuttGyroZChart({
  samples,
  meta,
  compareSamples,
  sharedAxisDurationMs,
}: {
  samples: ImuSample[]
  meta?: MetaPacket
  compareSamples?: Array<{ samples: ImuSample[]; meta?: MetaPacket }>
  // Optional shared axis max for session overlays (longest putt duration).
  sharedAxisDurationMs?: number
}) {
  const { width } = useWindowDimensions()
  const [measuredWidth, setMeasuredWidth] = useState<number | null>(null)

  const chartWidth = measuredWidth ?? Math.max(240, width - 60)
  const chartHeight = CHART_HEIGHT
  const chartPaddingLeft = 8
  const chartPaddingRight = 10
  const chartPaddingTop = 22
  const chartPaddingBottom = 22

  const { markers, xTicks, zeroLineTop, smoothPath, overlayPaths } = useMemo(() => {
    if (!meta || samples.length === 0) {
      return {
        markers: [] as ChartLabel[],
        xTicks: [] as Array<{ key: string; left: number; label: string }>,
        zeroLineTop: null as number | null,
        smoothPath: '',
        overlayPaths: [] as string[],
      }
    }

    const eventStartSampleTsMs = samples[0].tsMs
    const startTsMs = metaTimeToSampleTsMs(meta, eventStartSampleTsMs, meta.backswingStartUs)
    const followWindowEndUs = meta.followStartUs + 200000
    const endTsMs = metaTimeToSampleTsMs(meta, eventStartSampleTsMs, followWindowEndUs)

    const selectedSamples = samples.filter((sample) => sample.tsMs >= startTsMs && sample.tsMs <= endTsMs)
    const chartSamples = selectedSamples.length > 1 ? selectedSamples : samples

    const totalDurationMs = Math.max(1, endTsMs - startTsMs)
    const baseAxisDurationMs = Math.max(totalDurationMs, sharedAxisDurationMs ?? 0)
    const roundedAxisDurationMs = Math.max(AXIS_TICK_MS, Math.ceil(baseAxisDurationMs / AXIS_TICK_MS) * AXIS_TICK_MS)

    const gyroSeries = chartSamples.map((sample) => ({
      timeMs: sample.tsMs - startTsMs,
      value: sample.gz * GYRO_Z_DPS_PER_LSB,
    }))
    const angleSeries = integrateGyroSeries(gyroSeries)
    const fittedValues = fitChartValues(angleSeries, 72, totalDurationMs)
    const fitOnlyValues = fittedValues.map((entry) => entry.value)

    const overlaySeries = (compareSamples ?? [])
      .filter((entry) => entry.meta && entry.samples.length > 0)
      .map((entry) => {
        const overlayMeta = entry.meta as MetaPacket
        const overlayEventStartSampleTsMs = entry.samples[0].tsMs
        const overlayStartTsMs = metaTimeToSampleTsMs(
          overlayMeta,
          overlayEventStartSampleTsMs,
          overlayMeta.backswingStartUs
        )
        const overlayFollowWindowEndUs = overlayMeta.followStartUs + 200000
        const overlayEndTsMs = metaTimeToSampleTsMs(
          overlayMeta,
          overlayEventStartSampleTsMs,
          overlayFollowWindowEndUs
        )

        const overlaySelectedSamples = entry.samples.filter(
          (sample) => sample.tsMs >= overlayStartTsMs && sample.tsMs <= overlayEndTsMs
        )
        const overlayChartSamples =
          overlaySelectedSamples.length > 1 ? overlaySelectedSamples : entry.samples

        const overlayDurationMs = Math.max(1, overlayEndTsMs - overlayStartTsMs)
        const overlayGyroSeries = overlayChartSamples.map((sample) => ({
          timeMs: sample.tsMs - overlayStartTsMs,
          value: sample.gz * GYRO_Z_DPS_PER_LSB,
        }))
        const overlayAngleSeries = integrateGyroSeries(overlayGyroSeries)
        const overlayFittedValues = fitChartValues(overlayAngleSeries, 72, overlayDurationMs)

        return {
          durationMs: overlayDurationMs,
          fittedValues: overlayFittedValues,
        }
      })

    const allValues = [
      ...fitOnlyValues,
      ...overlaySeries.flatMap((series) => series.fittedValues.map((entry) => entry.value)),
    ]
    const minChartValue = Math.min(...allValues)
    const maxChartValue = Math.max(...allValues)
    const range = Math.max(1, maxChartValue - minChartValue)

    const plotWidth = chartWidth - chartPaddingLeft - chartPaddingRight
    const plotHeight = chartHeight - chartPaddingTop - chartPaddingBottom

    const clampX = (value: number): number => Math.max(chartPaddingLeft, Math.min(chartPaddingLeft + plotWidth, value))
    const mapTimeToLeft = (timeMs: number): number => {
      const clampedMs = Math.max(0, Math.min(roundedAxisDurationMs, timeMs))
      const ratio = clampedMs / roundedAxisDurationMs
      return clampX(chartPaddingLeft + ratio * plotWidth)
    }

    const mappedPoints = fittedValues.map(({ ratio, value }) => {
      const normalized = (value - minChartValue) / range
      const x = mapTimeToLeft(ratio * totalDurationMs)
      const y = chartPaddingTop + (1 - normalized) * plotHeight
      return { x, y }
    })

    const smoothPath = buildSmoothPath(mappedPoints)

    const overlayPaths = overlaySeries
      .map((series) => {
        const overlayPoints = series.fittedValues.map(({ ratio, value }) => {
          const normalized = (value - minChartValue) / range
          const x = mapTimeToLeft(ratio * series.durationMs)
          const y = chartPaddingTop + (1 - normalized) * plotHeight
          return { x, y }
        })

        return buildSmoothPath(overlayPoints)
      })
      .filter((path) => path.length > 0)

    const inversionPeak = angleSeries.reduce((bestPoint, currentPoint) =>
      currentPoint.value > bestPoint.value ? currentPoint : bestPoint
    )

    const phaseMarkers = buildUniqueChartLabels([
      {
        key: 'backswing-start',
        label: 'BS start',
        color: '#7f7f7f',
        left: mapTimeToLeft(metaTimeToSampleTsMs(meta, eventStartSampleTsMs, meta.backswingStartUs) - startTsMs),
      },
      {
        key: 'forward-start',
        label: 'Inversion',
        color: '#7f7f7f',
        left: mapTimeToLeft(inversionPeak.timeMs),
      },
      {
        key: 'impact',
        label: 'Impact',
        color: '#c62828',
        left: mapTimeToLeft(metaTimeToSampleTsMs(meta, eventStartSampleTsMs, meta.impactUs) - startTsMs),
      },
      {
        key: 'follow-start',
        label: 'Follow',
        color: '#7f7f7f',
        left: mapTimeToLeft(metaTimeToSampleTsMs(meta, eventStartSampleTsMs, meta.followStartUs) - startTsMs),
      },
      {
        key: 'window-end',
        label: 'End',
        color: '#7f7f7f',
        left: mapTimeToLeft(totalDurationMs),
      },
    ])

    const xTickCount = Math.floor(roundedAxisDurationMs / AXIS_TICK_MS) + 1
    const xTicksData = Array.from({ length: xTickCount }, (_, index) => {
      const tickMs = index * AXIS_TICK_MS
      return {
        key: `x-${index}`,
        left: mapTimeToLeft(tickMs),
        label: (tickMs / 1000).toFixed(2),
      }
    })

    const zeroLineTop =
      minChartValue <= 0 && maxChartValue >= 0
        ? chartPaddingTop + ((maxChartValue - 0) / range) * plotHeight
        : null

    return {
      markers: phaseMarkers,
      xTicks: xTicksData,
      zeroLineTop,
      smoothPath,
      overlayPaths,
    }
  }, [
    chartHeight,
    chartPaddingBottom,
    chartPaddingLeft,
    chartPaddingRight,
    chartPaddingTop,
    chartWidth,
    meta,
    samples,
    compareSamples,
    sharedAxisDurationMs,
  ])

  if (!meta || samples.length === 0) {
    return <Text style={styles.puttChartEmptyText}>Ingen data til grafen.</Text>
  }

  return (
    <View style={styles.puttChartCard}>
      <View style={styles.puttChartHeaderRow}>
        <Text style={styles.puttChartTitle}>Tempo</Text>
      </View>

      <View
        style={[styles.puttChartPlot, { width: '100%', height: chartHeight }]}
        onLayout={(event) => {
          const nextWidth = Math.round(event.nativeEvent.layout.width)
          if (nextWidth > 0 && nextWidth !== measuredWidth) {
            setMeasuredWidth(nextWidth)
          }
        }}
      >
        <View style={styles.puttChartClipper}>
          <Svg width={measuredWidth ?? chartWidth} height={chartHeight} style={StyleSheet.absoluteFillObject}>
            {zeroLineTop !== null && (
              <Path
                d={`M ${chartPaddingLeft} ${zeroLineTop} H ${chartWidth - chartPaddingRight}`}
                stroke="#bdbdbd"
                strokeWidth={1}
                fill="none"
              />
            )}
            {overlayPaths.map((path, index) => (
              <Path
                key={`overlay-${index}`}
                d={path}
                stroke="#7e7e7e"
                strokeWidth={0.5}
                fill="none"
                strokeLinecap="round"
                strokeLinejoin="round"
              />
            ))}
            {smoothPath.length > 0 && (
              <Path
                d={smoothPath}
                stroke="#d53f3f"
                strokeWidth={1.5}
                fill="none"
                strokeLinecap="round"
                strokeLinejoin="round"
              />
            )}
          </Svg>
        </View>

        <View pointerEvents="none" style={styles.puttChartMarkerLayer}>
          {markers.map((marker) => (
            <View key={marker.key} style={[styles.puttChartPhaseMarker, { left: marker.left }]}> 
              <Text numberOfLines={1} style={[styles.puttChartPhaseLabel, { color: marker.color }]}> 
                {marker.label}
              </Text>
              <View style={[styles.puttChartPhaseLine, { backgroundColor: marker.color }]} />
            </View>
          ))}
        </View>

        {xTicks.map((tick) => (
          <Text key={tick.key} style={[styles.puttChartXAxisLabel, { left: tick.left }]}> 
            {tick.label}
          </Text>
        ))}
      </View>
    </View>
  )
}

const styles = StyleSheet.create({
  puttChartCard: {
    borderRadius: 16,
    backgroundColor: '#ffffff',
    borderWidth: 1,
    borderColor: '#dedede',
    paddingHorizontal: 12,
    paddingTop: 8,
    paddingBottom: 10,
    marginTop: 6,
    boxShadow: 'rgba(100, 100, 111, 0.18) 0px 1px 6px 0px',
  },
  puttChartHeaderRow: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    marginBottom: 8,
  },
  puttChartTitle: {
    fontSize: 13,
    fontFamily: 'Montserrat_600SemiBold',
    color: '#1f1f1f',
  },
  puttChartPlot: {
    position: 'relative',
    backgroundColor: '#fff',
    borderRadius: 12,
  },
  puttChartClipper: {
    ...StyleSheet.absoluteFillObject,
    overflow: 'hidden',
    borderRadius: 12,
  },
  puttChartMarkerLayer: {
    ...StyleSheet.absoluteFillObject,
    overflow: 'visible',
    zIndex: 2,
  },
  puttChartPhaseMarker: {
    position: 'absolute',
    top: 0,
    bottom: 10,
    width: 36,
    zIndex: 2,
    transform: [{ translateX: -18 }],
    alignItems: 'center',
  },
  puttChartPhaseLabel: {
    fontSize: 7,
    fontFamily: 'Montserrat_400Regular',
    marginBottom: 3,
    backgroundColor: 'transparent',
    width: '100%',
    textAlign: 'center',
  },
  puttChartPhaseLine: {
    width: 0.5,
    flex: 1,
    opacity: 0.9,
    borderRadius: 2,
  },
  puttChartXAxisLabel: {
    position: 'absolute',
    bottom: 2,
    marginLeft: -12,
    width: 24,
    textAlign: 'center',
    fontSize: 7,
    fontFamily: 'Montserrat_400Regular',
    color: '#8d8d8d',
  },
  puttChartFootnote: {
    fontSize: 10,
    color: '#8a8a8a',
  },
  puttChartEmptyText: {
    fontSize: 12,
    color: '#7f7f7f',
    textAlign: 'center',
    paddingVertical: 16,
  },
})