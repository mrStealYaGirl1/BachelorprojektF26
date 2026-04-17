import React, { useEffect, useMemo, useState } from 'react'
import {
  ActivityIndicator,
  FlatList,
  SafeAreaView,
  ScrollView,
  StyleSheet,
  Text,
  View,
} from 'react-native'
import { useLocalSearchParams } from 'expo-router'
import type { ImuSample, MetaPacket } from '../../lib/imuPackets'
import { loadStoredEvents, type StoredImuEvent } from '../../lib/imuStorage'
import { loadStoredTrainingSessions, type StoredTrainingSession } from '../../lib/trainingStorage'

const CHART_HEIGHT = 200
const POINT_STEP_X = 2
const POINT_SIZE = 2
const CHART_HORIZONTAL_PADDING = 4
const CHART_VERTICAL_PADDING = 4

type ChartPoint = {
  x: number
  y: number
}

type PhaseMarker = {
  key: 'backswing' | 'forward' | 'impact'
  label: string
  color: string
  left: number
}

type Segment = {
  key: string
  left: number
  top: number
  width: number
  angleDeg: string
}

function findNearestSampleIndex(samples: ImuSample[], targetTsMs: number): number {
  let bestIndex = 0
  let bestDistance = Number.POSITIVE_INFINITY

  for (let i = 0; i < samples.length; i += 1) {
    const distance = Math.abs(samples[i].tsMs - targetTsMs)
    if (distance < bestDistance) {
      bestDistance = distance
      bestIndex = i
    }
  }

  return bestIndex
}

function buildPhaseMarkers(samples: ImuSample[], meta?: MetaPacket): PhaseMarker[] {
  if (!meta || samples.length === 0) {
    return []
  }

  const eventStartSampleTsMs = samples[0].tsMs

  const markers = [
    {
      key: 'backswing' as const,
      label: 'BS',
      color: '#1976d2',
      targetTsMs: (meta.backswingStartUs - meta.eventStartUs) / 1000 + eventStartSampleTsMs,
    },
    {
      key: 'forward' as const,
      label: 'FS',
      color: '#ef6c00',
      targetTsMs: (meta.forwardStartUs - meta.eventStartUs) / 1000 + eventStartSampleTsMs,
    },
    {
      key: 'impact' as const,
      label: 'IMP',
      color: '#c62828',
      targetTsMs: (meta.impactUs - meta.eventStartUs) / 1000 + eventStartSampleTsMs,
    },
  ]

  return markers
    .filter((marker) => Number.isFinite(marker.targetTsMs))
    .map((marker) => {
      const index = findNearestSampleIndex(samples, marker.targetTsMs)
      const left = CHART_HORIZONTAL_PADDING + index * POINT_STEP_X

      return {
        key: marker.key,
        label: marker.label,
        color: marker.color,
        left,
      }
    })
}

function GzChart({ samples, meta }: { samples: ImuSample[]; meta?: MetaPacket }) {
  const gzValues = useMemo(() => samples.map((sample) => sample.gz), [samples])
  const markers = useMemo(() => buildPhaseMarkers(samples, meta), [samples, meta])

  const { min, max, points, segments, zeroLineTop, chartWidth } = useMemo(() => {
    if (gzValues.length === 0) {
      return {
        min: 0,
        max: 0,
        points: [] as ChartPoint[],
        segments: [] as Segment[],
        zeroLineTop: null as number | null,
        chartWidth: CHART_HORIZONTAL_PADDING * 2,
      }
    }

    const minValue = Math.min(...gzValues)
    const maxValue = Math.max(...gzValues)
    const range = Math.max(1, maxValue - minValue)
    const drawableHeight = CHART_HEIGHT - CHART_VERTICAL_PADDING * 2
    const mappedPoints = gzValues.map((value, index) => {
      const normalized = (value - minValue) / range
      const y = CHART_VERTICAL_PADDING + (1 - normalized) * drawableHeight
      const x = CHART_HORIZONTAL_PADDING + index * POINT_STEP_X

      return { x, y }
    })

    const mappedSegments: Segment[] = []
    for (let i = 0; i < mappedPoints.length - 1; i += 1) {
      const start = mappedPoints[i]
      const end = mappedPoints[i + 1]
      const dx = end.x - start.x
      const dy = end.y - start.y
      const width = Math.sqrt(dx * dx + dy * dy)
      const angleDeg = `${(Math.atan2(dy, dx) * 180) / Math.PI}deg`

      mappedSegments.push({
        key: `seg-${i}`,
        left: start.x,
        top: start.y,
        width,
        angleDeg,
      })
    }

    const zeroLineTop =
      minValue <= 0 && maxValue >= 0
        ? CHART_VERTICAL_PADDING + ((maxValue - 0) / range) * drawableHeight
        : null

    const width = CHART_HORIZONTAL_PADDING * 2 + Math.max(1, gzValues.length - 1) * POINT_STEP_X

    return {
      min: minValue,
      max: maxValue,
      points: mappedPoints,
      segments: mappedSegments,
      zeroLineTop,
      chartWidth: width,
    }
  }, [gzValues])

  if (samples.length === 0) {
    return <Text style={styles.emptyText}>Ingen samples at plotte.</Text>
  }

  return (
    <View>
      <Text style={styles.chartMeta}>gz min: {min}</Text>
      <Text style={styles.chartMeta}>gz max: {max}</Text>
      <Text style={styles.chartMeta}>BS=backswing start, FS=forward start, IMP=impact</Text>

      <ScrollView horizontal showsHorizontalScrollIndicator>
        <View style={[styles.chartRow, { width: chartWidth }]}>
          {zeroLineTop !== null && <View style={[styles.zeroLine, { top: zeroLineTop }]} />}

          {markers.map((marker) => (
            <View key={marker.key} style={[styles.markerWrap, { left: marker.left }]}>
              <Text style={[styles.markerLabel, { color: marker.color }]}>{marker.label}</Text>
              <View style={[styles.markerLine, { backgroundColor: marker.color }]} />
            </View>
          ))}

          {segments.map((segment) => (
            <View
              key={segment.key}
              style={[
                styles.lineSegment,
                {
                  left: segment.left,
                  top: segment.top,
                  width: segment.width,
                  transform: [{ rotate: segment.angleDeg }],
                },
              ]}
            />
          ))}

          {points.map((point, index) => (
            <View
              key={`pt-${index}`}
              style={[
                styles.point,
                {
                  left: point.x - POINT_SIZE / 2,
                  top: point.y - POINT_SIZE / 2,
                },
              ]}
            />
          ))}
        </View>
      </ScrollView>
    </View>
  )
}

const ActivityDetails = () => {
  const params = useLocalSearchParams<{ id?: string; sessionId?: string }>()
  const [event, setEvent] = useState<StoredImuEvent | null>(null)
  const [trainingSession, setTrainingSession] = useState<StoredTrainingSession | null>(null)
  const [sessionEvents, setSessionEvents] = useState<StoredImuEvent[]>([])
  const [isLoading, setIsLoading] = useState(true)
  const [errorText, setErrorText] = useState<string | null>(null)

  useEffect(() => {
    const loadData = async () => {
      setIsLoading(true)
      setErrorText(null)

      try {
        const events = await loadStoredEvents()

        if (params.sessionId) {
          const sessions = await loadStoredTrainingSessions()
          const foundSession = sessions.find((entry) => entry.id === params.sessionId)

          setTrainingSession(foundSession ?? null)
          setEvent(null)

          if (foundSession) {
            const mappedSessionEvents = foundSession.puttEventIds
              .map((puttEventId) => events.find((entry) => entry.id === puttEventId))
              .filter((entry): entry is StoredImuEvent => !!entry)

            setSessionEvents(mappedSessionEvents)
          } else {
            setSessionEvents([])
          }

          return
        }

        const found = events.find((entry) => entry.id === params.id)
        setEvent(found ?? null)
        setTrainingSession(null)
        setSessionEvents([])
      } catch (error) {
        const message = error instanceof Error ? error.message : 'Kunne ikke indlaese event'
        setErrorText(message)
      } finally {
        setIsLoading(false)
      }
    }

    void loadData()
  }, [params.id, params.sessionId])

  if (isLoading) {
    return (
      <SafeAreaView style={styles.container}>
        <ActivityIndicator />
      </SafeAreaView>
    )
  }

  if (!event && !trainingSession) {
    return (
      <SafeAreaView style={styles.container}>
        <Text style={styles.title}>Data ikke fundet</Text>
        {!!errorText && <Text style={styles.errorText}>{errorText}</Text>}
      </SafeAreaView>
    )
  }

  if (trainingSession) {
    return (
      <SafeAreaView style={styles.container}>
        <FlatList
          data={sessionEvents}
          keyExtractor={(item) => item.id}
          ListHeaderComponent={
            <View>
              <Text style={styles.title}>Traeningssession</Text>
              <Text style={styles.subTitle}>Start: {new Date(trainingSession.startedAt).toLocaleString()}</Text>
              <Text style={styles.subTitle}>Slut: {new Date(trainingSession.endedAt).toLocaleString()}</Text>
              <Text style={styles.subTitle}>Varighed: {trainingSession.durationSeconds}s</Text>
              <Text style={styles.subTitle}>Putts: {trainingSession.puttsCount}</Text>
              <Text style={styles.sectionTitle}>Putt-events i session</Text>
            </View>
          }
          renderItem={({ item, index }) => (
            <View style={styles.sampleRow}>
              <Text style={styles.sampleText}>
                Putt #{index + 1} | eventId {item.eventId} | samples {item.sampleCount} | gemt{' '}
                {new Date(item.savedAt).toLocaleTimeString()}
              </Text>
            </View>
          )}
          ListEmptyComponent={<Text style={styles.emptyText}>Ingen putts registreret i denne session.</Text>}
          contentContainerStyle={styles.listContent}
        />

        {!!errorText && <Text style={styles.errorText}>{errorText}</Text>}
      </SafeAreaView>
    )
  }

  const singleEvent = event
  if (!singleEvent) {
    return null
  }

  return (
    <SafeAreaView style={styles.container}>
      <FlatList
        data={singleEvent.samples}
        keyExtractor={(item, index) => `${item.seq}-${index}`}
        ListHeaderComponent={
          <View>
            <Text style={styles.title}>Putt #{singleEvent.localEventId ?? '-'}</Text>
            <Text style={styles.subTitle}>Device event ID: {singleEvent.eventId}</Text>
            <Text style={styles.subTitle}>Tidspunkt: {new Date(singleEvent.savedAt).toLocaleString()}</Text>
            <Text style={styles.subTitle}>Samples: {singleEvent.sampleCount}</Text>
            <Text style={styles.sectionTitle}>gz graf</Text>
            <GzChart samples={singleEvent.samples} meta={singleEvent.meta} />
            <Text style={styles.sectionTitle}>Sample-liste</Text>
          </View>
        }
        renderItem={({ item, index }) => (
          <View style={styles.sampleRow}>
            <Text style={styles.sampleText}>
              #{index + 1} | seq {item.seq} | t {item.tsMs} | gz {item.gz} | gx {item.gx} | gy {item.gy}
            </Text>
          </View>
        )}
        ListEmptyComponent={<Text style={styles.emptyText}>Ingen samples i eventet.</Text>}
        contentContainerStyle={styles.listContent}
      />

      {!!errorText && <Text style={styles.errorText}>{errorText}</Text>}
    </SafeAreaView>
  )
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    paddingHorizontal: 16,
    paddingTop: 18,
    backgroundColor: '#f4f5f6',
    justifyContent: 'center',
    alignSelf: 'center',
    width: '90%',
  },
  title: {
    fontSize: 26,
    fontWeight: '700',
    marginBottom: 8,
  },
  subTitle: {
    fontSize: 14,
    marginBottom: 4,
    opacity: 0.85,
  },
  sectionTitle: {
    marginTop: 14,
    marginBottom: 8,
    fontSize: 16,
    fontWeight: '700',
  },
  chartMeta: {
    fontSize: 12,
    opacity: 0.7,
  },
  chartRow: {
    marginTop: 8,
    height: CHART_HEIGHT,
    position: 'relative',
    borderWidth: 1,
    borderColor: '#ccd0d4',
    borderRadius: 8,
    backgroundColor: '#ffffff',
    overflow: 'hidden',
  },
  zeroLine: {
    position: 'absolute',
    left: 0,
    right: 0,
    height: 1,
    backgroundColor: '#aab2bb',
    opacity: 0.8,
    zIndex: 1,
  },
  markerWrap: {
    position: 'absolute',
    top: 0,
    bottom: 0,
    width: 1,
    zIndex: 2,
    alignItems: 'center',
  },
  markerLine: {
    width: 2,
    flex: 1,
    opacity: 0.8,
  },
  markerLabel: {
    fontSize: 10,
    fontWeight: '700',
    marginBottom: 2,
    backgroundColor: '#ffffff',
    paddingHorizontal: 2,
  },
  lineSegment: {
    position: 'absolute',
    height: 1,
    backgroundColor: '#2f6f3f',
    zIndex: 2,
    transformOrigin: 'left center',
  },
  point: {
    position: 'absolute',
    width: POINT_SIZE,
    height: POINT_SIZE,
    borderRadius: POINT_SIZE / 2,
    backgroundColor: '#2f6f3f',
    zIndex: 3,
  },
  sampleRow: {
    borderBottomWidth: 1,
    borderBottomColor: '#d9dde2',
    paddingVertical: 8,
  },
  sampleText: {
    fontSize: 12,
  },
  listContent: {
    paddingBottom: 30,
  },
  emptyText: {
    marginTop: 10,
    opacity: 0.8,
  },
  errorText: {
    marginTop: 10,
    color: '#b00020',
  },
})

export default ActivityDetails
